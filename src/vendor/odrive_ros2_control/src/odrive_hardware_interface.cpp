// Adapted from odriverobotics/ros_odrive efe8f775 (MIT, ../LICENSE.odrive).
// Velocity-only ODrive 3.x / CAN Simple 0.5.6 integration for ros2_control.
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <set>
#include <mutex>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace odrive_ros2_control {
using Clock = std::chrono::steady_clock;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
static double now_s() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

struct Axis {
    uint32_t node_id;
    double direction;
    double vel_gain = -1., vel_integrator_gain = -1.;
    double command = 0., position = 0., velocity = 0.;
    double heartbeat = -1., feedback = -1.;
    uint32_t error = 0;
    uint8_t state = 0;
    bool commanded = false, position_valid = false;
    double last_position = 0.;
};

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using State = rclcpp_lifecycle::State;
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State&) override;
    CallbackReturn on_activate(const State&) override;
    CallbackReturn on_deactivate(const State&) override;
    CallbackReturn on_cleanup(const State&) override;
    CallbackReturn on_shutdown(const State& s) override { return on_cleanup(s); }
    CallbackReturn on_error(const State& s) override { std::lock_guard<std::recursive_mutex> guard(mutex_); fault_ = true; return on_cleanup(s); }
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    return_type prepare_command_mode_switch(const std::vector<std::string>& start,
                                            const std::vector<std::string>& stop) override;
    return_type perform_command_mode_switch(const std::vector<std::string>& start,
                                            const std::vector<std::string>& stop) override;
    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;
private:
    template<class Msg> bool send(const Axis& axis, const Msg& msg) {
        can_frame frame{};
        frame.can_id = (axis.node_id << 5) | Msg::cmd_id;
        frame.can_dlc = Msg::msg_length;
        msg.encode_buf(frame.data);
        bool ok = can_.send_can_frame(frame);
        // MCP2515 has only three TX buffers. Four axes must not burst into them.
        if (pace_us_ > 0) std::this_thread::sleep_for(std::chrono::microseconds(pace_us_));
        return ok;
    }
    void receive(const can_frame& frame);
    void drain() { for (int i=0; i<512 && can_.read_nonblocking(); ++i) {} }
    bool fresh(const Axis& a, double now) const {
        return now-a.heartbeat <= feedback_timeout_ && now-a.feedback <= feedback_timeout_;
    }
    bool stop_ready() const {
        if(stop_file_.empty()) return interface_ != "can0";
        int enabled=0;
        std::ifstream file(stop_file_);
        return (file >> enabled) && enabled==1;
    }
    bool idle_all();
    return_type fail(const std::string& reason);
    std::recursive_mutex mutex_;
    bool connected_ = false, active_ = false, fault_ = false;
    double arm_deadline_ = 0., last_iq_request_ = 0.;
    double feedback_timeout_ = .5, max_velocity_ = .6283185307179586, current_limit_ = 1.;
    int pace_us_ = 1500, lock_fd_ = -1;
    std::string interface_, lock_path_, stop_file_;
    EpollEventLoop loop_;
    SocketCanIntf can_;
    std::vector<Axis> axes_;
};

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    try {
        interface_ = info_.hardware_parameters.at("can");
        lock_path_ = info_.hardware_parameters.at("lock_path");
        auto lease=info_.hardware_parameters.find("stop_file");
        if(lease!=info_.hardware_parameters.end()) stop_file_=lease->second;
        auto get = [&](const char* key, double fallback) {
            auto it=info_.hardware_parameters.find(key);
            return it == info_.hardware_parameters.end() ? fallback : std::stod(it->second);
        };
        feedback_timeout_ = get("feedback_timeout", .5);
        max_velocity_ = get("max_velocity_rad_s", max_velocity_);
        current_limit_ = get("current_limit", 1.);
        pace_us_ = static_cast<int>(get("tx_spacing_us", 1500));
        if (!std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0. ||
            !std::isfinite(current_limit_) || current_limit_ <= 0. || current_limit_ > 20. ||
            !std::isfinite(max_velocity_) || max_velocity_ <= 0. || pace_us_ < 0 || pace_us_ > 5000)
            throw std::runtime_error("Invalid hardware parameters");
        std::set<uint32_t> ids;
        for (const auto& joint : info_.joints) {
            int id=std::stoi(joint.parameters.at("node_id"));
            double sign=std::stod(joint.parameters.at("direction"));
            if (id<0 || id>63 || !ids.insert(id).second || (sign!=1. && sign!=-1.))
                throw std::runtime_error("Invalid node ID/direction");
            if (joint.command_interfaces.size()!=1 || joint.command_interfaces[0].name!="velocity")
                throw std::runtime_error("Legacy profile supports velocity commands only");
            Axis axis{static_cast<uint32_t>(id),sign};
            auto gain=joint.parameters.find("vel_gain");
            auto integrator=joint.parameters.find("vel_integrator_gain");
            if(gain!=joint.parameters.end() && integrator!=joint.parameters.end()) {
                axis.vel_gain=std::stod(gain->second);
                axis.vel_integrator_gain=std::stod(integrator->second);
                if(!std::isfinite(axis.vel_gain) || axis.vel_gain<0. || axis.vel_gain>1. ||
                   !std::isfinite(axis.vel_integrator_gain) || axis.vel_integrator_gain<0. || axis.vel_integrator_gain>2.)
                    throw std::runtime_error("Invalid velocity gains");
            }
            axes_.push_back(axis);
        }
        if (axes_.empty()) throw std::runtime_error("No axes configured");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "%s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    lock_fd_=open(lock_path_.c_str(), O_CREAT|O_RDWR, 0600);
    if (lock_fd_<0 || flock(lock_fd_, LOCK_EX|LOCK_NB)!=0) {
        if (lock_fd_>=0) close(lock_fd_);
        lock_fd_=-1;
        RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "CAN owner lock unavailable: %s", lock_path_.c_str());
        return CallbackReturn::ERROR;
    }
    connected_=can_.init(interface_, &loop_, [this](const can_frame& f){receive(f);});
    if (!connected_) { close(lock_fd_); lock_fd_=-1; return CallbackReturn::ERROR; }
    fault_=false; active_=false;
    for(auto& a:axes_) { a.command=0.; a.commanded=false; a.position_valid=false; a.velocity=0.; a.heartbeat=a.feedback=-1.; }
    // Receive only: configuring the plugin must not start motors.
    double deadline=now_s()+1.;
    do {
        drain();
        if (can_.healthy() && std::all_of(axes_.begin(),axes_.end(),[&](const Axis& a){return fresh(a,now_s());}))
            return CallbackReturn::SUCCESS;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while(now_s()<deadline);
    on_cleanup(State{});
    RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Missing heartbeat/encoder estimates during configure");
    return CallbackReturn::ERROR;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    drain();
    if (fault_ || !can_.healthy()) return CallbackReturn::ERROR;
    for(const auto& a:axes_)
        if (!fresh(a,now_s()) || a.state!=AXIS_STATE_IDLE || (a.error!=0 && a.error!=2048))
            return CallbackReturn::ERROR;
    active_=true;
    return CallbackReturn::SUCCESS;
}

bool ODriveHardwareInterface::idle_all() {
    bool ok=true;
    arm_deadline_=now_s()+.3;
    for(auto& a:axes_) {
        a.command=0.; a.commanded=false; a.position_valid=false; a.velocity=0.;
        if (connected_) {
            Set_Axis_State_msg_t msg; msg.Axis_Requested_State=AXIS_STATE_IDLE;
            ok=send(a,msg) && ok;
        }
    }
    return ok;
}
CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    active_=false;
    return idle_all() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}
CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    active_=false;
    if(connected_) { idle_all(); can_.deinit(); connected_=false; }
    if(lock_fd_>=0) { close(lock_fd_); lock_fd_=-1; }
    return CallbackReturn::SUCCESS;
}
return_type ODriveHardwareInterface::fail(const std::string& reason) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "%s", reason.c_str());
    fault_=true; active_=false; idle_all();
    return return_type::ERROR;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> out;
    for(size_t i=0;i<axes_.size();++i) {
        out.emplace_back(info_.joints[i].name,"position", &axes_[i].position);
        out.emplace_back(info_.joints[i].name,"velocity", &axes_[i].velocity);
    }
    return out;
}
std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> out;
    for(size_t i=0;i<axes_.size();++i) out.emplace_back(info_.joints[i].name,"velocity", &axes_[i].command);
    return out;
}
return_type ODriveHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start,const std::vector<std::string>& stop) {
    size_t starting=0,stopping=0;
    for(const auto& joint:info_.joints) {
        const auto key=joint.name+"/velocity";
        starting+=std::count(start.begin(),start.end(),key);
        stopping+=std::count(stop.begin(),stop.end(),key);
    }
    // A differential base is one unit: do not arm just part of the chassis.
    return ((starting==0 || starting==axes_.size()) && (stopping==0 || stopping==axes_.size()))
        ? return_type::OK : return_type::ERROR;
}
return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start,const std::vector<std::string>& stop) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if(prepare_command_mode_switch(start,stop)!=return_type::OK) return return_type::ERROR;
    bool starting=false,stopping=false;
    for(const auto& joint:info_.joints) {
        starting |= std::find(start.begin(),start.end(),joint.name+"/velocity")!=start.end();
        stopping |= std::find(stop.begin(),stop.end(),joint.name+"/velocity")!=stop.end();
    }
    if(stopping && !idle_all()) return fail("CAN failure during stop");
    if(!starting) return return_type::OK;
    if(!stop_ready()) return fail("STOP button not ready; activation refused");
    drain();
    if(!active_ || fault_ || !can_.healthy()) return fail("Hardware not ready for activation");
    for(const auto& a:axes_)
        if(!fresh(a,now_s()) || a.state!=AXIS_STATE_IDLE || (a.error!=0 && a.error!=2048))
            return fail("Axes must be fresh and Idle before starting controller");
    for(auto& a:axes_) {
        a.command=0.; a.position_valid=false;
        Set_Input_Vel_msg_t zero; zero.Input_Vel=0.; zero.Input_Torque_FF=0.;
        Set_Limits_msg_t limits; limits.Velocity_Limit=2.; limits.Current_Limit=current_limit_;
        Set_Controller_Mode_msg_t mode; mode.Control_Mode=CONTROL_MODE_VELOCITY_CONTROL; mode.Input_Mode=INPUT_MODE_PASSTHROUGH;
        Clear_Errors_msg_t clear; clear.Identify=0;
        if(a.vel_gain>=0.) {
            Set_Vel_Gains_msg_t gains;
            gains.Vel_Gain=a.vel_gain; gains.Vel_Integrator_Gain=a.vel_integrator_gain;
            if(!send(a,gains)) return fail("CAN failure setting velocity gains");
        }
        Set_Axis_State_msg_t state; state.Axis_Requested_State=AXIS_STATE_CLOSED_LOOP_CONTROL;
        if(!send(a,zero) || !send(a,limits) || !send(a,mode) || !send(a,clear) || !send(a,state))
            return fail("CAN failure during zero-speed activation");
        a.commanded=true;
    }
    arm_deadline_=now_s()+.8;
    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time&,const rclcpp::Duration&) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if(!connected_ || fault_) return return_type::ERROR;
    drain();
    if(!can_.healthy()) return fail("CAN receive failure");
    if(std::any_of(axes_.begin(),axes_.end(),[](const Axis& a){return a.commanded;}) && !stop_ready())
        return fail("STOP button latched or flag unavailable");
    double now=now_s();
    for(const auto& a:axes_) {
        if(!fresh(a,now)) return fail("Stale feedback on axis "+std::to_string(a.node_id));
        if(a.commanded) {
            // Until the post-activation heartbeat arrives, only zero setpoints are sent.
            bool grace=now<arm_deadline_;
            if((a.error && !(grace && a.error==2048)) || (!grace && a.state!=AXIS_STATE_CLOSED_LOOP_CONTROL))
                return fail("Axis "+std::to_string(a.node_id)+" fault/state "+std::to_string(a.error)+"/"+std::to_string(a.state));
        } else if(a.state!=AXIS_STATE_IDLE && now>=arm_deadline_) {
            return fail("Unexpected enabled axis "+std::to_string(a.node_id));
        }
    }
    return return_type::OK;
}
return_type ODriveHardwareInterface::write(const rclcpp::Time&,const rclcpp::Duration&) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if(fault_) return return_type::ERROR;
    if(!active_) return return_type::OK;
    if(std::any_of(axes_.begin(),axes_.end(),[](const Axis& a){return a.commanded;}) && !stop_ready())
        return fail("STOP button latched or flag unavailable");
    bool ready=std::all_of(axes_.begin(),axes_.end(),[](const Axis& a){return a.state==AXIS_STATE_CLOSED_LOOP_CONTROL && a.error==0;});
    for(const auto& a:axes_) if(a.commanded && (!std::isfinite(a.command) || std::abs(a.command)>max_velocity_+1e-6))
        return fail("Non-finite or excessive wheel velocity");
    for(const auto& a:axes_) if(a.commanded) {
        Set_Input_Vel_msg_t msg;
        msg.Input_Vel=ready ? a.direction*a.command/(2*M_PI) : 0.;
        msg.Input_Torque_FF=0.;
        if(!send(a,msg)) return fail("CAN transmit failure");
    }
    // Diagnostics share the CAN owner and stop with the drive. Requests feed the
    // legacy firmware watchdog, so never poll an Idle/disarmed axis.
    if(ready && now_s()-last_iq_request_ >= .1) {
        for(const auto& a:axes_) if(a.commanded) {
            can_frame request{};
            request.can_id=(a.node_id << 5) | Get_Iq_msg_t::cmd_id | CAN_RTR_FLAG;
            request.can_dlc=8;
            if(!can_.send_can_frame(request)) return fail("CAN transmit failure requesting Iq");
            if(pace_us_>0) std::this_thread::sleep_for(std::chrono::microseconds(pace_us_));
        }
        last_iq_request_=now_s();
    }
    return return_type::OK;
}
void ODriveHardwareInterface::receive(const can_frame& frame) {
    if(frame.can_id & (CAN_EFF_FLAG|CAN_RTR_FLAG|CAN_ERR_FLAG)) return;
    if(frame.can_dlc!=8) return;
    uint32_t node=frame.can_id>>5,cmd=frame.can_id&31;
    for(auto& a:axes_) if(a.node_id==node) {
        if(cmd==1) { // 0.5.6: uint32 axis_error + uint8 state; newer flags differ.
            std::memcpy(&a.error,frame.data,4); a.state=frame.data[4]; a.heartbeat=now_s();
        } else if(cmd==Get_Encoder_Estimates_msg_t::cmd_id) {
            Get_Encoder_Estimates_msg_t msg; msg.decode_buf(frame.data);
            if(std::isfinite(msg.Pos_Estimate) && std::isfinite(msg.Vel_Estimate)) {
                // Legacy Hall estimates reset on enable and are invalid in Idle.
                // Preserve joint position across Idle/re-arm without inventing odometry.
                if(a.commanded && a.state==AXIS_STATE_CLOSED_LOOP_CONTROL && a.error==0) {
                    double position=a.direction*msg.Pos_Estimate*(2*M_PI);
                    if(a.position_valid) a.position+=position-a.last_position;
                    a.last_position=position; a.position_valid=true;
                    a.velocity=a.direction*msg.Vel_Estimate*(2*M_PI);
                } else { a.velocity=0.; a.position_valid=false; }
                a.feedback=now_s();
            }
        }
        // 0x1C is ADC voltage on v3.6, not Get_Torques. Ignore it.
    }
}
} // namespace odrive_ros2_control
PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
