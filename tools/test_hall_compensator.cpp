// Compile with -I <patched ODrive>/Firmware/MotorControl. No hardware I/O.
#include "hall_velocity_compensator.hpp"
#include <cassert>
#include <cmath>
#include <complex>
#include <iostream>

int main() {
    constexpr double pi = 3.14159265358979323846;
    constexpr float dt = 1.0f / 8000;
    HallVelocityCompensator filter;
    for (float constant : {0.f, .1f, -2.f, 3.f}) {
        filter.reset();
        for (int k = 0; k < 80000; ++k)
            assert(std::abs(filter.update(constant, dt) - constant) < 1e-5f);
    }
    for (double frequency : {.5, 7., 50., 1000.}) {
        filter.reset();
        std::complex<double> measured{0, 0};
        for (int k = 0; k < 160000; ++k) {
            double angle = 2*pi*frequency*k/8000;
            float y = filter.update(std::cos(angle), dt);
            assert(std::isfinite(y));
            if (k >= 80000)
                measured += double(y)*std::exp(std::complex<double>(0, -angle))*2./80000.;
        }
        auto s = std::complex<double>(0, 16000*std::tan(pi*frequency/8000));
        auto expected = (1.2*s*s + 46.6*s + 550.)/(s*s + 115.*s + 550.);
        assert(std::abs(measured - expected) < .002);
    }
    filter.reset();
    assert(filter.update(0.f, dt) == 0.f);
    std::cout << "DC preservation, frequency response, finite output and reset: PASS\n";
}
