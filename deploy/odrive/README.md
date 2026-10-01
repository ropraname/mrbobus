# Experimental Hall loop compensation

Suspended-wheel tests passed; floor validation pending. Based on upstream fw-v0.5.6 commit
`a308314ed2ca613164b81e7bbdfacc53cd1859ff`; apply the adjacent patch at
repository root. Build with `CONFIG_MRBOBUS_HALL_COMPENSATION=true` in
Firmware/tup.config for v3.6-56V. Without that flag the original control
law remains. The patch does not change the persistent config schema.

The candidate shapes velocity feedback with
`F(s)=(1.2s²+46.6s+550)/(s²+115s+550)`, unity DC gain, poles -5/-110 rad/s.
It applies only to local Hall feedback, CPR60, bandwidth30, velocity mode.
P/I remain mixed_drive. FOC phase, current control and raw-feedback safety
checks are unchanged. Two first-order Tustin sections avoid precision loss
from nearly cancelling biquad coefficients at the 8kHz control rate.

`tools/model_hall_loop.py` fits the final zero-command oscillation and
evaluates a lumped inertia/damping + PLL + PI model. Dependencies numpy,
scipy; store outputs in .local. The fitted inertia is an equivalent model,
not a physical measurement. Quantized nonlinear simulations retain residual
limit cycles; one trace does not validate changes of load, tyre friction,
Hall spacing, electrical phase or chassis dynamics.

Before any flash: save full configuration and original flash image, verify
board identity/voltage variant and disabled startup, keep STOP latched and
CAN owner stopped. Flash one board first; read back image and config. First
test is suspended, zero command, limited current. Only progress to original
current and stepped speed if oscillations decay or stay bounded within the
test envelope. No floor test until unloaded test passes and user lowers robot.
Never infer success from a build, simulation or raising an error threshold.

2026-09-29: both boards flashed and image readback compared byte-for-byte.
Binary SHA256 `be78fdddb236d9eb672edeaee6b3d6b04b2959e09f07bcdd540421751800320c`,
319360 bytes. Full original 1MiB images retained in .local/odrive-backups
and mini-PC ~/robot/firmware/images. Runtime source/build is on mini-PC
~/robot/firmware/odrive-firmware. Ubuntu ARM GCC13.2 + tup build succeeded;
legacy `make` post-build CAN-DBC generation is incompatible with the installed
modern cantools, so firmware completion was verified with `tup` and ELF size.

Trials 053322 (1A) and 053412 (20A), same mixed_drive P/I, passed +/-
.03/.06/.12m/s equivalent wheel commands with zero-command intervals.
At20A full CAN peak estimated speed .877rev/s, sampled Iq peak1.371A;
last .8s all four estimated speeds zero and encoder position unchanged.
This is a bounded free-wheel result, not validation of transient unloading
during ground contact or proof of preserved loaded turns.

Host verification: compile tools/test_hall_compensator.cpp with patched
Firmware/MotorControl include path and run. Acceptance on hardware must cover
zero-command settling, +/- velocities, stop/reversal, then loaded 90-degree
turn/return and straight/return. Preserve original firmware for rollback.

Floor follow-up 2026-09-29: trial053953 completed +90deg/return at20A
with unchanged mixed_drive gains. LIO settled89.78deg and final-1.37deg;
sampled peak Iq6.49A. Trial054533 completed .35m straight/return at .12m/s:
LIO displacement .340m forward, .0069m after return, heading changes
.23deg/.29deg. Neither trial faulted. Both boards' Hall30/mixed_drive/20A/
3rev/s settings saved and verified after reboot. Transient unloading and
re-contact while climbing an obstacle is still NOT validated.
