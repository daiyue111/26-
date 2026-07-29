# 2026 H Vehicle-Mounted Ball Balance Car

MSPM0G3507 Keil project for the 2026 H problem. It is derived from the
working 2025 E chassis project while leaving that source directory unchanged.

Implemented foundations:

- hardware-PWM two-motor differential drive using the proven 2025 E chassis;
- left/right encoder speed PI loops;
- continuous eight-channel infrared line centroid control;
- stadium-track distance scheduling for 1.5 m straights and 0.5 m arcs;
- H2-H6 nonblocking mission state machine;
- camera packet parser with CRC, confidence, and age checks;
- alpha-beta ball position/velocity estimator;
- ball state feedback plus chassis-acceleration feedforward;
- active braking, line-loss, vision-loss, ball-limit, timeout, and encoder
  fault handling;
- Keil Watch telemetry for mission, chassis, ball, vision, and scheduler.

Open `empty_mspm0g3507_nortos_keil.uvprojx` and build target
`empty_mspm0g3507`. The default runtime is H2 so the chassis can be validated
before the camera and rod actuator are connected.

Read `H2026_BRINGUP.md` before connecting Lite-K230D or a servo. The final UART,
hardware-PWM servo pin, OLED, and two dedicated A-line sensors are deliberately
left in hardware adapter hooks until their actual modules and physical pins
are confirmed.
