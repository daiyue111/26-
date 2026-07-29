# 2026 H Ball-Car Bring-up

This directory is an independent copy of the working 2025 E chassis project.
The source project and the `ec084bb` competition baseline are unchanged.

## Current Build

- Keil target: `empty_mspm0g3507`
- Default runtime: `APP_MODE_H2026`
- Default task: H2, one lap and active braking at A
- Generated image: `Objects/empty_mspm0g3507.hex`
- The existing motor, encoder, line sensor, start key, and power pins are
  unchanged.

## H2 Pure Line Profile

H2 uses continuous line following on the official field. Route distance,
gyro curve detection, and curve feedforward are not part of the H2 control
loop. The current profile uses 6/5/4 ticks for small/medium/large line error.
It validates:

- continuous line following through both semicircles;
- return-to-A marker recognition after leaving the start marker and running
  for at least 5 seconds;
- final approach and active braking.

Encoder distance remains telemetry only. Increase the three speed levels
only after this profile completes repeatable laps.

## Runtime Selection

Until the OLED/key menu is connected, set these globals with Keil Watch while
the mission is stopped:

- `gHRequestedTask`: 2, 3, 4, 5, or 6
- `gHRequestedTargetTenthMm`: task 6 target in 0.1 mm

Examples: `500` is +5 cm, `-750` is -7.5 cm. Press PA09 to start or stop.

## Vision Packet

Lite-K230D sends this seven-byte packet at 115200 baud:

```text
AA 55 XL XH CONFIDENCE FRAME_ID CRC8
```

- `X`: signed little-endian position in 0.1 mm
- `CONFIDENCE`: 0-100; firmware accepts 60 or above
- `CRC8`: polynomial 0x07 over `XL XH CONFIDENCE FRAME_ID`

Run `lite_k230d_ball_tracker.py` in CanMV IDE. The verified Lite-K230D
mapping uses IO3 as UART1 TX and IO4 as UART1 RX. Connect K230 IO3 to the
MSPM0 UART RX input and connect grounds. Connect the UART RX interrupt to:

```c
ball_vision_feed_byte(received_byte, control_scheduler_now_ms());
```

The script's ROI, circle radius/threshold, and two pixel calibration points
must be measured on the final mechanism. `openmv_ball_tracker.py` is retained
only as a legacy reference and is not for the Lite-K230D.

## Rod Actuator Port

The control stack already produces a command in millidegrees, limited to
`+/-7000`. Hardware output is intentionally isolated behind two weak hooks:

```c
void rod_actuator_hw_init(void);
void rod_actuator_hw_write_mdeg(int16_t angleMdeg);
```

Implement strong versions after the servo type, timer channel, output pin,
neutral pulse, and mechanical sign are confirmed. Use hardware PWM. Do not
generate the 1-2 ms servo pulse with the 1 ms application scheduler.

## First Bench Gate

1. Keep the drive wheels off the ground.
2. Run Lite-K230D and verify `gHVisionGoodFrames` rises continuously.
3. Verify `gHVisionCrcErrors` stays at zero and `gHVisionAgeMs < 50`.
4. Move the ball from negative to positive scale positions. Confirm
   `gHBallPositionTenthMm` is monotonic and within 20 units (2 mm).
5. Confirm a positive rod command moves the ball toward increasing camera
   coordinates. If not, change `H_BALL_CONTROL_SIGN`.
6. Only then enable the physical actuator and tune H3 on a stationary car.

## Main Telemetry

- Mission: `gHTask`, `gHState`, `gHFault`, `gHElapsedMs`, `gHResultMs`
- Chassis: `gHDistanceMm`, `gHSpeedCommandTicks`, `gHAccelerationMmps2`
- Ball: `gHBallTargetTenthMm`, `gHBallPositionTenthMm`,
  `gHBallVelocityTenthMmps`, `gHBallErrorTenthMm`, `gHRodAngleMdeg`
- Health: `gHVisionHealthy`, `gHVisionAgeMs`, `gSchedulerOverruns`

The main line array currently recognizes A when at least six sensors are
black. Two outboard marker sensors remain the preferred final hardware; add
them only after their actual GPIO pins and physical spacing are fixed.
