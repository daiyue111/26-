# Three-Loop Chassis Tuning Guide

Do not tune all loops at the same time. Use a stand for the first two stages
and keep the car stationary whenever PA09 starts IMU calibration.

## 1. Encoder Direction And Scale

1. Use `APP_MODE_ENCODER_DIAGNOSTIC` for the direction check.
2. Watch `gEncoderLeftCount` and `gEncoderRightCount` while rotating each
   wheel forward by hand.
3. Both counts must increase. Change the corresponding encoder sign in
   `app_config.h` if needed.
4. Rotate each wheel exactly one revolution and record the counts. Because
   A phase is counted on both edges, this is the effective counts per wheel
   revolution used by the controller.

Record these chassis values:

- Nominal counts/revolution: 280 (MG310 P20 provisional value)
- Wheel diameter: 48 mm
- Wheel track: TBD mm
- Sensor-to-chassis-center distance: TBD mm

The official 2025 E track outer edge is 1000 x 1000 mm, line width is
18 +/- 2 mm, and the required travel direction is counterclockwise.

For the final scale, use `APP_MODE_CHASSIS_CALIBRATION`, not a hand-turned
single wheel. Stage 1 measures both driven sides across a nominal 982 mm
line-center spacing and reports `gCalibrationCountsPerMeter`. Repeat it at
least three times from the same sensor position. Reject a run if the two side
counts differ abnormally or the car visibly loses the line.

## 2. Wheel Speed PI

1. Select `APP_MODE_SQUARE_3LOOP` and keep the chassis on a stand.
2. Start with identical low wheel-speed targets.
3. Tune `SPEED_PID_KP` until measured speed follows without sustained
   oscillation.
4. Increase `SPEED_PID_KI` only enough to remove steady-state mismatch.
5. Verify forward, reverse, and zero-speed braking for both wheels.

## 3. Corner Center Offset

The line sensor detects the corner before the chassis center reaches it.
`CHASSIS_CORNER_CENTER_OFFSET_MM` is the encoder distance from detection to
the point where the chassis center is directly above the corner.

1. Use a very low approach speed.
2. Mark the chassis center and field corner.
3. Calibration stage 3 records this point with PA09 and reports
   `gCalibrationCornerOffsetMm`.
4. Tune the distance PID only after the geometric offset is correct.

## 4. Straight Line PID

1. Set a low `SQUARE_CRUISE_SPEED_TICKS`.
2. Increase `SQUARE_LINE_PID_KP` until the car corrects line error promptly.
3. Increase `SQUARE_LINE_PID_KD` to suppress side-to-side oscillation.
4. Keep integral at zero unless a repeatable static lateral bias remains.

## 5. IMU Turn PID

1. Verify `gHeadingMdeg` changes in the expected direction during a manual
   right turn.
2. Change `SQUARE_TURN_DIRECTION` if the commanded turn direction is wrong.
3. Tune heading KP at low maximum turn speed.
4. Add KD to reduce overshoot near 90 degrees.
5. Confirm repeated turns return within the configured heading tolerance.
6. After the 90-degree stop, state `SQUARE_STATE_REACQUIRE_LINE` must find
   one of the two center sensors. Adjust the reacquire scan angle only after
   the basic 90-degree turn direction and heading are correct.

Before tuning 90-degree turns, calibration stage 2 performs a slow 360-degree
turn and combines actual IMU angle with both encoder distances. Its
`gCalibrationEffectiveTrackMm` is an effective skid-steer value that includes
floor and tire slip; it is intentionally not just the ruler-measured wheel
spacing.

## 6. Full Square Mission

Run one corner first, then two, then four. Check these variables after every
run:

- `gMissionState`
- `gMissionCornerCount`
- `gMissionDistanceError`
- `gMissionHeadingError`
- left/right encoder speed and PWM command

Final gains are chassis-specific. Keep a tested parameter set for each
combination of motor voltage, wheel size, payload, and floor material.
