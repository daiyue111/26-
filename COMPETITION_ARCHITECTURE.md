# 2026 H Competition Architecture

This file defines the final competition structure. Bench tests must use the
same production modules and interfaces. Temporary stand-alone control logic
must not be merged into the competition target.

## System Ownership

- Lite-K230D: camera detection, live video, recording output, ball coordinate.
- MSPM0G3507: the only real-time control authority and competition timer.
- Line MCU sensor: black-line position input.
- Rod hardware adapter: angle sensor input and stepper pulse/direction output.
- LCD: selected task before start, elapsed/result time after start.

The K230 is a perception coprocessor. It must not directly drive the rod
actuator. If a second real-time MCU is later required by the timer or pin
budget, it must remain behind the rod actuator interface so mission behavior
does not change.

## Competition Tasks

| Task | Route mode | Ball mode | Completion event |
| --- | --- | --- | --- |
| H2 | One lap, stop at A | Disabled | Chassis stopped at A |
| H3 | Wheels locked | +50 mm, then -50 mm | Ball settled at -50 mm |
| H4 | A to B | Hold 0 mm | B passed |
| H5 | One lap through A | Hold 0 mm | A passed |
| H6 | One lap through A | Hold initial position | A passed |

H6 captures the valid ball position at the START event. It does not require a
separate target-entry user interface.

## On-Site Operation

Task selection and mission start are separate physical inputs. A dedicated
rotary switch or DIP input selects H2 to H6 while the car is stopped. The
existing START key has exactly one meaning: start the selected task
immediately after debounce.

1. Power the car and start K230 video recording.
2. Set the dedicated task selector to H2, H3, H4, H5, or H6.
3. Confirm the selected task and readiness on the LCD.
4. Place the car and ball in the required initial positions.
5. Press START once. The selected task and timer start together.
6. Inputs are ignored while running. Reset the controller before another test.

Until the dedicated selector hardware is assigned, the production firmware
must default to H2 and preserve the verified single-press H2 behavior. There
is no software warmup lock, click counting, or short/long-press overloading.

## Production Runtime

```text
main / 1 ms scheduler
  competition_runtime             lifecycle and selected-task dispatch
    competition_selector          build target or dedicated selector input
    competition_registry          task-number to task implementation mapping
    competition_ui                START debounce, LCD, result and fault LEDs
    competition_safety            the single all-actuator stop path
    selected task
      task2                       frozen one-lap precise-stop implementation
      task3                       stationary +50 mm to -50 mm ball motion
      task4                       A to B while holding ball at center
      task5                       one lap while holding ball at center
      task6                       one lap while holding initial ball position

shared services
  H2 frozen chassis control
  loaded chassis route control for H4-H6
  ball estimator, ball position loop and rod angle loop

hardware drivers
  motor, encoder, track, K230 UART, angle ADC, stepper timer, LCD
```

The production `main` calls only `competition_runtime`. `car_app`,
`competition_app`, `h_mission`, the 24H square mission, and calibration test
programs are legacy/diagnostic sources and are not members of the production
Keil target. They must never be called from a competition task.

## File Ownership

| Module | Sole responsibility |
| --- | --- |
| `competition_runtime` | READY/RUNNING/COMPLETE/FAULT lifecycle |
| `competition_registry` | Resolve a task number to one task object |
| `competition_selector` | Read the compile-time or physical task selection |
| `competition_ui` | START event, task/result display, status indication |
| `competition_safety` | Stop wheels, ball actuator and switched outputs |
| `taskN` | Question-specific state machine and completion condition |
| `taskN_config` | Parameters owned only by that question |

The runtime does not contain line following, marker detection, ball control,
or actuator algorithms. The UI cannot start motors. A task cannot select or
call another task. Only the safety module may override every actuator.

## Control Composition

| Task | Chassis controller | Ball controller | Completion |
| --- | --- | --- | --- |
| H2 | Frozen H2 controller | Disabled | Stop at A |
| H3 | Wheels locked | +50 mm then -50 mm trajectory | Ball settled |
| H4 | Loaded A-to-B profile | Hold 0 mm | Pass B |
| H5 | Loaded one-lap profile | Hold 0 mm | Return to A |
| H6 | Loaded one-lap profile | Hold captured START position | Return to A |

H2 remains its own frozen control implementation. H4-H6 may share a loaded
chassis algorithm, but each task supplies an immutable task-local profile.
The rod angle inner loop is verified and frozen before H3 tuning; the ball
position loop is verified and frozen before H4 integration.

## Task Isolation Contract

Each task is a separate source module with its own state structure, tuning
parameters, completion condition, result time, and fault codes. A task must
not call another task or access another task's state.

All tasks expose the same small interface:

```text
init -> start -> update_1ms -> stop
                  |
                  +-> state / fault / result time
```

The dispatcher may call this interface but contains no line-following, ball,
motor, marker, or actuator control logic. Only the selected task receives
updates and owns actuator commands.

Shared code is limited to hardware drivers, safety shutdown, sensor parsing,
and stateless control math. Task-specific gains, speed profiles, thresholds,
timeouts, and route phases live in `task2_config` through `task6_config`, not
in one global H configuration. If two tasks require different behavior, they
use different profiles or task-local algorithms instead of changing another
task's tuned constants.

## Build Targets

The production source supports these build modes from the same task files:

- `H2_ONLY`, `H3_ONLY`, `H4_ONLY`, `H5_ONLY`, `H6_ONLY`
- `COMPETITION_ALL`

An ONLY target fixes one task at compile time and bypasses task selection,
but still runs that task's production module. `COMPETITION_ALL` adds only the
selector and dispatcher. There are no separate bench implementations to
merge later.

`competition_build.h` currently selects `H2_ONLY`. Until a task is registered,
selecting its ONLY build deliberately fails at compile time. This prevents an
incomplete task image from reaching the car.

## Frozen H2 Contract

The tag `h2-final-mcu-precise-stop` is the golden H2 fallback. Architecture
work must not change these H2 items without a separate measured tuning step:

- line sensor polarity and physical orientation;
- line error to differential PWM behavior;
- speed, slew, and correction limits;
- A marker qualification and debounce;
- `H_FINISH_PASS_MM` and active braking;
- motor and encoder pin mapping.

Every architecture change must build first, then pass one H2 selection/start
test, then pass repeated full H2 laps before it is committed.

## Integration Gates

All gates use the production competition target.

1. H2 regression: run `H2_ONLY` and repeat accurate A stops.
2. Vision input: wheels and actuator disabled; validate K230 coordinate age,
   confidence, sign, scale, and video recording.
3. Rod angle inner loop: no ball; validate horizontal zero, sign, limits, and
   emergency stop.
4. H3 position loop: wheels locked; tune +50 mm to -50 mm using the production
   H3 task.
5. H4: run only A to B while holding the center.
6. H5: extend the verified H4 combination to one lap.
7. H6: capture and hold several different initial ball positions for one lap.

No later gate may bypass or replace a module verified by an earlier gate.
After a task passes its ONLY target, the same task must pass unchanged in
`COMPETITION_ALL` before the next task is started.
