# X42S Closed-loop Stepper Integration

The tested X42S Emm UART driver is included as an optional competition
module. It is disabled by default, so the current car pin allocation and
runtime behavior remain unchanged.

## Enable

1. In `app_config.h`, set `APP_ENABLE_ZDT_X42S` to `1U`.
2. In SysConfig, add a UART instance named exactly `UART_ZDT`.
3. Configure it for 115200 baud, 8 data bits, no parity, and 1 stop bit.
4. Regenerate SysConfig code and rebuild the project.
5. Call `zdt_stepper_init()` after `SYSCFG_DL_init()` and before using the
   remaining APIs.

The build intentionally fails when the feature is enabled without a
`UART_ZDT` instance.

## Wiring

- MCU UART TX -> X42S `R/A/H`
- MCU UART RX <- X42S `T/B/L`
- MCU GND -> X42S signal GND
- X42S power input -> suitable external motor supply (12 V was verified)
- X42S motor connector -> stepper motor phase connector

The MCU and X42S power supply must share ground. Do not power the motor from
an MCU GPIO or MCU 3.3 V rail.

`PA10` and `PA11` cannot be used in the current car build: they are assigned
to left-rear TB6612 signals `CIN1` and `CIN2`. To reuse the tested PA10/PA11
pair, first release that motor channel in both wiring and SysConfig. A future
competition task may instead assign any free UART-capable TX/RX pair.

## API

Include `zdt_stepper.h`. The usual address is
`ZDT_STEPPER_DEFAULT_ADDRESS` (`1`).

```c
zdt_stepper_init();
zdt_stepper_enable(ZDT_STEPPER_DEFAULT_ADDRESS, true);
zdt_stepper_set_speed(ZDT_STEPPER_DEFAULT_ADDRESS,
    ZDT_STEPPER_DIRECTION_CW, 240U, 10U);
zdt_stepper_stop(ZDT_STEPPER_DEFAULT_ADDRESS);
zdt_stepper_enable(ZDT_STEPPER_DEFAULT_ADDRESS, false);
```

Available feedback reads are input pulse count, actual RPM, current position,
position error, status flags, and the combined `ZdtStepperStatus` snapshot.

- `speedRpm`: signed RPM
- `positionCdeg`: signed centidegrees (`36000` = one revolution)
- `positionErrorCdeg`: signed centidegrees
- `inputPulses`: signed pulse count

Every command waits for a UART response and returns `true` only for a valid
reply. This driver is blocking: call it from foreground/task code, never from
an interrupt service routine or the 1 ms control interrupt. When the feature
is disabled, all command/read functions return `false` and touch no hardware.
