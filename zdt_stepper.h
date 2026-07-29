#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

#define ZDT_STEPPER_DEFAULT_ADDRESS 1U
#define ZDT_STEPPER_DIRECTION_CW 0U
#define ZDT_STEPPER_DIRECTION_CCW 1U

#define ZDT_STATUS_ENABLED          (1U << 0)
#define ZDT_STATUS_POSITION_REACHED (1U << 1)

typedef struct {
    int64_t inputPulses;
    int32_t speedRpm;
    int64_t positionCdeg;
    int64_t positionErrorCdeg;
    uint8_t flags;
    bool enabled;
    bool positionReached;
} ZdtStepperStatus;

void zdt_stepper_init(void);
bool zdt_stepper_enable(uint8_t address, bool enable);
bool zdt_stepper_set_speed(uint8_t address, uint8_t direction,
    uint16_t rpm, uint8_t acceleration);
bool zdt_stepper_stop(uint8_t address);
bool zdt_stepper_read_input_pulses(uint8_t address, int64_t *pulses);
bool zdt_stepper_read_speed_rpm(uint8_t address, int32_t *rpm);
bool zdt_stepper_read_position_cdeg(uint8_t address, int64_t *position);
bool zdt_stepper_read_position_error_cdeg(uint8_t address, int64_t *error);
bool zdt_stepper_read_status_flags(uint8_t address, uint8_t *flags);
bool zdt_stepper_read_status(uint8_t address, ZdtStepperStatus *status);

#endif
