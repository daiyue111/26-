#include "zdt_stepper.h"

#include "app_config.h"

#if APP_ENABLE_ZDT_X42S

#ifndef UART_ZDT_INST
#error "APP_ENABLE_ZDT_X42S requires a SysConfig UART instance named UART_ZDT"
#endif

#define ZDT_FRAME_END 0x6BU
#define ZDT_RESPONSE_MAX 8U
#define ZDT_RESPONSE_TIMEOUT_LOOPS 20000U
#define ZDT_ACK_LENGTH 4U

static void drain_rx(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_ZDT_INST)) {
        (void) DL_UART_Main_receiveData(UART_ZDT_INST);
    }
}

static bool receive_response(uint8_t address, uint8_t function,
    uint8_t *response, uint8_t expectedLength)
{
    uint8_t count = 0U;

    if ((response == 0) || (expectedLength < ZDT_ACK_LENGTH) ||
        (expectedLength > ZDT_RESPONSE_MAX)) {
        return false;
    }

    for (uint32_t timeout = 0U;
         timeout < ZDT_RESPONSE_TIMEOUT_LOOPS; timeout++) {
        if (!DL_UART_Main_isRXFIFOEmpty(UART_ZDT_INST)) {
            uint8_t value =
                (uint8_t) DL_UART_Main_receiveData(UART_ZDT_INST);

            response[count++] = value;
            if (count == expectedLength) {
                return (response[0] == address) &&
                    (response[1] == function) &&
                    (response[expectedLength - 1U] == ZDT_FRAME_END);
            }
        } else {
            delay_cycles(32U);
        }
    }
    return false;
}

static bool transact(const uint8_t *command, uint8_t commandLength,
    uint8_t *response, uint8_t responseLength)
{
    uint8_t address = command[0];
    uint8_t function = command[1];

    drain_rx();
    for (uint8_t i = 0U; i < commandLength; i++) {
        DL_UART_Main_transmitDataBlocking(UART_ZDT_INST, command[i]);
    }
    while (DL_UART_Main_isBusy(UART_ZDT_INST)) {
    }

    return receive_response(address, function, response, responseLength);
}

static bool send_command(const uint8_t *command, uint8_t length)
{
    uint8_t response[ZDT_ACK_LENGTH];

    return transact(command, length, response, ZDT_ACK_LENGTH) &&
        (response[2] == 0x02U);
}

static bool read_parameter(uint8_t address, uint8_t function,
    uint8_t *response, uint8_t responseLength)
{
    const uint8_t command[3] = {address, function, ZDT_FRAME_END};

    return transact(command, (uint8_t) sizeof(command), response,
        responseLength);
}

static uint32_t read_u32_be(const uint8_t *data)
{
    return ((uint32_t) data[0] << 24) |
        ((uint32_t) data[1] << 16) |
        ((uint32_t) data[2] << 8) |
        (uint32_t) data[3];
}

static int64_t signed_magnitude(uint8_t direction, uint32_t magnitude)
{
    int64_t value = (int64_t) magnitude;

    return (direction == 0U) ? value : -value;
}

void zdt_stepper_init(void)
{
    drain_rx();
}

bool zdt_stepper_enable(uint8_t address, bool enable)
{
    const uint8_t command[6] = {
        address, 0xF3U, 0xABU, enable ? 1U : 0U, 0x00U, ZDT_FRAME_END
    };

    return send_command(command, (uint8_t) sizeof(command));
}

bool zdt_stepper_set_speed(uint8_t address, uint8_t direction,
    uint16_t rpm, uint8_t acceleration)
{
    uint8_t command[8];

    if (rpm > 5000U) {
        rpm = 5000U;
    }

    command[0] = address;
    command[1] = 0xF6U;
    command[2] = (direction == 0U) ? 0U : 1U;
    command[3] = (uint8_t) (rpm >> 8);
    command[4] = (uint8_t) rpm;
    command[5] = acceleration;
    command[6] = 0x00U;
    command[7] = ZDT_FRAME_END;

    return send_command(command, (uint8_t) sizeof(command));
}

bool zdt_stepper_stop(uint8_t address)
{
    const uint8_t command[5] = {
        address, 0xFEU, 0x98U, 0x00U, ZDT_FRAME_END
    };

    return send_command(command, (uint8_t) sizeof(command));
}

bool zdt_stepper_read_input_pulses(uint8_t address, int64_t *pulses)
{
    uint8_t response[8];

    if ((pulses == 0) ||
        !read_parameter(address, 0x32U, response, sizeof(response))) {
        return false;
    }

    *pulses = signed_magnitude(response[2], read_u32_be(&response[3]));
    return true;
}

bool zdt_stepper_read_speed_rpm(uint8_t address, int32_t *rpm)
{
    uint8_t response[6];
    uint16_t magnitude;

    if ((rpm == 0) ||
        !read_parameter(address, 0x35U, response, sizeof(response))) {
        return false;
    }

    magnitude = (uint16_t) (((uint16_t) response[3] << 8) | response[4]);
    *rpm = (response[2] == 0U) ? (int32_t) magnitude : -(int32_t) magnitude;
    return true;
}

bool zdt_stepper_read_position_cdeg(uint8_t address, int64_t *position)
{
    uint8_t response[8];
    int64_t raw;

    if ((position == 0) ||
        !read_parameter(address, 0x36U, response, sizeof(response))) {
        return false;
    }

    raw = signed_magnitude(response[2], read_u32_be(&response[3]));
    *position = (raw * 36000LL) / 65536LL;
    return true;
}

bool zdt_stepper_read_position_error_cdeg(uint8_t address, int64_t *error)
{
    uint8_t response[8];

    if ((error == 0) ||
        !read_parameter(address, 0x37U, response, sizeof(response))) {
        return false;
    }

    *error = signed_magnitude(response[2], read_u32_be(&response[3]));
    return true;
}

bool zdt_stepper_read_status_flags(uint8_t address, uint8_t *flags)
{
    uint8_t response[4];

    if ((flags == 0) ||
        !read_parameter(address, 0x3AU, response, sizeof(response))) {
        return false;
    }

    *flags = response[2];
    return true;
}

bool zdt_stepper_read_status(uint8_t address, ZdtStepperStatus *status)
{
    ZdtStepperStatus snapshot;

    if (status == 0) {
        return false;
    }
    if (!zdt_stepper_read_input_pulses(address, &snapshot.inputPulses) ||
        !zdt_stepper_read_speed_rpm(address, &snapshot.speedRpm) ||
        !zdt_stepper_read_position_cdeg(address, &snapshot.positionCdeg) ||
        !zdt_stepper_read_position_error_cdeg(address,
            &snapshot.positionErrorCdeg) ||
        !zdt_stepper_read_status_flags(address, &snapshot.flags)) {
        return false;
    }

    snapshot.enabled =
        (snapshot.flags & ZDT_STATUS_ENABLED) != 0U;
    snapshot.positionReached =
        (snapshot.flags & ZDT_STATUS_POSITION_REACHED) != 0U;
    *status = snapshot;
    return true;
}

#else

void zdt_stepper_init(void)
{
}

bool zdt_stepper_enable(uint8_t address, bool enable)
{
    (void) address;
    (void) enable;
    return false;
}

bool zdt_stepper_set_speed(uint8_t address, uint8_t direction,
    uint16_t rpm, uint8_t acceleration)
{
    (void) address;
    (void) direction;
    (void) rpm;
    (void) acceleration;
    return false;
}

bool zdt_stepper_stop(uint8_t address)
{
    (void) address;
    return false;
}

bool zdt_stepper_read_input_pulses(uint8_t address, int64_t *pulses)
{
    (void) address;
    (void) pulses;
    return false;
}

bool zdt_stepper_read_speed_rpm(uint8_t address, int32_t *rpm)
{
    (void) address;
    (void) rpm;
    return false;
}

bool zdt_stepper_read_position_cdeg(uint8_t address, int64_t *position)
{
    (void) address;
    (void) position;
    return false;
}

bool zdt_stepper_read_position_error_cdeg(uint8_t address, int64_t *error)
{
    (void) address;
    (void) error;
    return false;
}

bool zdt_stepper_read_status_flags(uint8_t address, uint8_t *flags)
{
    (void) address;
    (void) flags;
    return false;
}

bool zdt_stepper_read_status(uint8_t address, ZdtStepperStatus *status)
{
    (void) address;
    (void) status;
    return false;
}

#endif
