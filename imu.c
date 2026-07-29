#include "imu.h"

#include <stddef.h>

#define IMU_SPI SPI1
#define IMU_SPI_SCLK_IOMUX IOMUX_PINCM39
#define IMU_SPI_SCLK_FUNC IOMUX_PINCM39_PF_SPI1_SCLK
#define IMU_SPI_POCI_IOMUX IOMUX_PINCM38
#define IMU_SPI_POCI_FUNC IOMUX_PINCM38_PF_SPI1_POCI
#define IMU_SPI_PICO_IOMUX IOMUX_PINCM32
#define IMU_SPI_PICO_FUNC IOMUX_PINCM32_PF_SPI1_PICO
#define IMU_SPI_CS_PORT GPIOA
#define IMU_SPI_CS_PIN DL_GPIO_PIN_8
#define IMU_SPI_CS_IOMUX IOMUX_PINCM19

#define SPI_TIMEOUT_CYCLES 40000U
#define IMU_INIT_ATTEMPTS 3U
#define IMU_WHO_AM_I_RETRIES 5U
#define IMU_WHO_AM_I_RETRY_MS 20U
#define IMU_WHO_AM_I_REG 0x75U
#define IMU_WHO_AM_I_EXPECTED 0x47U
#define IMU_REG_BANK_SEL 0x76U
#define IMU_PWR_MGMT0 0x4EU
#define IMU_GYRO_CONFIG0 0x4FU
#define IMU_GYRO_DATA_Z1 0x29U
#define IMU_BIAS_SETTLE_MS 220U
#define IMU_BIAS_SAMPLES 32U
#define IMU_BIAS_SAMPLE_MS 4U

static int32_t gGyroZBias;
static bool gGyroReady;
static uint8_t gImuError;
static int32_t gHeadingMdeg;
static int32_t gLastGyroDelta;

void imu_bus_prepare(void)
{
    DL_GPIO_initDigitalOutput(IMU_SPI_CS_IOMUX);
    DL_GPIO_setPins(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN);
    DL_GPIO_enableOutput(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN);
}

static void imu_delay_ms(uint32_t ms)
{
    while (ms > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        ms--;
    }
}

static void imu_spi_init(void)
{
    DL_SPI_Config config = {
        .mode = DL_SPI_MODE_CONTROLLER,
        .frameFormat = DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
        .parity = DL_SPI_PARITY_NONE,
        .dataSize = DL_SPI_DATA_SIZE_8,
        .bitOrder = DL_SPI_BIT_ORDER_MSB_FIRST,
        .chipSelectPin = DL_SPI_CHIP_SELECT_0,
    };
    DL_SPI_ClockConfig clockConfig = {
        .clockSel = DL_SPI_CLOCK_BUSCLK,
        .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1,
    };

    imu_bus_prepare();
    DL_GPIO_initPeripheralOutputFunction(IMU_SPI_SCLK_IOMUX,
        IMU_SPI_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(IMU_SPI_PICO_IOMUX,
        IMU_SPI_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(IMU_SPI_POCI_IOMUX,
        IMU_SPI_POCI_FUNC);

    DL_SPI_reset(IMU_SPI);
    DL_SPI_enablePower(IMU_SPI);
    delay_cycles(16U);

    DL_SPI_init(IMU_SPI, &config);
    DL_SPI_setClockConfig(IMU_SPI, &clockConfig);
    DL_SPI_setBitRateSerialClockDivider(IMU_SPI, 63U);
    DL_SPI_setFIFOThreshold(IMU_SPI, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME,
        DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);
    DL_SPI_enable(IMU_SPI);
}

static bool spi_transfer_byte(uint8_t tx, uint8_t *rx)
{
    while (!DL_SPI_isRXFIFOEmpty(IMU_SPI)) {
        (void) DL_SPI_receiveData8(IMU_SPI);
    }

    for (uint32_t t = 0U; t < SPI_TIMEOUT_CYCLES; t++) {
        if (!DL_SPI_isTXFIFOFull(IMU_SPI)) {
            DL_SPI_transmitData8(IMU_SPI, tx);
            break;
        }
        if (t == (SPI_TIMEOUT_CYCLES - 1U)) {
            return false;
        }
    }

    for (uint32_t t = 0U; t < SPI_TIMEOUT_CYCLES; t++) {
        if (!DL_SPI_isRXFIFOEmpty(IMU_SPI)) {
            *rx = DL_SPI_receiveData8(IMU_SPI);
            return true;
        }
    }

    return false;
}

static bool imu_spi_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t dummy = 0U;

    DL_GPIO_clearPins(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN);
    delay_cycles(8U);

    if (!spi_transfer_byte((uint8_t)(reg | 0x80U), &dummy) ||
        !spi_transfer_byte(0x00U, value)) {
        DL_GPIO_setPins(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN);
        return false;
    }

    while (DL_SPI_isBusy(IMU_SPI)) {
    }
    delay_cycles(8U);
    DL_GPIO_setPins(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN);
    return true;
}

static bool imu_spi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t dummy = 0U;

    DL_GPIO_clearPins(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN);
    delay_cycles(8U);

    if (!spi_transfer_byte((uint8_t)(reg & 0x7FU), &dummy) ||
        !spi_transfer_byte(value, &dummy)) {
        DL_GPIO_setPins(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN);
        return false;
    }

    while (DL_SPI_isBusy(IMU_SPI)) {
    }
    delay_cycles(8U);
    DL_GPIO_setPins(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN);
    return true;
}

static bool imu_spi_read_i16(uint8_t regHigh, int16_t *value)
{
    uint8_t high = 0U;
    uint8_t low = 0U;

    if (!imu_spi_read_reg(regHigh, &high) ||
        !imu_spi_read_reg((uint8_t)(regHigh + 1U), &low)) {
        return false;
    }

    *value = (int16_t)(((uint16_t)high << 8) | low);
    return true;
}

static bool imu_start_gyro(void)
{
    uint8_t who = 0U;
    bool readCompleted = false;

    imu_delay_ms(120U);
    for (uint8_t attempt = 0U; attempt < IMU_WHO_AM_I_RETRIES;
         attempt++) {
        if (imu_spi_read_reg(IMU_WHO_AM_I_REG, &who)) {
            readCompleted = true;
            if (who == IMU_WHO_AM_I_EXPECTED) {
                break;
            }
        }
        imu_delay_ms(IMU_WHO_AM_I_RETRY_MS);
    }
    if (who != IMU_WHO_AM_I_EXPECTED) {
        gImuError = readCompleted ? 2U : 1U;
        return false;
    }
    if (!imu_spi_write_reg(IMU_REG_BANK_SEL, 0x00U)) {
        gImuError = 3U;
        return false;
    }
    if (!imu_spi_write_reg(IMU_GYRO_CONFIG0, 0x06U)) {
        gImuError = 4U;
        return false;
    }
    if (!imu_spi_write_reg(IMU_PWR_MGMT0, 0x0FU)) {
        gImuError = 5U;
        return false;
    }
    imu_delay_ms(200U);
    return true;
}

bool imu_init_gyro_z(void)
{
    int32_t sum = 0;
    uint8_t count = 0U;
    bool started = false;

    gGyroReady = false;
    gImuError = 0U;
    imu_heading_reset();
    for (uint8_t attempt = 0U; attempt < IMU_INIT_ATTEMPTS; attempt++) {
        imu_spi_init();
        if (imu_start_gyro()) {
            started = true;
            break;
        }
        imu_delay_ms(100U);
    }
    if (!started) {
        return false;
    }

    imu_delay_ms(IMU_BIAS_SETTLE_MS);
    for (uint8_t i = 0U; i < IMU_BIAS_SAMPLES; i++) {
        int16_t gz = 0;

        if (imu_spi_read_i16(IMU_GYRO_DATA_Z1, &gz)) {
            sum += gz;
            count++;
        }
        imu_delay_ms(IMU_BIAS_SAMPLE_MS);
    }

    if (count == 0U) {
        gImuError = 6U;
        return false;
    }

    gGyroZBias = sum / (int32_t)count;
    gGyroReady = true;
    return true;
}

bool imu_read_gyro_z_delta(int32_t *delta)
{
    int16_t gz = 0;

    if (!gGyroReady || (delta == NULL) ||
        !imu_spi_read_i16(IMU_GYRO_DATA_Z1, &gz)) {
        return false;
    }

    *delta = (int32_t)gz - gGyroZBias;
    return true;
}

uint8_t imu_get_error(void)
{
    return gImuError;
}

void imu_heading_reset(void)
{
    gHeadingMdeg = 0;
    gLastGyroDelta = 0;
}

bool imu_heading_update(uint32_t sampleMs)
{
    int32_t delta;

    if (!imu_read_gyro_z_delta(&delta)) {
        return false;
    }

    gLastGyroDelta = delta;
    if ((delta > 8) || (delta < -8)) {
        /* GYRO_CONFIG0=0x06 selects 2000 dps, 16.4 LSB/(deg/s). */
        gHeadingMdeg += (delta * (int32_t)sampleMs * 10) / 164;
    }
    return true;
}

int32_t imu_heading_get_mdeg(void)
{
    return gHeadingMdeg;
}

int32_t imu_get_last_gyro_delta(void)
{
    return gLastGyroDelta;
}
