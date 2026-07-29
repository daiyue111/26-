#include "calibration_store.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define CALIBRATION_FLASH_ADDRESS 0x0001FC00UL
#define CALIBRATION_MAGIC 0x43414C31UL
#define CALIBRATION_VERSION 1UL
#define CALIBRATION_WORD_COUNT \
    (sizeof(ChassisCalibrationRecord) / sizeof(uint32_t))

static uint32_t record_checksum(const ChassisCalibrationRecord *record)
{
    const uint32_t *words = (const uint32_t *)record;
    uint32_t value = 2166136261UL;

    for (uint32_t i = 0U; i < (CALIBRATION_WORD_COUNT - 1U); i++) {
        value ^= words[i];
        value *= 16777619UL;
    }
    return value;
}

bool calibration_store_load(ChassisCalibrationRecord *record)
{
    const ChassisCalibrationRecord *stored =
        (const ChassisCalibrationRecord *)CALIBRATION_FLASH_ADDRESS;
    uint32_t *destination = (uint32_t *)record;

    if (record == NULL) {
        return false;
    }
    for (uint32_t i = 0U; i < CALIBRATION_WORD_COUNT; i++) {
        destination[i] = ((const uint32_t *)stored)[i];
    }

    return (record->magic == CALIBRATION_MAGIC) &&
        (record->version == CALIBRATION_VERSION) &&
        (record->flags == CALIBRATION_FLAGS_ALL) &&
        (record->countsPerMeter != 0U) &&
        (record->checksum == record_checksum(record));
}

bool calibration_store_save(ChassisCalibrationRecord *record)
{
    DL_FLASHCTL_COMMAND_STATUS status;
    bool success;

    if (record == NULL) {
        return false;
    }

    record->magic = CALIBRATION_MAGIC;
    record->version = CALIBRATION_VERSION;
    record->checksum = record_checksum(record);

    __disable_irq();
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, CALIBRATION_FLASH_ADDRESS,
        DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
        CALIBRATION_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    success = (status != DL_FLASHCTL_COMMAND_STATUS_FAILED);

    if (success) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
#ifdef __MSPM0_HAS_ECC__
        status =
            DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
                FLASHCTL, CALIBRATION_FLASH_ADDRESS,
                (uint32_t *)record, CALIBRATION_WORD_COUNT,
                DL_FLASHCTL_REGION_SELECT_MAIN);
#else
        status = DL_FlashCTL_programMemoryFromRAM(FLASHCTL,
            CALIBRATION_FLASH_ADDRESS, (uint32_t *)record,
            CALIBRATION_WORD_COUNT, DL_FLASHCTL_REGION_SELECT_MAIN);
#endif
        success = (status != DL_FLASHCTL_COMMAND_STATUS_FAILED);
    }
    DL_FlashCTL_protectMainMemory(FLASHCTL);
    __enable_irq();

    if (!success) {
        return false;
    }

    ChassisCalibrationRecord verify;
    return calibration_store_load(&verify) &&
        (verify.sequence == record->sequence) &&
        (verify.checksum == record->checksum);
}
