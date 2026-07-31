#include "competition_registry.h"

#include "task2.h"

#include <stddef.h>

const CompetitionTaskOps *competition_registry_find(uint8_t taskNumber)
{
    switch (taskNumber) {
        case 2U:
            return task2_get_ops();
        default:
            return NULL;
    }
}

bool competition_registry_is_available(uint8_t taskNumber)
{
    return competition_registry_find(taskNumber) != NULL;
}
