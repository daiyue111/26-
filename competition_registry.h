#ifndef COMPETITION_REGISTRY_H
#define COMPETITION_REGISTRY_H

#include "competition_task.h"

const CompetitionTaskOps *competition_registry_find(uint8_t taskNumber);
bool competition_registry_is_available(uint8_t taskNumber);

#endif
