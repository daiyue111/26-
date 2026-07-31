#include "competition_selector.h"

#include "competition_build.h"

void competition_selector_init(void)
{
}

uint8_t competition_selector_read(void)
{
#if COMPETITION_BUILD_TARGET == COMPETITION_BUILD_ALL
    /* Replace only this adapter when the selector hardware is assigned. */
    return COMPETITION_DEFAULT_TASK;
#else
    return COMPETITION_BUILD_TARGET;
#endif
}
