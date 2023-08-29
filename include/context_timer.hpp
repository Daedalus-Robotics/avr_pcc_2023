#include <rcl/rcl.h>

#ifndef AVR_PCC_2023_CONTEXT_TIMER_HPP
#define AVR_PCC_2023_CONTEXT_TIMER_HPP

struct TimerWithContext
{
    rcl_timer_t timer;
    void *context;
};

#endif //AVR_PCC_2023_CONTEXT_TIMER_HPP
