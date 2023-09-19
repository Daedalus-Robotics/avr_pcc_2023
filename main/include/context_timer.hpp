#include <rcl/rcl.h>

#define CONTEXT_TIMER_CALLBACK(cls, func) [](rcl_timer_t *timer, int64_t n) \
{                                                                           \
    auto context_timer = (TimerWithContext *) timer;                        \
    auto context = (cls *) context_timer->context;                          \
    context->func(timer, n);                                                \
}

#ifndef AVR_PCC_2023_CONTEXT_TIMER_HPP
#define AVR_PCC_2023_CONTEXT_TIMER_HPP

struct TimerWithContext
{
    rcl_timer_t timer;
    void *context;
};

#endif //AVR_PCC_2023_CONTEXT_TIMER_HPP
