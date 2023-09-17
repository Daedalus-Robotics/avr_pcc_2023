#include <rcl/rcl.h>

#ifndef AVR_PCC_2023_CONTEXT_TIMER_HPP
#define AVR_PCC_2023_CONTEXT_TIMER_HPP

#define CONTEXT_TIMER_CALLBACK(cls, func) [](rcl_timer_t *timer, int64_t n) \
{                                                                           \
    auto context_timer = (TimerWithContext *) timer;                        \
    auto context = (cls *) context_timer->context;                          \
    context->func(timer, n);                                                \
}

struct TimerWithContext
{
    rcl_timer_t timer;
    void *context;
};

#endif //AVR_PCC_2023_CONTEXT_TIMER_HPP
