extern "C"
{
#include <micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.h>
}

#define DIAGNOSTIC_CONTEXT_TASK_CALLBACK(cls, func) [](diagnostic_value_t values[                         \
                                                         MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_VALUES_PER_TASK \
                                                      ],                                                  \
                                                      uint8_t *number_of_values)                          \
{                                                                                                         \
    auto diagnostic_context_task = (DiagnosticTaskWithContext *) ((diagnostic_task_t *) number_of_values);\
    auto context = (cls *) diagnostic_context_task->context;                                              \
    return context->func(values, number_of_values);                                                       \
}

#ifndef AVR_PCC_2023_DIAGNOSTIC_CONTEXT_TASK_HPP
#define AVR_PCC_2023_DIAGNOSTIC_CONTEXT_TASK_HPP

struct DiagnosticTaskWithContext
{
    diagnostic_task_t diagnosticTask;
    void *context;
};

#endif //AVR_PCC_2023_DIAGNOSTIC_CONTEXT_TASK_HPP
