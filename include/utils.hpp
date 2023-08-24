#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rcl_interfaces/msg/log.h>

#ifndef AVR_PCC_2023_UTILS_HPP
#define AVR_PCC_2023_UTILS_HPP


void reset();

enum LogLevel
{
    DEBUG = 10,
    INFO = 20,
    WARN = 30,
    ERROR = 40,
    FATAL = 50
};

void log(LogLevel level, const char msg[]);

void initLogger(rclc_support_t* support);

void handleError(rcl_ret_t error, bool do_reset = false);


#endif //AVR_PCC_2023_UTILS_HPP
