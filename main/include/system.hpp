#include <driver/gpio.h>
#include <driver/rmt.h>
#include <rcl/rcl.h>
#include <rcl_interfaces/msg/log.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "neopixel_strip.hpp"

#ifndef AVR_PCC_2023_SYSTEM_HPP
#define AVR_PCC_2023_SYSTEM_HPP

#define LED_PIN GPIO_NUM_13
#define SYSTEM_EXECUTOR_HANDLES 2

#define LOG(logLevel, msg) log(logLevel, msg, __FILE__, __PRETTY_FUNCTION__, __LINE__)

enum [[maybe_unused]] LogLevel
{
    LOGLEVEL_DEBUG = rcl_interfaces__msg__Log__DEBUG,
    LOGLEVEL_INFO = rcl_interfaces__msg__Log__INFO,
    LOGLEVEL_WARN = rcl_interfaces__msg__Log__WARN,
    LOGLEVEL_ERROR = rcl_interfaces__msg__Log__ERROR,
    LOGLEVEL_FATAL = rcl_interfaces__msg__Log__FATAL
};
#define HANDLE_ROS_ERROR(rc, do_reset) handleError(rc, true, do_reset, __FILE__, __PRETTY_FUNCTION__, __LINE__)
#define HANDLE_ESP_ERROR(rc, do_reset) handleError(rc, false, do_reset, __FILE__, __PRETTY_FUNCTION__, __LINE__)
#define CONTEXT_SERVICE_CALLBACK(cls, func) [](const void *req, void *res, void *void_context) \
{                                                                                              \
    auto context = (cls *) void_context;                                                       \
    context->func(req, res);                                                                   \
}

/**
 * Set the NeopixelStrip object to use for the status light
 */
void setStatusStrip(NeopixelStrip *strip);

/**
 * Send a log message on a ros topic
 * @param level The log level
 * @param msg The log message to be sent
 * @param file The name of the file where the log was called
 * @param function The function where the log was called
 * @param line The line number that the log was called
 * @return Whether it was successful
 */
bool log(LogLevel level,
         const char msg[], const char file[] = "", const char function[] = "", uint32_t line = 0);

/**
 * Checks the error code passed in and logs if it is an issue
 * @param rc The error code
 * @param is_ros Whether the error is coming from a ros function or an esp-idf function
 * @param do_reset Whether to reset the microcontroller if there is an error
 * @param file The name of the file (for logging)
 * @param function The name of the function (for logging)
 * @param line The line number (for logging)
 * @return Whether it was successful
 */
bool handleError(int32_t rc,
                 bool is_ros, bool do_reset = false,
                 const char file[] = "", const char function[] = "", uint32_t line = 0);

/**
 * Set up logging and reset service
 * @param support A micro ros support structure
 * @param executor An initialized executor to bind handles to
 */
void setupSystem(rclc_support_t *support, rclc_executor_t *executor);

/**
 * Clean up logging and reset service
 */
void cleanupSystem();

/**
 * Initializes the logging and starts the ping timer
 * @param setup_func Function to call when connecting to agent
 * @param cleanup_func Function to call when disconnecting from agent
 */
void initSystem(void (*setup_func)(), void (*cleanup_func)());

#endif //AVR_PCC_2023_SYSTEM_HPP
