#include <Arduino.h>
#include <Adafruit_SleepyDog.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rcl_interfaces/msg/log.h>

#ifndef AVR_PCC_2023_UTILS_HPP
#define AVR_PCC_2023_UTILS_HPP

#define CLEANUP_ACTION(context, callback) { addCleanup((CleanupAction) {context, callback}); }

/**
 * Reset the microcontroller
 */
void reset();

class Node; // forward declaration

struct CleanupAction
{
    Node* context;

    rcl_ret_t (* callback)(Node*);
};

/**
 * Add a callback to be called on cleanup
 * They will be executed in reverse order
 * @param callback The callback to add (Should return a rcl_ret_t from a micro-ros function or 0)
 */
void addCleanup(CleanupAction cleanupAction);

enum LogLevel
{
    DEBUG = 10,
    INFO = 20,
    WARN = 30,
    ERROR = 40,
    FATAL = 50
};

/**
 * Send a log message on a ros topic
 * @param level The log level
 * @param msg The log message to be sent
 */
void log(LogLevel level, const char msg[]);

/**
 * Sets up the logger node and its publisher
 * @param support The support structure that is needed to init a node
 */
void initLogger(rclc_support_t* support);

/**
 * Checks the error code passed in and logs if it is an issue
 * @param error The error code
 * @param do_reset Whether to reset the microcontroller if there is an error
 */
void handleError(rcl_ret_t error, bool do_reset = false);


#endif //AVR_PCC_2023_UTILS_HPP
