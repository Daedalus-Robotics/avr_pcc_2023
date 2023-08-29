#include <Arduino.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_NeoPixel.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl_interfaces/msg/log.h>
#include <std_srvs/srv/empty.h> //ToDo: Replace with trigger type

#ifndef AVR_PCC_2023_UTILS_HPP
#define AVR_PCC_2023_UTILS_HPP

#define SYSTEM_EXECUTOR_HANDLES 3

#define CLEANUP_ACTION(context, callback) addCleanup((CleanupAction) {context, callback})
#define LOG(logLevel, msg) log(logLevel, msg, __FILE__, __FUNCTION__, __LINE__)
#define HANDLE_ERROR(rc, do_reset) handleError(rc, __FILE__, __FUNCTION__, __LINE__, do_reset)
#define SERVICE_CALLBACK(callback) reinterpret_cast<rclc_service_callback_t>(callback)
#define SERVICE_CALLBACK_CONTEXT(callback) reinterpret_cast<rclc_service_callback_with_context_t>(callback)

void setOnboardNeopixel(uint8_t r, uint8_t g, uint8_t b);

void beginOnboardNeopixel();

class Node; // forward declaration

/**
 * An action to be executed at cleanup
 */
struct CleanupAction
{
    /**
     * The context that is passed into the action
     */
    Node *context;

    /**
     * The callback to call
     * The context will be passed in
     * @return A return code or a 0
     */
    rcl_ret_t (*callback)(Node *);
};

/**
 * Add a cleanup action to be called on cleanup
 * They will be executed in reverse order
 * @param cleanup_action The action to add (The callback should return a rcl_ret_t from a micro-ros function or 0)
 */
void addCleanup(CleanupAction cleanup_action);

enum [[maybe_unused]] LogLevel
{
    DEBUG = rcl_interfaces__msg__Log__DEBUG,
    INFO = rcl_interfaces__msg__Log__INFO,
    WARN = rcl_interfaces__msg__Log__WARN,
    ERROR = rcl_interfaces__msg__Log__ERROR,
    FATAL = rcl_interfaces__msg__Log__FATAL
};

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
 * Checks the error code passed in and resets
 * Only use this if the logger has not been set up yet
 * @param rc The error code
 */
void handleEarlyError(rcl_ret_t rc);

/**
 * Checks the error code passed in and logs if it is an issue
 * @param rc The error code
 * @param do_reset Whether to reset the microcontroller if there is an error
 * @return Whether it was successful
 */
bool handleError(rcl_ret_t rc,
                 const char file[] = "", const char function[] = "", uint32_t line = 0,
                 bool do_reset = false);

/**
 * Sets up the logger node and its publisher
 * @param support The support structure that is needed to init a node
 */
void initSystem(rclc_support_t *support, rclc_executor_t *executor);


#endif //AVR_PCC_2023_UTILS_HPP
