#include "system.hpp"

#include <esp_log.h>
#include <rcl/error_handling.h>
#include <rcl_interfaces/msg/log.h>
#include <rmw_microros/rmw_microros.h>
#include <std_srvs/srv/trigger.h>

/**
 * If this is 1, errors will turn the neopixel red and blink the
 * led for each digit in the error code in reverse order
 * @example If the error is 123, it will blink 3 times, 2 times, then 1 time
 */
#define ENABLE_BLINK_ERROR 0

bool setupDone = false;
bool resetScheduled = false;

void (*setupFunc)();
void (*cleanupFunc)();
vprintf_like_t oldLogger;

NeopixelStrip *statusStrip;
rcl_timer_t pingTimer;
rcl_node_t systemNode;
rcl_publisher_t loggerPublisher;
rcl_service_t resetService;
std_srvs__srv__Trigger_Request resetServiceRequest;
std_srvs__srv__Trigger_Response resetServiceResponse;

void setStatusStrip(NeopixelStrip *strip)
{
    statusStrip = strip;
}

void reset()
{
    LOG(LOGLEVEL_INFO, "Resetting");
    statusStrip->fill(0, 255, 100);
    statusStrip->show();
    gpio_set_level(LED_PIN, 1);
    rclc_sleep_ms(50);
    gpio_set_level(LED_PIN, 0);
    rclc_sleep_ms(50);
    gpio_set_level(LED_PIN, 1);
    esp_restart();
}

#include "esp_timer.h"

bool log(const LogLevel level, const char msg[], const char file[], const char function[], uint32_t line)
{
    if (setupDone)
    {
        rcl_interfaces__msg__Log logger_msg;

        logger_msg.name.data = const_cast<char *>("PCC");
        logger_msg.name.size = 3;

        logger_msg.stamp.nanosec = esp_timer_get_time() * 1000;

        logger_msg.level = level;
        logger_msg.msg.data = const_cast<char *>(msg);
        logger_msg.msg.size = strlen(msg);
        logger_msg.file.data = const_cast<char *>(file);
        logger_msg.file.size = strlen(file);
        logger_msg.function.data = const_cast<char *>(function);
        logger_msg.function.size = strlen(function);
        logger_msg.line = line;

        rcl_ret_t rc = rcl_publish(&loggerPublisher, &logger_msg, nullptr);
        return rc == RCL_RET_OK;
    }
    return false;
}

void blinkError(rcl_ret_t error)
{
    int digit;
    while (error > 0)
    {
        digit = error % 10;
        error /= 10;

        statusStrip->fill(255, 0, 0);
        statusStrip->show();
        gpio_set_level(LED_PIN, 0);
        rclc_sleep_ms(2500);
        while (digit > 0)
        {
            rclc_sleep_ms(1000);
            gpio_set_level(LED_PIN, 1);
            rclc_sleep_ms(500);
            gpio_set_level(LED_PIN, 0);
            digit--;
        }
    }
    rclc_sleep_ms(2500);
}

bool handleError(const int32_t rc,
                 const bool is_ros, const bool do_reset,
                 const char file[], const char function[], uint32_t line)
{
    if (rc != ESP_OK)
    {
        char message[32];
        if (is_ros)
        {
            snprintf(message, sizeof(message), "ROS ERROR: %li", rc);
        }
        else
        {
            esp_err_to_name_r(rc, message, sizeof(message));
        }

        if (do_reset)
        {
            log(LOGLEVEL_FATAL, message, file, function, line);
#if (ENABLE_BLINK_ERROR == 1)
            blinkError(rc);
#endif
            reset();
        }
        else
        {
#if (ENABLE_BLINK_ERROR == 1)
            bool can_log = log(LOGLEVEL_ERROR, message, file, function, line);
            if (!can_log)
            {
                blinkError(rc);
            }
#else
            log(LOGLEVEL_ERROR, message, file, function, line);
#endif
        }
    }
    return rc == RCL_RET_OK;
}

void pingTimerCallback(__attribute__((unused)) rcl_timer_t *timer, __attribute__((unused)) int64_t last_call_time)
{
    LOG(LOGLEVEL_INFO, "Running...");
    if (resetScheduled)
    {
        LOG(LOGLEVEL_INFO, "Running scheduled reset");
        setupDone = false;
        cleanupFunc();
        reset();
        return;
    }

    rmw_ret_t ping_result = rmw_uros_ping_agent(100, 2);
    if (ping_result == RMW_RET_ERROR)
    {
        reset();
    }
}

void resetCallback(__attribute__((unused)) const void *request, void *response)
{
    auto response_msg = (std_srvs__srv__Trigger_Response *) response;
    response_msg->success = true;
    response_msg->message.data = const_cast<char *>("Reset scheduled");
    response_msg->message.size = 15;

    resetScheduled = true;
    LOG(LOGLEVEL_INFO, "Reset scheduled");
}

void setupSystem(rclc_support_t *support, rclc_executor_t *executor)
{
    HANDLE_ROS_ERROR(rclc_timer_init_default(
            &pingTimer,
            support,
            RCL_MS_TO_NS(500),
            &pingTimerCallback), true);
    HANDLE_ROS_ERROR(rclc_executor_add_timer(executor, &pingTimer), true);

    HANDLE_ROS_ERROR(rclc_node_init_default(&systemNode, "pcc_system", "pcc", support), true);
    HANDLE_ROS_ERROR(rclc_publisher_init_default(&loggerPublisher,
                                                 &systemNode,
                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
                                                 "/rosout"), true);
    LOG(LOGLEVEL_INFO, "Logger started");

    HANDLE_ROS_ERROR(rclc_service_init_best_effort(&resetService,
                                                   &systemNode,
                                                   ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                                   "reset"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service(executor, &resetService,
                                               &resetServiceRequest, &resetServiceResponse,
                                               resetCallback), true);
    LOG(LOGLEVEL_DEBUG, "Set up reset service");
}

void cleanupSystem()
{
    HANDLE_ROS_ERROR(rcl_service_fini(&resetService, &systemNode), false);
    HANDLE_ROS_ERROR(rcl_publisher_fini(&loggerPublisher, &systemNode), false);
    HANDLE_ROS_ERROR(rcl_node_fini(&systemNode), false);
}

int vprintfLog(const char *format, va_list arg)
{
    int length = -1;
    if (oldLogger != nullptr)
    {
        length = oldLogger(format, arg);

        char* buffer = new char[length + 1];
        vsnprintf(buffer, length, format, arg);

        log(LOGLEVEL_DEBUG, buffer);

        delete[] buffer;
    }

    return length;
}

void setupThread(__attribute((unused)) void *arg)
{
    statusStrip->fill(255, 16, 0);
    statusStrip->show();
    while (true)
    {
        rmw_ret_t ping_result = rmw_uros_ping_agent(200, 5);
        if (ping_result == RMW_RET_OK)
        {
            statusStrip->fill(0, 0, 0);
            statusStrip->show();
            setupDone = true;
            setupFunc();

            vTaskDelete(nullptr);
            return;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void initSystem(void (*setup_func)(), void (*cleanup_func)())
{
    setupFunc = setup_func;
    cleanupFunc = cleanup_func;

    ESP_LOGI("sys_log", "Starting logging to /rosout");
    oldLogger = esp_log_set_vprintf(&vprintfLog);
    ESP_LOGI("sys_log", "ESP log started");

    statusStrip->fill(0, 0, 0);
    statusStrip->show();

    xTaskCreate(setupThread,
                "system_setup",
                16000,
                nullptr,
                5,
                nullptr);
}
