#include "utils.hpp"

/**
 * If this is 1, errors will turn the neopixel red and blink the
 * led for each digit in the error code in reverse order
 * @example If the error is 123, it will blink 3 times, 2 times, then 1 time
 */
#define ENABLE_BLINK_ERROR 0

Adafruit_NeoPixel onboardNeopixel(1, 8, NEO_GRB);

int cleanupCallbackIndex = 0;
CleanupAction cleanupActions[15];

rcl_timer_t pingTimer;

rcl_node_t systemNode;

rcl_publisher_t loggerPublisher;
rcl_interfaces__msg__Log loggerMsg;

rcl_service_t resetService;
std_srvs__srv__Empty_Request resetServiceRequest;
std_srvs__srv__Empty_Request resetServiceResponse;

rcl_service_t cleanupService;
std_srvs__srv__Empty_Request cleanupServiceRequest;
std_srvs__srv__Empty_Request cleanupServiceResponse;

void setOnboardNeopixel(uint8_t r, uint8_t g, uint8_t b)
{
    onboardNeopixel.setPixelColor(0, (r << 16) | (g << 8) | b);
    onboardNeopixel.show();
}

void beginOnboardNeopixel()
{
    onboardNeopixel.begin();
    onboardNeopixel.setBrightness(100);
    setOnboardNeopixel(255, 95, 0);
}

void addCleanup(CleanupAction cleanup_action)
{
    cleanupActions[cleanupCallbackIndex++] = cleanup_action;
}

void cleanup()
{
    for (int i = cleanupCallbackIndex - 1; i >= 0; i--)
    {
        cleanupActions[i].callback(cleanupActions[i].context);
    }
}

[[noreturn]] void forceReset()
{
    Watchdog.enable(1);
    while (true)
    {
    }
}

[[noreturn]] void doReset()
{
    // Ensure that the cleanup doesn't stop the reset
    Watchdog.enable(8000);
    cleanup();
    Watchdog.enable(1);
    while (true)
    {
    }
}

void reset()
{
    setOnboardNeopixel(255, 16, 0);
    digitalWrite(LED_BUILTIN, 1);
    delay(50);
    digitalWrite(LED_BUILTIN, 0);
    delay(50);
    digitalWrite(LED_BUILTIN, 1);
    doReset();
}

bool log(LogLevel level, const char msg[], const char file[], const char function[], uint32_t line)
{
    loggerMsg.level = level;
    loggerMsg.msg.data = const_cast<char *>(msg);
    loggerMsg.msg.size = strlen(msg);
    loggerMsg.file.data = const_cast<char *>(file);
    loggerMsg.file.size = strlen(file);
    loggerMsg.function.data = const_cast<char *>(function);
    loggerMsg.function.size = strlen(function);
    loggerMsg.line = line;

    rcl_ret_t rc = rcl_publish(&loggerPublisher, &loggerMsg, nullptr);
    return rc == RCL_RET_OK;
}

void blinkError(rcl_ret_t error)
{
    int digit;
    while (error > 0)
    {
        digit = error % 10;
        error /= 10;

        setOnboardNeopixel(255, 0, 0);
        digitalWrite(LED_BUILTIN, 0);
        delay(2500);
        while (digit > 0)
        {
            delay(1000);
            digitalWrite(LED_BUILTIN, 1);
            delay(500);
            digitalWrite(LED_BUILTIN, 0);
            digit--;
        }
    }
    delay(2000);
}

void pingTimerCallback(__attribute__((unused)) rcl_timer_t *timer,
                       __attribute__((unused)) int64_t time_since_last_call)
{
    rmw_ret_t ping_result = rmw_uros_ping_agent(10, 1);
    if (ping_result != RMW_RET_OK)
    {
        forceReset(); //ToDo: Only reset when the connection comes back
    }
}

void resetCallback(__attribute__((unused)) const void *request_msg,
                   __attribute__((unused)) void *response_msg)
{
    reset(); //ToDo: Use a timer or something to make sure the response is returned
}

[[noreturn]] void cleanupCallback(__attribute__((unused)) const void *request_msg,
                                  __attribute__((unused)) void *response_msg)
{
    cleanup();
    while (true)
    {
    }
}

void handleEarlyError(rcl_ret_t rc)
{
    if (rc != RCL_RET_OK)
    {
#if (ENABLE_BLINK_ERROR == 1)
        blinkError(rc);
#endif
        setOnboardNeopixel(255, 0, 25);
        digitalWrite(LED_BUILTIN, 1);
        for (int i = 0; i < 3; i++)
        {
            delay(200);
            digitalWrite(LED_BUILTIN, 0);
            delay(50);
            digitalWrite(LED_BUILTIN, 1);
        }
        doReset();
    }
}

bool handleError(rcl_ret_t rc, const char file[], const char function[], uint32_t line, bool do_reset)
{
    if (rc != RCL_RET_OK)
    {
        static char message[12];
        snprintf(message, sizeof(message), "ERROR: %li", rc);
        if (do_reset)
        {
            log(LogLevel::FATAL, message, file, function, line);
#if (ENABLE_BLINK_ERROR == 1)
            blinkError(rc);
#endif
            reset();
        }
        else
        {
#if (ENABLE_BLINK_ERROR == 1)
            bool can_log = log(LogLevel::ERROR, message, message, file, function, line);
            if (!can_log)
            {
                blinkError(rc);
            }
#else
            log(LogLevel::ERROR, message, file, function, line);
#endif
        }
    }
    return rc == RCL_RET_OK;
}

void initSystem(rclc_support_t *support, rclc_executor_t *executor)
{
    handleEarlyError(rclc_timer_init_default(&pingTimer, support, RCL_MS_TO_NS(1000), pingTimerCallback));
    handleEarlyError(rclc_executor_add_timer(executor, &pingTimer));
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_timer_fini(&pingTimer); });

    handleEarlyError(rclc_node_init_default(&systemNode, "pcc_system", "pcc", support));
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_node_fini(&systemNode); });

    handleEarlyError(rclc_publisher_init_default(&loggerPublisher,
                                                 &systemNode,
                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
                                                 "/rosout"));
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_publisher_fini(&loggerPublisher, &systemNode); });

    loggerMsg.name.data = const_cast<char *>("PCC");
    loggerMsg.name.size = sizeof(loggerMsg.name.data);

    LOG(LogLevel::INFO, "Logger started");

    HANDLE_ERROR(rclc_service_init_best_effort(&resetService,
                                               &systemNode,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
                                               "reset"), true);
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_service_fini(&resetService, &systemNode); });
    HANDLE_ERROR(rclc_executor_add_service(executor, &resetService,
                                           &resetServiceRequest, &resetServiceResponse,
                                           resetCallback), true);
    LOG(LogLevel::DEBUG, "Set up reset service");

    HANDLE_ERROR(rclc_service_init_best_effort(&cleanupService,
                                               &systemNode,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
                                               "cleanup"), true);
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_service_fini(&cleanupService, &systemNode); });
    HANDLE_ERROR(rclc_executor_add_service(executor, &cleanupService,
                                           &cleanupServiceRequest, &cleanupServiceResponse,
                                           cleanupCallback), true);
    LOG(LogLevel::DEBUG, "Set up cleanup service");
}
