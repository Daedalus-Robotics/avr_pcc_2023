#include "utils.hpp"

/**
 * If this is 1, errors will turn the neopixel red and blink the
 * led for each digit in the error code in reverse order
 * @example If the error is 123, it will blink 3 times, 2 times, then 1 time
 */
#define ENABLE_BLINK_ERROR 0
#define DO_NOTHING(X) {}

#if ENABLE_BLINK_ERROR == 1
#define MAYBE_BLINK_ERROR(error) { blinkError(error); }
#else
#define MAYBE_BLINK_ERROR(error) {}
#endif

Adafruit_NeoPixel onboardNeopixel(1, 8, NEO_GRB);

int cleanupCallbackIndex = 0;
CleanupAction cleanupActions[15];

rcl_node_t systemNode;

rcl_publisher_t loggerPublisher;
rcl_interfaces__msg__Log loggerMsg;

rcl_service_t rebootService;
std_srvs__srv__Empty_Request rebootServiceRequest;
std_srvs__srv__Empty_Request rebootServiceResponse;

rcl_service_t shutdownService;
std_srvs__srv__Empty_Request shutdownServiceRequest;
std_srvs__srv__Empty_Request shutdownServiceResponse;

void setOnboardNeopixel(uint8_t r, uint8_t g, uint8_t b)
{
    onboardNeopixel.setPixelColor(0, (r << 16) | (g << 8) | b);
    onboardNeopixel.show();
}

void beginOnboardNeopixel()
{
    onboardNeopixel.begin();
    onboardNeopixel.setBrightness(20);
    setOnboardNeopixel(255, 95, 0);
}

void addCleanup(CleanupAction cleanup_action)
{
    cleanupActions[cleanupCallbackIndex++] = cleanup_action;
}

void cleanup()
{
    for (int i = cleanupCallbackIndex; i > 0; i--)
    {
        cleanupActions[i].callback(cleanupActions[i].context);
    }
}

[[noreturn]] void shutdown()
{
    setOnboardNeopixel(16, 0, 16);
    cleanup();
    while (true)
    {
    }
}

[[noreturn]] void doReset()
{
    // Ensure that the cleanup doesn't stop the reset
    Watchdog.enable(5000);
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

void loggingReset()
{
    setOnboardNeopixel(255, 16, 0);
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

void log(LogLevel level, const char msg[])
{
    loggerMsg.level = level;
    loggerMsg.msg.data = (char *) msg;
    loggerMsg.msg.size = strlen(msg);
    DO_NOTHING(rcl_publish(&loggerPublisher, &loggerMsg, NULL))
}

void rebootCallback(__attribute__((unused)) const void *request_msg, __attribute__((unused)) void *response_msg)
{
    reset();
}

void shutdownCallback(__attribute__((unused)) const void *request_msg, __attribute__((unused)) void *response_msg)
{
    shutdown();
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

void initSystemNode(rclc_support_t *support, rclc_executor_t *executor)
{
    rcl_ret_t rc = rclc_node_init_default(&systemNode, "pcc-system", "pcc-system", support);
    if (rc != RCL_RET_OK)
    {
        MAYBE_BLINK_ERROR(rc)
        loggingReset();
        return;
    }
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_node_fini(&systemNode); })

    rc = rclc_publisher_init_default(&loggerPublisher,
                                     &systemNode,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
                                     "rosout");
    if (rc != RCL_RET_OK)
    {
        MAYBE_BLINK_ERROR(rc)
        loggingReset();
        return;
    }
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_publisher_fini(&loggerPublisher, &systemNode); })

    loggerMsg.name.data = (char *) "PCC ";
    loggerMsg.name.size = sizeof(loggerMsg.name.data);

    handleError(rclc_service_init_default(&rebootService,
                                          &systemNode,
                                          ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
                                          "reboot"), true);
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_service_fini(&rebootService, &systemNode); })
    handleError(rclc_executor_add_service(executor, &rebootService,
                                          &rebootServiceRequest, &rebootServiceResponse,
                                          rebootCallback), true);

    handleError(rclc_service_init_default(&shutdownService,
                                          &systemNode,
                                          ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
                                          "shutdown"), true);
    CLEANUP_ACTION(nullptr, [](Node *_) { return rcl_service_fini(&shutdownService, &systemNode); })
    handleError(rclc_executor_add_service(executor, &shutdownService,
                                          &shutdownServiceRequest, &shutdownServiceResponse,
                                          shutdownCallback), true);
}

void handleError(rcl_ret_t error, bool do_reset)
{
    if (error != RCL_RET_OK)
    {
        MAYBE_BLINK_ERROR(error)
        static char message[12];
        snprintf(message, sizeof(message), "ERROR: %li", error);
        if (do_reset)
        {
            log(LogLevel::FATAL, message);
            reset();
        }
        else
        {
            log(LogLevel::ERROR, message);
        }
    }
}
