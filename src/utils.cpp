#include "utils.hpp"

#define DO_NOTHING(X) {}

rcl_node_t logger_node;
rcl_publisher_t logger_publisher;
rcl_interfaces__msg__Log logger_msg;
int cleanup_callback_index = 0;

CleanupAction cleanupActions[3];

void addCleanup(CleanupAction cleanupAction)
{
    cleanupActions[cleanup_callback_index++] = cleanupAction;
}

void cleanup()
{
    for (int i = sizeof(cleanupActions); i > 0; i--)
    {
        cleanupActions[i].callback(cleanupActions[i].context);
    }
}

[[noreturn]] void doReset()
{
    cleanup();
    Watchdog.disable();
    Watchdog.enable(1);
    while (true)
    {
    }
}

void reset()
{
    digitalWrite(LED_BUILTIN, 1);
    delay(100);
    digitalWrite(LED_BUILTIN, 0);
    delay(200);
    digitalWrite(LED_BUILTIN, 1);
    doReset();
}

void loggingReset()
{
    digitalWrite(LED_BUILTIN, 1);
    for (int i = 0; i < 3; i++)
    {
        delay(200);
        digitalWrite(LED_BUILTIN, 0);
        delay(10);
        digitalWrite(LED_BUILTIN, 1);
    }
    doReset();
}

void log(LogLevel level, const char msg[])
{
    logger_msg.level = level;
    logger_msg.msg.data = (char*) msg;
    logger_msg.msg.size = strlen(msg);
    DO_NOTHING(rcl_publish(&logger_publisher, &logger_msg, NULL))
}

void initLogger(rclc_support_t* support)
{
    rcl_ret_t rc = rclc_node_init_default(&logger_node, "pcc-logger", "pcc-logger", support);
    if (rc != RCL_RET_OK)
    {
        loggingReset();
        return;
    }
    CLEANUP_ACTION(nullptr, [](Node* _) { return rcl_node_fini(&logger_node); })

    rc = rclc_publisher_init_default(&logger_publisher,
                                     &logger_node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
                                     "rosout");
    if (rc != RCL_RET_OK)
    {
        loggingReset();
        return;
    }
    CLEANUP_ACTION(nullptr, [](Node* _) { return rcl_publisher_fini(&logger_publisher, &logger_node); })

    logger_msg.name.data = (char*) "PCC ";
    logger_msg.name.size = sizeof(logger_msg.name.data);
}

void handleError(rcl_ret_t error, bool do_reset)
{
    if (error != RCL_RET_OK)
    {
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