#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "utils.hpp"

#define NUM_TIMERS 0
#define NUM_SUBSCRIPTIONS 0
#define NUM_SERVICES 2
#define NUM_EXECUTOR_HANDLES (NUM_TIMERS + NUM_SUBSCRIPTIONS + NUM_SERVICES)

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

void setup()
{
    digitalWrite(LED_BUILTIN, 0);

    while (!Serial)
        yield();
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rcl_ret_t rc = rclc_support_init(&support, 0, nullptr, &allocator);
    if (rc != RCL_RET_OK)
    {
        reset();
        return;
    }

    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, NUM_EXECUTOR_HANDLES, &allocator);

    initSystemNode(&support, &executor);

    digitalWrite(LED_BUILTIN, 1);
    log(LogLevel::INFO, "Setup complete");
}

[[maybe_unused]] void loop()
{
    rclc_executor_spin(&executor);
}
