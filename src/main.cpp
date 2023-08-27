#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "utils.hpp"

#define NUM_TIMERS 0
#define NUM_SUBSCRIPTIONS 0
#define NUM_SERVICES 0
#define NUM_EXECUTOR_HANDLES (SYSTEM_EXECUTOR_HANDLES + NUM_TIMERS + NUM_SUBSCRIPTIONS + NUM_SERVICES)

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

[[maybe_unused]] void setup()
{
    digitalWrite(LED_BUILTIN, 0);
    beginOnboardNeopixel();

    while (!Serial)
    {
        yield();
    }
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    handleEarlyError(rclc_support_init(&support, 0, nullptr, &allocator));

    executor = rclc_executor_get_zero_initialized_executor();
    handleEarlyError(rclc_executor_init(&executor, &support.context, NUM_EXECUTOR_HANDLES, &allocator));

    initSystem(&support, &executor);

    setOnboardNeopixel(0, 0, 0);
    digitalWrite(LED_BUILTIN, 1);
    LOG(LogLevel::INFO, "Setup complete");
}

[[maybe_unused]] void loop()
{
    handleError(rclc_executor_spin(&executor));
}
