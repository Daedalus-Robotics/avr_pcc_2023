#include <Arduino.h>
#include <Adafruit_SleepyDog.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include "utils.hpp"

rcl_allocator_t allocator;
rclc_support_t support;

void setup()
{
    digitalWrite(LED_BUILTIN, 0);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rcl_ret_t rc = rclc_support_init(&support, 0, nullptr, &allocator);
    if (rc != RCL_RET_OK)
    {
        reset();
        return;
    }

    initLogger(&support);

    digitalWrite(LED_BUILTIN, 1);
    log(LogLevel::INFO, "Setup complete");

    Watchdog.enable(2500);
}

void loop()
{
    Watchdog.reset();
}
