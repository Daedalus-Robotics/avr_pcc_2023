#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>

#include "esp32_serial_transport.hpp"
#include "neopixel_strip.hpp"
#include "system.hpp"
#include "nodes/laser.hpp"

#ifndef RMW_UXRCE_TRANSPORT_CUSTOM
#error micro-ROS transports misconfigured
#endif

#define EXECUTOR_HANDLES (SYSTEM_EXECUTOR_HANDLES + LASER_NODE_EXECUTOR_HANDLES)

static const size_t uartPort = UART_NUM_0;

NeopixelStrip *strip;

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

LaserNode *laserNode;

void setup()
{
    // Init support
    HANDLE_ROS_ERROR(rclc_support_init(&support, 0, nullptr, &allocator), true);

    // Init executor
    HANDLE_ROS_ERROR(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator), true);

    setupSystem(&support, &executor);

    ESP_LOGI("agent", "Connected to micro-ros agent");

    laserNode->setup(&support, &executor);

    // Spin the executor
    xTaskCreate([](void *arg) { rclc_executor_spin(&executor); },
                "uros_executor",
                16000,
                nullptr,
                3,
                nullptr);

    HANDLE_ESP_ERROR(gpio_set_level(LED_PIN, 1), true);

    LOG(LOGLEVEL_INFO, "Setup complete");
}

void cleanup()
{
    ESP_LOGI("agent", "Disconnected from micro-ros agent");

    laserNode->cleanup();

    cleanupSystem();
}

extern "C" [[maybe_unused]] void app_main()
{
    // Setup status led
    const gpio_config_t led_pin_config = {
            1ULL << LED_PIN,
            GPIO_MODE_OUTPUT,
            GPIO_PULLUP_DISABLE,
            GPIO_PULLDOWN_DISABLE,
            GPIO_INTR_DISABLE
    };
    HANDLE_ESP_ERROR(gpio_config(&led_pin_config), true);
    HANDLE_ESP_ERROR(gpio_set_level(LED_PIN, 0), true);

    // Setup neopixel strip
    strip = new NeopixelStrip(GPIO_NUM_12, NEOPIXEL_TYPE_WS2812, 30);
    strip->fill(75, 0, 255);
    strip->show();
    setStatusStrip(strip);

    // Setup serial transport
    HANDLE_ROS_ERROR(rmw_uros_set_custom_transport(
            true,
            (void *) &uartPort,
            esp32SerialOpen,
            esp32SerialClose,
            esp32SerialWrite,
            esp32SerialRead
    ), true);

    // Get allocator
    allocator = rcl_get_default_allocator();

    initSystem(&setup, &cleanup);

    laserNode = new LaserNode(GPIO_NUM_0, strip); // ForTesting
}
