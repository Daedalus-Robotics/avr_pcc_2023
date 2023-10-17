#include <i2cdev.h>
#include <gpio_cxx.hpp>
#include <driver/gpio.h>
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
#include "nodes/led_strip.hpp"
#include "nodes/servo_node.hpp"
#include "nodes/thermal_camera.hpp"

#ifndef RMW_UXRCE_TRANSPORT_CUSTOM
#error micro-ROS transports misconfigured
#endif

#define EXECUTOR_HANDLES (SYSTEM_EXECUTOR_HANDLES + \
                          LASER_NODE_EXECUTOR_HANDLES + \
                          LED_STRIP_NODE_EXECUTOR_HANDLES + \
                          SERVO_NODE_EXECUTOR_HANDLES + \
                          THERMAL_CAMERA_NODE_EXECUTOR_HANDLES)

static const size_t uartPort = UART_NUM_0;

NeopixelStrip *strip;

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

LaserNode *laserNode;
LedStripNode *ledStripNode;
ServoNode *servoNode;
ThermalCameraNode *thermalCameraNode;

void setup()
{
    // Init support
    HANDLE_ROS_ERROR(rclc_support_init(&support, 0, nullptr, &allocator), true);

    // Init executor
    HANDLE_ROS_ERROR(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator), true);

    setupSystem(&support, &executor);

    ESP_LOGI("agent", "Connected to micro-ros agent");

    laserNode->setup(&support, &executor);
    ledStripNode->setup(&support, &executor);
    servoNode->setup(&support, &executor);
    thermalCameraNode->setup(&support, &executor);

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

    thermalCameraNode->cleanup();
    servoNode->cleanup();
    ledStripNode->cleanup();
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

    i2cdev_init();

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

    laserNode = new LaserNode(GPIO_NUM_4);
    ledStripNode = new LedStripNode(strip);
    servoNode = new ServoNode(GPIO_NUM_23 , GPIO_NUM_22, I2C_NUM_0);
    thermalCameraNode = new ThermalCameraNode(idf::GPIONumBase<idf::SDA_type>(GPIO_NUM_18),
                                              idf::GPIONumBase<idf::SCL_type>(GPIO_NUM_19),
                                              idf::I2CNumber::I2C1());
}
