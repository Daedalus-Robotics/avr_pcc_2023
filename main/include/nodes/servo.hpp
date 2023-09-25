#include <i2cdev.h>
#include <pca9685.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/set_bool.h>
#include <avr_pcc_2023_interfaces/srv/set_servo.h>

extern "C"
{
#include <micro_ros_diagnostic_updater/micro_ros_diagnostic_updater.h>
}

#include "node.hpp"
#include "diagnostic_context_task.hpp"

#define SERVO_NODE_EXECUTOR_HANDLES 4

#ifndef AVR_PCC_2023_SERVO_NODE_HPP
#define AVR_PCC_2023_SERVO_NODE_HPP


class ServoNode : Node
{
public:
    ServoNode(gpio_num_t sda, gpio_num_t scl, i2c_port_t port = I2C_NUM_0);

    void setup(rclc_support_t *support, rclc_executor_t *executor) override;

    void cleanup(rclc_executor_t *executor) override;

private:
    // services, topics, subscribers, clients, and variables defined here
    // 1. from constructor (try to make it constant)
    const gpio_num_t sda;
    const gpio_num_t scl;
    const i2c_port_t port;

    // 2. things you instantiate (services)
    i2c_dev_t device;
    rcl_service_t enableService;
    rcl_service_t setPosService;
    diagnostic_updater_t diagnosticUpdater;
    DiagnosticTaskWithContext diagnosticTask;

    bool running = false;
    bool connectionState; //is this what we're checking?
//    rcl_service_t setDefaultService; // save default val to flash mem and read on startup
    std_srvs__srv__SetBool_Request enableRequest;
    avr_pcc_2023_interfaces__srv__SetServo_Request setPosRequest;
    std_srvs__srv__SetBool_Response enableResponse;
    avr_pcc_2023_interfaces__srv__SetServo_Response setPosResponse;

    // 3. things that happen at runtime
    // 4. private functions (callbacks for services and such)

    void setupServoController();

    void statusThread();

    rcl_ret_t diagnosticTaskCallback(diagnostic_value_t *diagnostic_values, uint8_t *number_of_values);


    void enableCallback(const void *request, void *response);

    void setPosCallback(const void *request, void *response);
};


#endif //AVR_PCC_2023_SERVO_NODE_HPP
