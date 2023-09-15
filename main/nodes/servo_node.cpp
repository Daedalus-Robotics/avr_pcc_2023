#include "nodes/servo_node.hpp"

#include "system.hpp"

#define SERVO_DRIVER_ADDRESS 0x40
#define PWM_FREQ 50

ServoNode::ServoNode(gpio_num_t sda, gpio_num_t scl, i2c_port_t port) : Node("pcc_servo", "servo"),
                                                                        device(),
                                                                        enableService(), setPosService(),
                                                                        enableRequest(), setPosRequest(),
                                                                        enableResponse(), setPosResponse()
{
    HANDLE_ESP_ERROR(pca9685_init_desc(&device, SERVO_DRIVER_ADDRESS, port, sda, scl), true);
    HANDLE_ESP_ERROR(pca9685_init(&device), true);
    HANDLE_ESP_ERROR(pca9685_set_pwm_frequency(&device, PWM_FREQ), true);
    HANDLE_ESP_ERROR(pca9685_set_pwm_value(&device, PCA9685_CHANNEL_ALL, 0), true);
    HANDLE_ESP_ERROR(pca9685_sleep(&device, true), true);
}

void ServoNode::setup(rclc_support_t *support, rclc_executor_t *executor)
{
    LOG(LOGLEVEL_INFO, "Setting up ServoNode");

    Node::setup(support, executor);

    LOG(LOGLEVEL_DEBUG, "Setting up ServoNode: enable service");
    HANDLE_ROS_ERROR(rclc_service_init_default(&enableService,
                                               &node,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                               "enable"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service_with_context(executor,
                                                            &enableService,
                                                            &enableRequest,
                                                            &enableResponse,
                                                            CONTEXT_SERVICE_CALLBACK(ServoNode, enableCallback),
                                                            this), true);

    LOG(LOGLEVEL_DEBUG, "Setting up ServoNode: set pos service");
    HANDLE_ROS_ERROR(rclc_service_init_default(&setPosService,
                                               &node,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(avr_pcc_2023_interfaces, srv, SetServo),
                                               "set_position"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service_with_context(executor,
                                                            &setPosService,
                                                            &setPosRequest,
                                                            &setPosResponse,
                                                            CONTEXT_SERVICE_CALLBACK(ServoNode, setPosCallback),
                                                            this), true);
}

void ServoNode::cleanup()
{
    HANDLE_ESP_ERROR(pca9685_sleep(&device, true), false);
    LOG(LOGLEVEL_DEBUG, "Cleaning up ServoNode");

    HANDLE_ROS_ERROR(rcl_service_fini(&setPosService, &node), false);
    HANDLE_ROS_ERROR(rcl_service_fini(&enableService, &node), false);

    Node::cleanup();
}

void ServoNode::enableCallback(const void *request, void *response)
{
    auto request_msg = (std_srvs__srv__SetBool_Request *) request;
    auto response_msg = (std_srvs__srv__SetBool_Response *) response;

    response_msg->success = HANDLE_ESP_ERROR(pca9685_sleep(&device, !request_msg->data), false);
    response_msg->message.data = const_cast<char *>(response_msg->success ? "Success" : "Failed");
    response_msg->message.size = response_msg->success ? 7 : 6;
}

void ServoNode::setPosCallback(const void *request, void *response)
{
    auto request_msg = (avr_pcc_2023_interfaces__srv__SetServo_Request *) request;
    auto response_msg = (avr_pcc_2023_interfaces__srv__SetServo_Response *) response;

    auto pwm_value = (uint16_t)((float) request_msg->value * ((float) 380 / 255) + 90); // ToDo: Recalibrate range
    bool success = HANDLE_ESP_ERROR(pca9685_set_pwm_value(&device, request_msg->servo_num, pwm_value), false);
    response_msg->success = success;
}
