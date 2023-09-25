#include "nodes/servo.hpp"

#include "system.hpp"

#define SERVO_CONTROLLER_DIAGNOSTIC_UPDATER_ID 1
#define SERVO_CONTROLLER_DIAGNOSTIC_HARDWARE_ID 1
#define SERVO_CONTROLLER_STATE_KEY 0

#define SERVO_DRIVER_ADDRESS 0x40
#define PWM_FREQ 50

ServoNode::ServoNode(gpio_num_t sda, gpio_num_t scl, i2c_port_t port) : Node("pcc_servo", "servo"),
                                                                        sda(sda), scl(scl), port(port),
                                                                        device(),
                                                                        enableService(), setPosService(),
                                                                        enableRequest(), setPosRequest(),
                                                                        enableResponse(), setPosResponse()
{
    HANDLE_ESP_ERROR(pca9685_init_desc(&device, SERVO_DRIVER_ADDRESS, port, sda, scl), true);

    diagnosticTask.context = this;

    setupServoController();
}

void ServoNode::setup(rclc_support_t *support, rclc_executor_t *executor)
{
    LOG(LOGLEVEL_INFO, "Setting up ServoNode");

    HANDLE_ROS_ERROR(rclc_diagnostic_updater_init(&diagnosticUpdater, &node, executor), true);
    HANDLE_ROS_ERROR(rclc_diagnostic_task_init(&diagnosticTask.diagnosticTask,
                                               SERVO_CONTROLLER_DIAGNOSTIC_HARDWARE_ID,
                                               SERVO_CONTROLLER_DIAGNOSTIC_UPDATER_ID,
                                               DIAGNOSTIC_CONTEXT_TASK_CALLBACK(ServoNode, diagnosticTaskCallback)), true);
    HANDLE_ROS_ERROR(rclc_diagnostic_updater_add_task(&diagnosticUpdater, &diagnosticTask.diagnosticTask), true);

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

    running = true;
    xTaskCreate(CONTEXT_TASK_CALLBACK(ServoNode, statusThread),
                "servo_status",
                configMINIMAL_STACK_SIZE,
                this,
                4,
                nullptr);
}

void ServoNode::cleanup(rclc_executor_t *executor)
{
    running = false;
    HANDLE_ESP_ERROR(pca9685_sleep(&device, true), false);
    LOG(LOGLEVEL_DEBUG, "Cleaning up ServoNode");

    HANDLE_ROS_ERROR(rcl_service_fini(&setPosService, &node), false);
    HANDLE_ROS_ERROR(rcl_service_fini(&enableService, &node), false);

    HANDLE_ROS_ERROR(rclc_diagnostic_updater_fini(&diagnosticUpdater, &node, executor), false);

    Node::cleanup(executor);
}

void ServoNode::setupServoController()
{
    HANDLE_ESP_ERROR(pca9685_init(&device), true);
    HANDLE_ESP_ERROR(pca9685_set_pwm_frequency(&device, PWM_FREQ), true);
    HANDLE_ESP_ERROR(pca9685_set_pwm_value(&device, PCA9685_CHANNEL_ALL, 0), true);
    HANDLE_ESP_ERROR(pca9685_sleep(&device, true), true);
}

void ServoNode::statusThread()
{
    while (running)
    {
        // Check connection
        __attribute__((unused)) bool sleeping;
        connectionState = HANDLE_ESP_ERROR(pca9685_is_sleeping(&device, &sleeping), false);

        HANDLE_ROS_ERROR(rclc_diagnostic_updater_update(&diagnosticUpdater), false);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

rcl_ret_t ServoNode::diagnosticTaskCallback(diagnostic_value_t *diagnostic_values, uint8_t *number_of_values)
{
    (void) number_of_values;
    *number_of_values = 1;

    diagnostic_values[0].key = SERVO_CONTROLLER_STATE_KEY;

    rclc_diagnostic_value_set_bool(&diagnostic_values[0], connectionState); //or why this works?
    if (!connectionState)
    {
        rclc_diagnostic_value_set_level(&diagnostic_values[0],
                                        micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__ERROR);
    }
    else
    {
        rclc_diagnostic_value_set_level(&diagnostic_values[0],
                                        micro_ros_diagnostic_msgs__msg__MicroROSDiagnosticStatus__OK);
    }
    return RCL_RET_OK;
    // what else do we have to do
    // first test ok
    // one sec tho
    // we have to add things to the colcon.meta, ?
    // do you have to put the updaters in there?
    // o rihgt that one
    // ok
    // you just increment a number
    // that file preallocates the memory for different things like pubs and subs and services
}

void ServoNode::enableCallback(const void *request, void *response)
{
    if (connectionState)
    {
        auto request_msg = (std_srvs__srv__SetBool_Request *) request;
        auto response_msg = (std_srvs__srv__SetBool_Response *) response;

        response_msg->success = HANDLE_ESP_ERROR(pca9685_sleep(&device, !request_msg->data), false);
        if (!response_msg->success)
        {
            connectionState = false;
            HANDLE_ROS_ERROR(rclc_diagnostic_updater_update(&diagnosticUpdater), false);
        }
        response_msg->message.data = const_cast<char *>(response_msg->success ? "Success" : "Failed");
        response_msg->message.size = response_msg->success ? 7 : 6;
    }
    else
    {
        setupServoController();
    }
}

void ServoNode::setPosCallback(const void *request, void *response)
{
    auto request_msg = (avr_pcc_2023_interfaces__srv__SetServo_Request *) request;
    auto response_msg = (avr_pcc_2023_interfaces__srv__SetServo_Response *) response;

    auto pwm_value = (uint16_t)((float) request_msg->value * ((float) 380 / 255) + 90); // ToDo: Recalibrate range
    // why
    bool success = HANDLE_ESP_ERROR(pca9685_set_pwm_value(&device, request_msg->servo_num, pwm_value), false);
    response_msg->success = success;
}
