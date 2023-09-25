#include "nodes/laser.hpp"

#include "system.hpp"

#define LASER_FIRE_DURATION 250
#define LASER_FIRE_COOLDOWN 750
#define LASER_LOOP_DURATION 100
#define LASER_LOOP_COOLDOWN 500

// ForTesting: Remove neopixel strip
LaserNode::LaserNode(gpio_num_t laser_pin, NeopixelStrip *n) : Node("pcc_laser", "laser"),
                                                               laserPin(laser_pin),
                                                               fireService(), setLoopService(),
                                                               fireRequest(), setLoopRequest(),
                                                               fireResponse(), setLoopResponse()
{
    neo = n; // ForTesting

    const gpio_config_t pin_config = {
            .pin_bit_mask = 1ULL << laserPin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    HANDLE_ESP_ERROR(gpio_config(&pin_config), true);
    HANDLE_ESP_ERROR(gpio_set_level(laserPin, 0), true);
}

void LaserNode::setup(rclc_support_t *support, rclc_executor_t *executor)
{
    LOG(LOGLEVEL_INFO, "Setting up LaserNode");

    Node::setup(support, executor);

    LOG(LOGLEVEL_DEBUG, "Setting up LaserNode: fire service");
    HANDLE_ROS_ERROR(rclc_service_init_default(&fireService,
                                               &node,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                               "fire"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service_with_context(executor,
                                                            &fireService,
                                                            &fireRequest, &fireResponse,
                                                            CONTEXT_SERVICE_CALLBACK(LaserNode, fireCallback),
                                                            this), true);

    LOG(LOGLEVEL_DEBUG, "Setting up LaserNode: set loop service");
    HANDLE_ROS_ERROR(rclc_service_init_default(&setLoopService,
                                               &node,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                               "set_loop"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service_with_context(executor,
                                                            &setLoopService,
                                                            &setLoopRequest, &setLoopResponse,
                                                            CONTEXT_SERVICE_CALLBACK(LaserNode, setLoopCallback),
                                                            this), true);
}

void LaserNode::cleanup(rclc_executor_t *executor)
{
    HANDLE_ESP_ERROR(gpio_set_level(laserPin, 0), false);
    LOG(LOGLEVEL_DEBUG, "Cleaning up LaserNode");

    HANDLE_ROS_ERROR(rcl_service_fini(&setLoopService, &node), false);
    HANDLE_ROS_ERROR(rcl_service_fini(&fireService, &node), false);

    Node::cleanup(executor);
}

void LaserNode::setLaser(bool state)
{
    laserState = state;
//    HANDLE_ESP_ERROR(gpio_set_level(laserPin, state), !state);

    // ForTesting
    if (state)
    {
        neo->fill(0, 255, 0);
    }
    else
    {
        neo->fill(0, 0, 0);
    }
    neo->show(); // ForTesting
}

void LaserNode::tryStartLoop()
{
    if (loopState && !laserState && !cooldownState)
    {
        LOG(LOGLEVEL_DEBUG, "Laser loop: starting loop");

        xTaskCreate(CONTEXT_TASK_CALLBACK(LaserNode, loopThread),
                    "laser_loop",
                    configMINIMAL_STACK_SIZE,
                    this,
                    10,
                    nullptr);
    }
}

void LaserNode::fireThread()
{
    setLaser(true);
    vTaskDelay(LASER_FIRE_DURATION / portTICK_PERIOD_MS);

    cooldownState = true;
    RCL_UNUSED(cooldownState); // Stop unused warnings
    setLaser(false);
    vTaskDelay(LASER_FIRE_COOLDOWN / portTICK_PERIOD_MS);

    cooldownState = false;

    tryStartLoop();

    vTaskDelete(nullptr);
}

void LaserNode::loopThread()
{
    while (loopState)
    {
        setLaser(true);
        vTaskDelay(LASER_LOOP_DURATION / portTICK_PERIOD_MS);

        cooldownState = true;
        RCL_UNUSED(cooldownState); // Stop unused warnings
        setLaser(false);
        vTaskDelay(LASER_LOOP_COOLDOWN / portTICK_PERIOD_MS);

        cooldownState = false;
    }

    LOG(LOGLEVEL_DEBUG, "Laser loop: ended");

    vTaskDelete(nullptr);
}

void LaserNode::fireCallback(__attribute__((unused)) const void *request, void *response)
{
    auto response_msg = (std_srvs__srv__Trigger_Response *) response;
    response_msg->success = false;

    if (!laserState && !loopState && !cooldownState)
    {
        LOG(LOGLEVEL_DEBUG, "Laser fire: starting fire");

        laserState = true;
        xTaskCreate(CONTEXT_TASK_CALLBACK(LaserNode, fireThread),
                    "laser_fire",
                    configMINIMAL_STACK_SIZE,
                    this,
                    10,
                    nullptr);

        response_msg->success = true;
    }

    if (response_msg->success)
    {
        response_msg->message.data = const_cast<char *>("Success");
        response_msg->message.size = 7;
    }
    else if (loopState)
    {
        response_msg->message.data = const_cast<char *>("Loop is on");
        response_msg->message.size = 10;
        LOG(LOGLEVEL_WARN, "Tried to fire laser while loop is on");
    }
    else if (cooldownState)
    {
        response_msg->message.data = const_cast<char *>("Laser is on cooldown");
        response_msg->message.size = 20;
        LOG(LOGLEVEL_WARN, "Tried to fire laser while on cooldown");
    }
    else if (laserState)
    {
        response_msg->message.data = const_cast<char *>("Already firing");
        response_msg->message.size = 14;
        LOG(LOGLEVEL_WARN, "Tried to fire laser while already on");
    }
}

void LaserNode::setLoopCallback(const void *request, void *response)
{
    auto request_msg = (std_srvs__srv__SetBool_Request *) request;
    auto response_msg = (std_srvs__srv__SetBool_Response *) response;

    if (request_msg->data == loopState)
    {
        response_msg->success = false;
        response_msg->message.data = const_cast<char *>("Already set");
        response_msg->message.size = 11;
    }
    else
    {
        loopState = request_msg->data;

        tryStartLoop();

        response_msg->success = true;
        response_msg->message.data = const_cast<char *>("Loop set");
        response_msg->message.size = 8;
    }
}
