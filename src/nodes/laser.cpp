#include "nodes/laser.hpp"

LaserNode::LaserNode(
        rclc_support_t *support,
        rclc_executor_t *executor,
        uint32_t laser_pin) : Node(support, "pcc_laser", "laser"),
                              laserPin(laser_pin),
                              fireService(), setLoopService(),
                              fireRequest(), setLoopRequest(),
                              fireResponse(), setLoopResponse()
{
    digitalWrite(laserPin, 0);
    CLEANUP_ACTION(this, [](Node *context)
    {
        auto *laser_node = (LaserNode * )(context);
        digitalWrite(laser_node->laserPin, 0);
        return static_cast<rcl_ret_t>(RCL_RET_OK);
    });

    HANDLE_ERROR(rclc_service_init_best_effort(&fireService,
                                               &node,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                               "fire"), true);
    CLEANUP_ACTION(this, [](Node *context)
    {
        auto *laser_node = (LaserNode * )(context);
        return rcl_service_fini(&laser_node->fireService, &laser_node->node);
    });
    HANDLE_ERROR(rclc_executor_add_service_with_context(executor,
                                                        &fireService,
                                                        &fireRequest, &fireResponse,
                                                        SERVICE_CALLBACK_CONTEXT(&LaserNode::fireCallback),
                                                        this), true);

    HANDLE_ERROR(rclc_service_init_default(&setLoopService,
                                           &node,
                                           ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                           "set_loop"), true);
    CLEANUP_ACTION(this, [](Node *context)
    {
        auto *laser_node = (LaserNode * )(context);
        return rcl_service_fini(&laser_node->setLoopService, &laser_node->node);
    });
    HANDLE_ERROR(rclc_executor_add_service_with_context(executor,
                                                        &setLoopService,
                                                        &setLoopRequest, &setLoopResponse,
                                                        SERVICE_CALLBACK_CONTEXT(&LaserNode::setLoopCallback),
                                                        this), true);
}

bool LaserNode::fire()
{

}

bool LaserNode::setLoop(bool state)
{

}

void LaserNode::setLaser(bool state)
{
    digitalWrite(laserPin, state);
    if (state)
    {
        setOnboardNeopixel(200, 100, 0);
    }
    else
    {
        setOnboardNeopixel(0, 0, 0);
    }
}

void LaserNode::fireCallback(const std_srvs__srv__Trigger_Request *request_msg,
                             std_srvs__srv__Trigger_Response *response_msg,
                             LaserNode laser_node)
{

}

void LaserNode::setLoopCallback(const std_srvs__srv__SetBool_Request *request_msg,
                                std_srvs__srv__SetBool_Response *response_msg,
                                LaserNode laser_node)
{

}
