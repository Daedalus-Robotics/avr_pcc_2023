#include "nodes/laser.hpp"

LaserNode::LaserNode(
        rclc_support_t *support,
        rclc_executor_t *executor,
        uint32_t laser_pin) : Node(support, "pcc_laser", "laser"),
                              laserPin(laser_pin),
                              fireOffTimer(), fireCooldownTimer(),
                              loopOnTimer(), loopOffTimer(), loopCooldownTimer(),
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

    fireOffTimer.context = this;
    fireCooldownTimer.context = this;
    loopOnTimer.context = this;
    loopOffTimer.context = this;
    loopCooldownTimer.context = this;

    HANDLE_ERROR(rclc_timer_init_default(&fireOffTimer.timer, support, LASER_FIRE_DURATION,
                                         [](rcl_timer_t *timer, __attribute__((unused)) int64_t n)
                                         {
                                             auto context_timer = (TimerWithContext *) timer;
                                             auto context = (LaserNode * )
                                             context_timer->context;

                                             HANDLE_ERROR(rcl_timer_cancel(timer), false);
                                             context->setLaser(false);
                                             context->cooldownState = true;
                                             HANDLE_ERROR(rcl_timer_reset(&context->fireCooldownTimer.timer), false);

                                             LOG(LogLevel::DEBUG, "Laser fire off");
                                         }), true);
    HANDLE_ERROR(rclc_executor_add_timer(executor, &fireOffTimer.timer), false);

    HANDLE_ERROR(rclc_timer_init_default(&fireCooldownTimer.timer, support, LASER_FIRE_COOLDOWN,
                                         [](rcl_timer_t *timer, __attribute__((unused)) int64_t n)
                                         {
                                             auto context_timer = (TimerWithContext *) timer;
                                             auto context = (LaserNode * )
                                             context_timer->context;

                                             HANDLE_ERROR(rcl_timer_cancel(timer), false);
                                             context->cooldownState = false;

                                             LOG(LogLevel::DEBUG, "Laser fire cooldown finished");
                                         }), true);
    HANDLE_ERROR(rclc_executor_add_timer(executor, &fireCooldownTimer.timer), false);

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
                                                        SERVICE_CALLBACK_CONTEXT(&fireCallback), this), true);

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
                                                        SERVICE_CALLBACK_CONTEXT(&setLoopCallback), this), true);
}

bool LaserNode::fire()
{
    if (!laserState && !loopState && !cooldownState)
    {
        if (!HANDLE_ERROR(rcl_timer_reset(&fireOffTimer.timer), false))
        {
            setLaser(true);
            LOG(LogLevel::DEBUG, "Laser fire on");
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool LaserNode::setLoop(bool state)
{

}

void LaserNode::setLaser(bool state)
{
    laserState = state;
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

void LaserNode::fireCallback(__attribute__((unused)) const std_srvs__srv__Trigger_Request *request_msg,
                             std_srvs__srv__Trigger_Response *response_msg,
                             LaserNode laser_node)
{
    bool success = laser_node.fire();
    response_msg->success = success;
    response_msg->message.data = const_cast<char *>(success ? "Success" : "Loop is active");
    response_msg->message.size = sizeof(response_msg->message.data);
}

void LaserNode::setLoopCallback(const std_srvs__srv__SetBool_Request *request_msg,
                                std_srvs__srv__SetBool_Response *response_msg,
                                LaserNode laser_node)
{

}
