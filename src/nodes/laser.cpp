#include "nodes/laser.hpp"

#include <Arduino.h>
#include "utils.hpp"

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
    HANDLE_ERROR(rcl_timer_cancel(&fireOffTimer.timer), true);
    HANDLE_ERROR(rclc_executor_add_timer(executor, &fireOffTimer.timer), true);

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
    HANDLE_ERROR(rcl_timer_cancel(&fireCooldownTimer.timer), true);
    HANDLE_ERROR(rclc_executor_add_timer(executor, &fireCooldownTimer.timer), true);

    HANDLE_ERROR(rclc_service_init_default(&fireService,
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
                                                        CONTEXT_SERVICE_CALLBACK(LaserNode, fire), this), true);

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

void LaserNode::fire(__attribute__((unused)) const void *request, void *response)
{
    auto response_msg = (std_srvs__srv__Trigger_Response *) response;
    response_msg->success = false;

    if (!laserState && !loopState && !cooldownState)
    {
        if (HANDLE_ERROR(rcl_timer_reset(&fireOffTimer.timer), false))
        {
            setLaser(true);
            response_msg->success = true;
            LOG(LogLevel::DEBUG, "Laser fire on");
        }
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
        LOG(LogLevel::WARN, "Tried to fire laser while loop is on");
    }
    else if (cooldownState)
    {
        response_msg->message.data = const_cast<char *>("Laser is on cooldown");
        response_msg->message.size = 20;
        LOG(LogLevel::WARN, "Tried to fire laser while on cooldown");
    }
    else if (laserState)
    {
        response_msg->message.data = const_cast<char *>("Already firing");
        response_msg->message.size = 14;
        LOG(LogLevel::WARN, "Tried to fire laser while already on");
    }
}

bool LaserNode::setLoop(bool state)
{
    loopState = state;
    return true;
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

void LaserNode::setLoopCallback(const std_srvs__srv__SetBool_Request *request_msg,
                                std_srvs__srv__SetBool_Response *response_msg,
                                LaserNode *laser_node)
{

}
