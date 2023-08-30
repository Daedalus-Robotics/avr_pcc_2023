#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>
#include "context_timer.hpp"
#include "node.hpp"

#ifndef AVR_PCC_2023_LASER_HPP
#define AVR_PCC_2023_LASER_HPP

#define LASER_FIRE_DURATION RCL_MS_TO_NS(250)
#define LASER_FIRE_COOLDOWN RCL_MS_TO_NS(750)
#define LASER_LOOP_DURATION RCL_MS_TO_NS(100)
#define LASER_LOOP_COOLDOWN RCL_MS_TO_NS(500)

#define LASER_NODE_EXECUTOR_HANDLES 7

class LaserNode : Node
{
public:
    LaserNode(rclc_support_t *support, rclc_executor_t *executor, uint32_t laser_pin);

private:
    bool loopState = false;
    bool laserState = false;
    bool cooldownState = false;

    uint32_t laserPin;
    TimerWithContext fireOffTimer;
    TimerWithContext fireCooldownTimer;
    TimerWithContext loopOnTimer;
    TimerWithContext loopOffTimer;
    TimerWithContext loopCooldownTimer;
    rcl_service_t fireService;
    rcl_service_t setLoopService;
    std_srvs__srv__Trigger_Request fireRequest;
    std_srvs__srv__SetBool_Request setLoopRequest;
    std_srvs__srv__Trigger_Response fireResponse;
    std_srvs__srv__SetBool_Response setLoopResponse;

    void fire(__attribute__((unused)) const void *request, void *response);

    bool setLoop(bool state);

    void setLaser(bool state);

    static void setLoopCallback(const std_srvs__srv__SetBool_Request *request_msg,
                                std_srvs__srv__SetBool_Response *response_msg,
                                LaserNode *laser_node);
};


#endif //AVR_PCC_2023_LASER_HPP
