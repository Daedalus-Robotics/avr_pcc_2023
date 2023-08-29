#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>
#include "context_timer.hpp"
#include "utils.hpp"
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

    bool fire();

    bool setLoop(bool state);

private:
    bool loopState = false;
    bool laserState = false;

    uint32_t laserPin;
    rcl_service_t fireService;
    rcl_service_t setLoopService;
    std_srvs__srv__Trigger_Request fireRequest;
    std_srvs__srv__SetBool_Request setLoopRequest;
    std_srvs__srv__Trigger_Response fireResponse;
    std_srvs__srv__SetBool_Response setLoopResponse;

    void setLaser(bool state);

    static void fireCallback(__attribute__((unused)) const std_srvs__srv__Trigger_Request *request_msg,
                             std_srvs__srv__Trigger_Response *response_msg,
                             LaserNode laser_node);

    static void setLoopCallback(const std_srvs__srv__SetBool_Request *request_msg,
                                std_srvs__srv__SetBool_Response *response_msg,
                                LaserNode laser_node);
};


#endif //AVR_PCC_2023_LASER_HPP
