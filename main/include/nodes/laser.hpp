#include "driver/gpio.h"

#include <atomic>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>

#include "node.hpp"
#include "context_timer.hpp"
#include "neopixel_strip.hpp" // ForTesting

#ifndef AVR_PCC_2023_LASER_HPP
#define AVR_PCC_2023_LASER_HPP

#define LASER_NODE_EXECUTOR_HANDLES 2

class LaserNode : Node
{
public:
    // ForTesting
    explicit LaserNode(gpio_num_t laser_pin, NeopixelStrip *n);

    void setup(rclc_support_t *support, rclc_executor_t *executor) override;

    void cleanup() override;

private:
    const gpio_num_t laserPin;
    NeopixelStrip *neo; // ForTesting

    rcl_service_t fireService;
    rcl_service_t setLoopService;
    std_srvs__srv__Trigger_Request fireRequest;
    std_srvs__srv__SetBool_Request setLoopRequest;
    std_srvs__srv__Trigger_Response fireResponse;
    std_srvs__srv__SetBool_Response setLoopResponse;

    std::atomic<bool> loopState = false;
    std::atomic<bool> laserState = false;
    std::atomic<bool> cooldownState = false;

    void setLaser(bool state);

    void tryStartLoop();

    void fireThread();

    void loopThread();

    void fireCallback(const void *request, void *response);

    void setLoopCallback(const void *request, void *response);

//    static void setLoopCallback(const std_srvs__srv__SetBool_Request *request_msg,
//                                std_srvs__srv__SetBool_Response *response_msg,
//                                LaserNode *laser_node);
};


#endif //AVR_PCC_2023_LASER_HPP
