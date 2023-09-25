#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <avr_pcc_2023_interfaces/srv/set_led_strip.h>
#include <atomic>

#include "node.hpp"
#include "neopixel_strip.hpp"

#define LED_STRIP_NODE_EXECUTOR_HANDLES 1

#ifndef AVR_PCC_2023_LED_STRIP_HPP
#define AVR_PCC_2023_LED_STRIP_HPP


class LedStripNode : Node
{
public:
    explicit LedStripNode(NeopixelStrip *strip);

    void setup(rclc_support_t *support, rclc_executor_t *executor) override;

    void cleanup(rclc_executor_t *executor) override;

private:
    NeopixelStrip *strip;

    rcl_service_t setModeService;
    avr_pcc_2023_interfaces__srv__SetLedStrip_Request setModeServiceRequest;
    avr_pcc_2023_interfaces__srv__SetLedStrip_Response setModeServiceResponse;

    std::atomic<bool> isUpdating;
    std::atomic<bool> shouldUpdate;
    std::atomic<uint8_t> state;
    AtomicRgbColor primaryColor;
    AtomicRgbColor secondaryColor;
    std::atomic<uint8_t> mode;
    std::atomic<uint8_t> modeArgument;

    void updateThread();

    void setModeCallback(const void *request, void *response);
};


#endif //AVR_PCC_2023_LED_STRIP_HPP
