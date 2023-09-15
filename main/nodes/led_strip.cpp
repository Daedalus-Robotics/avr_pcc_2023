#include "nodes/led_strip.hpp"

#include "system.hpp"

LedStripNode::LedStripNode(NeopixelStrip *strip) : Node("pcc_led_strip", "led_strip"),
                                                   strip(strip),
                                                   setModeService(),
                                                   setModeServiceRequest(), setModeServiceResponse(),
                                                   isUpdating(), shouldUpdate(),
                                                   primaryColor(), secondaryColor(),
                                                   mode(), modeArgument()
{
}

void LedStripNode::setup(rclc_support_t *support, rclc_executor_t *executor)
{
    LOG(LOGLEVEL_INFO, "Setting up LedStripNode");

    Node::setup(support, executor);

    LOG(LOGLEVEL_DEBUG, "Setting up LedStripNode: set mode service");
    HANDLE_ROS_ERROR(rclc_service_init_default(&setModeService,
                                               &node,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(avr_pcc_2023_interfaces, srv, SetLedStrip),
                                               "set"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service_with_context(executor,
                                                            &setModeService,
                                                            &setModeServiceRequest,
                                                            &setModeServiceResponse,
                                                            CONTEXT_SERVICE_CALLBACK(LedStripNode, setModeCallback),
                                                            this), true);
}

void LedStripNode::cleanup()
{
    LOG(LOGLEVEL_DEBUG, "Cleaning up LedStripNode");

    HANDLE_ROS_ERROR(rcl_service_fini(&setModeService, &node), false);

    Node::cleanup();
}

void LedStripNode::updateThread()
{
    isUpdating = true;
    RCLC_UNUSED(isUpdating); // Get rid of unused warning
    LOG(LOGLEVEL_DEBUG, "LED Strip update thread started");
    while (shouldUpdate)
    {
        switch (mode)
        {
            case avr_pcc_2023_interfaces__srv__SetLedStrip_Request__MODE_SOLID:
                strip->fill(&primaryColor);
                shouldUpdate = false;
                break;
            case avr_pcc_2023_interfaces__srv__SetLedStrip_Request__MODE_FLASH:
                if ((state & 0b11) == 0)
                {
                    if (modeArgument > 0)
                    {
                        strip->fill(&primaryColor);
                        modeArgument--;
                    }
                }
                else
                {
                    if (modeArgument == 0)
                    {
                        shouldUpdate = false;
                    }
                    strip->fill(0, 0, 0);
                }
                state++;
                break;
        }
        strip->show();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    isUpdating = false;
    LOG(LOGLEVEL_DEBUG, "LED Strip update thread ended");
    vTaskDelete(nullptr);
}

void LedStripNode::setModeCallback(const void *request, __attribute__((unused)) void *response)
{
    auto request_msg = (avr_pcc_2023_interfaces__srv__SetLedStrip_Request *) request;

    primaryColor.r = request_msg->color.r;
    primaryColor.g = request_msg->color.g;
    primaryColor.b = request_msg->color.b;

    secondaryColor.r = request_msg->secondary_color.r;
    secondaryColor.g = request_msg->secondary_color.g;
    secondaryColor.b = request_msg->secondary_color.b;

    mode = request_msg->mode;
    modeArgument = request_msg->argument;

    shouldUpdate = true;
    state = 0;

    if (!isUpdating)
    {
        xTaskCreate(CONTEXT_TASK_CALLBACK(LedStripNode, updateThread),
                    "led_strip_update",
                    configMINIMAL_STACK_SIZE,
                    this,
                    4,
                    nullptr);
    }
}
