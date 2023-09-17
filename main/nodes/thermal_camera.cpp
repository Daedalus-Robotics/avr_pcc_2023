#include "nodes/thermal_camera.hpp"

#include <cmath>
#include <esp_log.h>

#include "system.hpp"

#define AMG88XX_ADDR idf::I2CAddress(0x69)
#define AMG88XX_THERMISTOR_CONVERSION .0625
#define AMG88XX_PIXEL_TEMP_CONVERSION .25

enum [[maybe_unused]] Amg88XxRegisters
{
    AMG88XX_REG_POWER_MODE = 0x00,
    AMG88XX_REG_RESET = 0x01,
    AMG88XX_REG_FRAMERATE = 0x02,
    AMG_88_XX_INTC = 0x03,
    AMG_88_XX_STAT = 0x04,
    AMG_88_XX_SCLR = 0x05,
    AMG88XX_REG_MOVING_AVERAGE = 0x07,
    AMG_88_XX_INTHL = 0x08,
    AMG_88_XX_INTHH = 0x09,
    AMG_88_XX_INTLL = 0x0A,
    AMG_88_XX_INTLH = 0x0B,
    AMG_88_XX_IHYSL = 0x0C,
    AMG_88_XX_IHYSH = 0x0D,
    AMG88XX_REG_THERMISTOR = 0x0E,
    AMG_88_XX_TTHH = 0x0F,
    AMG_88_XX_INT_OFFSET = 0x010,
    AMG88XX_REG_PIXEL_OFFSET = 0x80
};

enum [[maybe_unused]] Amg88XxPowerModes
{
    AMG88XX_MODE_NORMAL = 0x00,
    AMG88XX_MODE_SLEEP = 0x01,
    AMG88XX_MODE_STAND_BY_60 = 0x20,
    AMG88XX_MODE_STAND_BY_10 = 0x21
};

enum [[maybe_unused]] Amg88XxFrameRates
{
    AMG88XX_FPS_10 = 0x00,
    AMG88XX_FPS_1 = 0x01
};

ThermalCameraNode::ThermalCameraNode(idf::GPIONumBase<idf::SDA_type> sda,
                                     idf::GPIONumBase<idf::SCL_type> scl,
                                     idf::I2CNumber port) : Node("pcc_thermal_camera", "thermal"),
                                                            master(new idf::I2CMaster(port, scl, sda,
                                                                                      idf::Frequency(100000))),
                                                            updateTimer(),
                                                            refPublisher(), refMessage(),
                                                            rawPublisher(), rawMessage(),
                                                            interpolatedPublisher(), interpolatedMessage(),
                                                            updateThermistor(),
                                                            thermistorBuffer(), pixelBuffer(),
                                                            pixels(),
                                                            time()
{
    try
    {
        master->sync_write(AMG88XX_ADDR, {AMG88XX_REG_POWER_MODE, AMG88XX_MODE_NORMAL}); // Set mode to normal
        master->sync_write(AMG88XX_ADDR, {AMG88XX_REG_RESET, 0x3F}); // Reset the camera
        master->sync_write(AMG88XX_ADDR, {AMG88XX_REG_FRAMERATE, AMG88XX_FPS_10}); // Set fps to 10
    }
    catch (idf::I2CException &)
    {
        // ToDo: Add diagnostics
        ESP_LOGI("thermal_camera", "Can't connect to the thermal camera");
    }

    updateTimer.context = this;

    refMessage.header.frame_id.data = const_cast<char *>("thermal_camera");
    refMessage.header.frame_id.size = 14;
    refMessage.variance = 0;

    rawMessage.length = 8;
    rawMessage.width = 8;
    rawMessage.step = 8 << 1;
    rawMessage.data.size = AMG88XX_PIXEL_ARRAY_SIZE;
}

void ThermalCameraNode::setup(rclc_support_t *support, rclc_executor_t *executor)
{
    LOG(LOGLEVEL_INFO, "Setting up ThermalCameraNode");

    Node::setup(support, executor);

    HANDLE_ROS_ERROR(rclc_timer_init_default(&updateTimer.timer,
                                             support,
                                             RCL_MS_TO_NS(100),
                                             CONTEXT_TIMER_CALLBACK(ThermalCameraNode, updateTimerCallback)), true);
    HANDLE_ROS_ERROR(rclc_executor_add_timer(executor, &updateTimer.timer), true);

    HANDLE_ROS_ERROR(rclc_publisher_init_default(&refPublisher,
                                                 &node,
                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
                                                 "ref"), true);
    HANDLE_ROS_ERROR(rclc_publisher_init_default(&rawPublisher,
                                                 &node,
                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(avr_pcc_2023_interfaces,
                                                                                          msg,
                                                                                          ThermalFrame),
                                                 "raw"), true);
}

void ThermalCameraNode::cleanup()
{
    LOG(LOGLEVEL_DEBUG, "Cleaning up ThermalCameraNode");

    HANDLE_ROS_ERROR(rcl_publisher_fini(&rawPublisher, &node), false);
    HANDLE_ROS_ERROR(rcl_publisher_fini(&refPublisher, &node), false);

    Node::cleanup();
}

void ThermalCameraNode::updateTimerCallback(__attribute__((unused)) rcl_timer_t *timer,
                                            __attribute__((unused)) int64_t n)
{
    updateThermistor = !updateThermistor;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    try
    {
        if (updateThermistor)
        {
            thermistorBuffer = master->sync_transfer(AMG88XX_ADDR,
                                                     {AMG88XX_REG_THERMISTOR},
                                                     2);
        }
        pixelBuffer = master->sync_transfer(AMG88XX_ADDR,
                                            {AMG88XX_REG_PIXEL_OFFSET},
                                            AMG88XX_PIXEL_ARRAY_SIZE << 1);
    }
    catch (idf::I2CException &)
    {
        // ToDo: Add diagnostics
        ESP_LOGI("thermal_camera", "Can't connect to the thermal camera at runtime");
        return;
    }

    time = (int32_t) (esp_timer_get_time() * 1000000);

    if (updateThermistor)
    {
        uint16_t recast = uInt8ToUInt16(thermistorBuffer[0], thermistorBuffer[1]);
        float thermistor_temp = signedMag12ToFloat(recast) * AMG88XX_THERMISTOR_CONVERSION;

        refMessage.header.stamp.sec = time;
        refMessage.temperature = thermistor_temp;

        HANDLE_ROS_ERROR(rcl_publish(&refPublisher, &refMessage, nullptr), false);
    }

    uint8_t pos;
    uint16_t recast_pixel;
    for (int i = 0; i < AMG88XX_PIXEL_ARRAY_SIZE; i++) {
        pos = i << 1;
        recast_pixel = uInt8ToUInt16(pixelBuffer[pos], pixelBuffer[pos + 1]);

        pixels[i] = int12ToFloat(recast_pixel) * AMG88XX_PIXEL_TEMP_CONVERSION;
    }

    rawMessage.header.stamp.sec = time;
    rawMessage.data.data = pixels;

    HANDLE_ROS_ERROR(rcl_publish(&rawPublisher, &rawMessage, nullptr), false);
}

float ThermalCameraNode::signedMag12ToFloat(uint16_t val)
{
    // take the first 11 bits as absolute val
    uint16_t abs_val = (val & 0x7FF);

    return (val & 0x800) ? 0 - (float)abs_val : (float)abs_val;
}

float ThermalCameraNode::int12ToFloat(uint16_t val)
{
    auto s_val = (int16_t)(val << 4); // shift to left so that the sign bit of the 12-bit integer number is
                                                  // placed on the sign bit of the 16-bit signed integer number
    return (float)(s_val >> 4); // shift back the signed number, return converts to float
}
