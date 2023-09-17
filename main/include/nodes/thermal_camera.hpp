#include <gpio_cxx.hpp>
#include <i2c_cxx.hpp>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/temperature.h>
#include <avr_pcc_2023_interfaces/msg/thermal_frame.h>

#include "node.hpp"
#include "context_timer.hpp"

#define THERMAL_CAMERA_NODE_EXECUTOR_HANDLES 1
#define AMG88XX_PIXEL_ARRAY_SIZE 64

#ifndef AVR_PCC_2023_THERMAL_CAMERA_HPP
#define AVR_PCC_2023_THERMAL_CAMERA_HPP


class ThermalCameraNode : Node
{
public:
    ThermalCameraNode(idf::GPIONumBase<idf::SDA_type> sda, idf::GPIONumBase<idf::SCL_type> scl, idf::I2CNumber port = idf::I2CNumber::I2C0());

    void setup(rclc_support_t *support, rclc_executor_t *executor) override;

    void cleanup() override;

private:
    std::shared_ptr<idf::I2CMaster> master;

    TimerWithContext updateTimer;
    rcl_publisher_t refPublisher;
    sensor_msgs__msg__Temperature refMessage;
    rcl_publisher_t rawPublisher;
    avr_pcc_2023_interfaces__msg__ThermalFrame rawMessage;
    rcl_publisher_t interpolatedPublisher;
    avr_pcc_2023_interfaces__msg__ThermalFrame interpolatedMessage;

    bool updateThermistor;
    std::vector<uint8_t> thermistorBuffer;
    std::vector<uint8_t> pixelBuffer;
    float pixels[AMG88XX_PIXEL_ARRAY_SIZE];
    int32_t time;
    uint32_t timeNs;

    void updateTimerCallback(rcl_timer_t *timer, int64_t n);

//    void interpolateFrame(float *data);

    inline static uint16_t uInt8ToUInt16(uint8_t v0, uint8_t v1)
    {
        return ((uint16_t) v1 << 8) | (uint16_t) v0;
    }

    static float signedMag12ToFloat(uint16_t val);

    static float int12ToFloat(uint16_t val);
};


#endif //AVR_PCC_2023_THERMAL_CAMERA_HPP
