#include <cstdint>
#include <driver/gpio.h>
#include <driver/rmt.h>

#ifndef AVR_PCC_2023_NEOPIXEL_STRIP_HPP
#define AVR_PCC_2023_NEOPIXEL_STRIP_HPP

#define NEOPIXEL_TYPE_WS2812 NeopixelType(400, 1000, 1000, 400, 24)

struct NeopixelType
{
    const uint32_t bit0HighTime;
    const uint32_t bit0LowTime;
    const uint32_t bit1HighTime;
    const uint32_t bit1LowTime;
    const uint8_t bitsPerCmd;
};

struct RgbColor
{
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
};

class NeopixelStrip
{
public:
    NeopixelStrip(gpio_num_t pin,
                  NeopixelType type,
                  size_t strip_length,
                  rmt_channel_t rmt_channel = RMT_CHANNEL_0);

    void show() const;

    void setPixel(size_t led_num, uint8_t red, uint8_t green, uint8_t blue);

    inline void setPixel(size_t led_num, RgbColor color)
    {
        setPixel(led_num, color.red, color.green, color.blue);
    }

    void fill(uint8_t red, uint8_t green, uint8_t blue);

    inline void fill(RgbColor color)
    {
        fill(color.red, color.green, color.blue);
    }

private:
    const uint8_t bitsPerCmd;
    const size_t length;
    const rmt_channel_t rmtChannel;
    rmt_item32_t rmtLookupTable[2];

    rmt_item32_t *buffer;

    void setBufferForLed(size_t led_num, uint32_t data);
};


#endif //AVR_PCC_2023_NEOPIXEL_STRIP_HPP
