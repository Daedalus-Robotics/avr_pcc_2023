#include <cstdint>
#include <color.h>
#include <driver/gpio.h>
#include <driver/rmt.h>
#include <atomic>

#define A_RGB(a_color) {.r = a_color.r, .g = a_color.g, .b = a_color.b}

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

struct AtomicRgbColor
{
    std::atomic<uint8_t> r;
    std::atomic<uint8_t> g;
    std::atomic<uint8_t> b;
};

class NeopixelStrip
{
public:
    NeopixelStrip(gpio_num_t pin,
                  NeopixelType type,
                  size_t strip_length,
                  rmt_channel_t rmt_channel = RMT_CHANNEL_0);

    void show() const;

    void setPixel(size_t led_num, rgb_t color);

    inline void setPixel(size_t led_num, uint8_t red, uint8_t green, uint8_t blue)
    {
        setPixel(led_num, {.r = red, .g = green, .b = blue});
    }

    inline void setPixel(size_t led_num, AtomicRgbColor *atomic_color)
    {
        setPixel(led_num, atomic_color->r, atomic_color->g, atomic_color->b);
    }

    void fill(rgb_t color);

    inline void fill(uint8_t red, uint8_t green, uint8_t blue)
    {
        fill({.r = red, .g = green, .b = blue});
    }

    inline void fill(AtomicRgbColor *atomic_color)
    {
        fill(atomic_color->r, atomic_color->g, atomic_color->b);
    }

    [[nodiscard]] size_t getLength() const;

private:
    const uint8_t bitsPerCmd;
    const size_t length;
    const rmt_channel_t rmtChannel;
    rmt_item32_t rmtLookupTable[2];

    rmt_item32_t *buffer;

    void setBufferForLed(size_t led_num, uint32_t data);
};


#endif //AVR_PCC_2023_NEOPIXEL_STRIP_HPP
