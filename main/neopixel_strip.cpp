#include "neopixel_strip.hpp"

#include <cstring>
#include <esp_err.h>

#include "system.hpp"

#define CLOCK_RATIO(time) (uint32_t) (((float) APB_CLK_FREQ / 2 / 1e09f) * (time))

NeopixelStrip::NeopixelStrip(const gpio_num_t pin,
                             const NeopixelType type,
                             const size_t strip_length,
                             const rmt_channel_t rmt_channel) : bitsPerCmd(type.bitsPerCmd),
                                                                length(strip_length),
                                                                rmtChannel(rmt_channel),
                                                                rmtLookupTable()
{
    rmtLookupTable[0] = (rmt_item32_t) {{{CLOCK_RATIO(type.bit0HighTime), 1, CLOCK_RATIO(type.bit0LowTime), 0}}};
    rmtLookupTable[1] = (rmt_item32_t) {{{CLOCK_RATIO(type.bit1HighTime), 1, CLOCK_RATIO(type.bit1LowTime), 0}}};
    buffer = new rmt_item32_t[length * bitsPerCmd];

    const rmt_config_t neopixel_rmt_config = {
            .rmt_mode = RMT_MODE_TX,
            .channel = rmt_channel,
            .gpio_num = pin,
            .clk_div = 2,
            .mem_block_num = 3,
            .tx_config = {
                    .idle_level = RMT_IDLE_LEVEL_LOW,
                    .carrier_en = false,
                    .loop_en = false,
                    .idle_output_en = true
            }
    };

    HANDLE_ESP_ERROR(rmt_config(&neopixel_rmt_config), true);
    HANDLE_ESP_ERROR(rmt_driver_install(neopixel_rmt_config.channel, 0, 0), true);
}

void NeopixelStrip::show() const
{
    HANDLE_ESP_ERROR(rmt_write_items(RMT_CHANNEL_0, buffer, (int) (length * bitsPerCmd), false), true);
    HANDLE_ESP_ERROR(rmt_wait_tx_done(rmtChannel, portMAX_DELAY), true);
}

void NeopixelStrip::setPixel(const size_t led_num,
                             const uint8_t red,
                             const uint8_t green,
                             const uint8_t blue)
{
    setBufferForLed(led_num, (green << 16) | (red << 8) | blue);
}

void NeopixelStrip::fill(const uint8_t red,
                         const uint8_t green,
                         const uint8_t blue)
{
    setPixel(0, red, green, blue);

    const size_t led_segment_size = sizeof(rmt_item32_t) * bitsPerCmd;
    for (size_t led = 0; led < length; led++)
    {
        memcpy(&buffer[led * bitsPerCmd], &buffer[0], led_segment_size);
    }
}

void NeopixelStrip::setBufferForLed(const size_t led_num, const uint32_t data)
{
    const size_t start_index = led_num * bitsPerCmd;
    uint32_t mask = 1 << (bitsPerCmd - 1);
    for (uint32_t bit = 0; bit < bitsPerCmd; bit++)
    {
        uint32_t bit_is_set = data & mask;
        buffer[start_index + bit] = rmtLookupTable[bit_is_set != 0];
        mask >>= 1;
    }
}
