#include "cpu.h"
#include "config.h"

#include <hardware/pwm.h>


void start_cpu_clock(void)
{
    // Calculate divider and wrap for best precision
    uint32_t div = 1;
    uint32_t wrap;

    while (div < 256) {
        wrap = (PICO_CLOCK_SPEED / (div * I8086_CLOCK_SPEED)) - 1;
        if (wrap <= 65535) break;
        div *= 10;  // Try 1, 10, 100, 1000
    }

    const uint32_t level = ((wrap + 1) * CONFIG_I8086_DUTY_CYCLE) / 100;

    gpio_set_function(CLOCK_PIN, GPIO_FUNC_PWM);

    const uint slice_num = pwm_gpio_to_slice_num(CLOCK_PIN);
    const uint channel = pwm_gpio_to_channel(CLOCK_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, div);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, false);
    pwm_set_chan_level(slice_num, channel, level);
    pwm_set_enabled(slice_num, true);
}

void reset_cpu(void)
{
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_OUT);

    // Assert RESET (active LOW) for minimum 4 clock cycles (use 10 for safety)
    gpio_put(RESET_PIN, 1);
    busy_wait_ms(10);
    gpio_put(RESET_PIN, 0);
}


