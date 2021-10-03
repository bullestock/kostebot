// Kostebot

#include <stdio.h>
#include <chrono>
#include <random>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/pwm.h"

#define INTERNAL_LED_PIN (gpio_num_t) 2

// PWM period 1000us(1Khz), same as depth
#define PWM_PERIOD    (1000)

#define DEBUG(x)
//#define DEBUG(x) printf x

enum Mode {
    Quiet,
    Sine,
    Square,
    Triangle,
    Noise,
    Last,
};

const char* mode_names[] = {
    "quiet",
    "sine",
    "square",
    "triangle",
    "noise",
    "last",
};

unsigned long millis()
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

static inline uint32_t getCycleCount()
{
    uint32_t ccount;
    __asm__ __volatile__("rsr %0,ccount":"=a" (ccount));
    return ccount;
}

uint32_t pwm_duties[] = {
    500
};

uint32_t pwm_pin_num[] = {
    2
};

float pwm_phases[] = {
    0
};

extern "C"
void app_main()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << INTERNAL_LED_PIN;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    pwm_init(PWM_PERIOD, pwm_duties, 1, pwm_pin_num);
    pwm_set_phases(pwm_phases);
    pwm_start();
    
    int cnt = 0;

    std::default_random_engine generator(getCycleCount());
    std::uniform_int_distribution<int> mode_distribution(0, Last-1);
    auto mode = static_cast<Mode>(mode_distribution(generator));
    if (mode == Quiet)
        mode = Sine;
    std::uniform_int_distribution<int> duration_distribution(10, 60);
    unsigned long duration = 1000 * duration_distribution(generator);
    std::uniform_int_distribution<int> period_distribution(1, 10);
    unsigned long period = 1000 * period_distribution(generator);
    std::uniform_int_distribution<int> bool_distribution(0, 1);

    printf("mode: %s dur %lu period %lu\n", mode_names[mode], duration, period);

    const int granularity = 10;

    auto start_time = millis();
    double power = 0.0;
    while (1)
    {
        // Check if we should change mode
        auto elapsed = (millis() - start_time + granularity - 1)/granularity*granularity;
        if (elapsed >= duration)
        {
            mode = static_cast<Mode>(mode_distribution(generator));
            duration = 1000 * duration_distribution(generator);
            period = 1000 * period_distribution(generator);
            printf("mode: %s dur %lu period %lu\n", mode_names[mode], duration, period);
            start_time = millis();
            elapsed = 0;
        }
        // Do mode-specific stuff
        // Goes from 0 to 1 for each period
        const auto modulus = elapsed % period;
        const auto fraction = static_cast<double>(modulus)/period;
        DEBUG(("elapsed %lu mod %lu fraction (x 100): %d\n", elapsed, modulus, static_cast<int>(fraction*100)));
        switch (mode)
        {
        case Quiet:
            power = 0.0;
            break;
        case Sine:
            {
                auto angle = 2*3.141592 * fraction;
                power = (1.0 + sin(angle))/2.0;
            }
            break;
        case Square:
            power = fraction >= 0.5 ? 1.0 : 0.0;
            break;
        case Triangle:
            if (fraction <= 0.5)
            {
                power = 2*fraction;
                DEBUG(("< power x 100 %d\n", static_cast<int>(power*100)));
            }
            else
            {
                power = 1.0 - 2*(fraction - 0.5);
                DEBUG(("> power x 100 %d\n", static_cast<int>(power*100)));
            }
            break;
        case Noise:
            if (fraction == 0)
                power = bool_distribution(generator);
            break;
        case Last:
            printf("Inconceivable!\n");
            break;
        }
        // Sleep
        DEBUG(("power x 100 %d\n", static_cast<int>(power*100)));
        if (power < 0 || power > 1.0)
        {
            printf("HALT\n");
            while (1)
                vTaskDelay(granularity / portTICK_RATE_MS);
        }
        if (power < 0.01)
            pwm_stop(0);
        else if (power > 0.99)
            pwm_stop(1);
        else
        {
            pwm_set_duty(0, static_cast<int>(power*1000));
            pwm_start();
        }
        vTaskDelay(granularity / portTICK_RATE_MS);
        gpio_set_level(INTERNAL_LED_PIN, cnt % 2);
        ++cnt;
    }
}
