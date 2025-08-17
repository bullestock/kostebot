// Kostebot

#include <stdio.h>
#include <chrono>
#include <random>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <driver/ledc.h>

#define DEBUG(x)
//#define DEBUG(x) printf x

constexpr const gpio_num_t PIN_MOTOR = (gpio_num_t) 2;
constexpr const gpio_num_t PIN_B1 = (gpio_num_t) 27;
constexpr const gpio_num_t PIN_B2 = (gpio_num_t) 25;
constexpr const gpio_num_t PIN_B3 = (gpio_num_t) 32;

enum Mode {
    // Off
    Quiet,
    // Sine wave
    Sine,
    // Square wave, random duty cycle
    Square,
    // Triangle
    Triangle,
    // Randomly switches between 0 and 1
    Noise,
    // Ramps from 0 to 1, then repeats
    Sawtooth,
    // Ramps from 1 to 0, then repeats
    ReverseSawtooth,
    // - - - . . . - - -
    Sos,
    // Like Square, but the transition is a little bit too early or too late
    VetinarisClock,
    Last,
};

const char* mode_names[] = {
    "quiet",
    "sine",
    "square",
    "triangle",
    "noise",
    "sawtooth",
    "rsawtooth",
    "sos",
    "vetinari",
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

extern "C"
void app_main()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask =
        (1ULL << PIN_B1) |
        (1ULL << PIN_B2) |
        (1ULL << PIN_B3);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = PIN_MOTOR,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&pwm_channel);

    std::default_random_engine generator(getCycleCount());
    std::uniform_int_distribution<int> mode_distribution(0, Last-1);
    auto mode = static_cast<Mode>(mode_distribution(generator));
    if (mode == Quiet)
        mode = Sine;

    std::uniform_int_distribution<int> duration_distribution(10, 60);
    unsigned long duration = 1000 * duration_distribution(generator);
        
    std::uniform_int_distribution<int> period_distribution(1, 10);
    unsigned long period = 1000 * period_distribution(generator);
    if (mode == VetinarisClock)
        period *= 3;
    
    std::uniform_int_distribution<int> bool_distribution(0, 1);

    printf("mode: %s dur %lu period %lu\n", mode_names[mode], duration, period);

    const int granularity = 10;

    auto start_time = millis();
    double power = 0.0;
    auto last_mode = mode;
    double threshold = 0.5;
    bool vetinari_first = true;
    double max_power = 1.0;
    auto last_key = start_time;
    while (1)
    {
        bool next = false;
        bool up = false;
        bool down = false;

        if (millis() - last_key > 500)
        {
            next = !gpio_get_level(PIN_B1);
            up = !gpio_get_level(PIN_B2);
            down = !gpio_get_level(PIN_B3);
            //printf("next %d up %d down %d\n", next, up, down);
            last_key = millis();
        }

        // Check if we should change mode
        auto elapsed = (millis() - start_time + granularity - 1)/granularity*granularity;
        if (next || elapsed >= duration)
        {
            while (mode == last_mode)
                mode = static_cast<Mode>(mode_distribution(generator));
        }

        if (up)
        {
            max_power += 0.1;
            if (max_power > 1.0)
                max_power = 1.0;
            printf("Power %.1f\n", max_power);
        }
        else if (down)
        {
            max_power -= 0.1;
            if (max_power < 0.1)
                max_power = 0.0;
            printf("Power %.1f\n", max_power);
        }

        if (mode != last_mode)
        {
            std::uniform_real_distribution<float> d(0.1, 0.9);
            threshold = d(generator);
            duration = 1000 * duration_distribution(generator);
            period = 1000 * period_distribution(generator);
            printf("mode: %s dur %lu period %lu threshold %.1f\n",
                   mode_names[mode], duration, period, threshold);
            start_time = millis();
            elapsed = 0;
            last_mode = mode;
        }
        // Do mode-specific stuff
        const auto modulus = elapsed % period;
        // Goes from 0 to 1 for each period
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
            power = fraction >= threshold ? 1.0 : 0.0;
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
        case Sawtooth:
            power = fraction;
            break;
        case ReverseSawtooth:
            power = 1.0 - fraction;
            break;
        case Sos:
            power = 0.0;
            if ((fraction < 1.0/32.0) ||
                ((fraction >= 2.0/32.0) && (fraction < 3.0/32.0)) || 
                ((fraction >= 4.0/32.0) && (fraction < 5.0/32.0)) || 
                ((fraction >= 8.0/32.0) && (fraction < 13.0/32.0)) || 
                ((fraction >= 16.0/32.0) && (fraction < 21.0/32.0)) || 
                ((fraction >= 24.0/32.0) && (fraction < 29.0/32.0)))
                power = 1.0;
            break;
        case VetinarisClock:
            if (vetinari_first)
            {
                power = 0.0;
                if (fraction >= threshold)
                {
                    vetinari_first = false;
                    power = 1.0;
                    std::uniform_real_distribution<float> d(0.4, 0.6);
                    threshold = d(generator);
                    printf("Next threshold %.2f\n", threshold);
                }
            }
            else if (fraction == 0.0)
                vetinari_first = true;
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
                vTaskDelay(granularity / portTICK_PERIOD_MS);
        }

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, power*max_power*255);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        vTaskDelay(granularity / portTICK_PERIOD_MS);
    }
}
