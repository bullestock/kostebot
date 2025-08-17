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

enum Mode {
    Quiet,
    Sine,
    Square,
    Triangle,
    Noise,
    Sawtooth,
    ReverseSawtooth,
    Sos,
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
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = 2,
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
    double vetinari_threshold = 0.5;
    bool vetinari_first = true;
    while (1)
    {
        // Check if we should change mode
        auto elapsed = (millis() - start_time + granularity - 1)/granularity*granularity;
        if (elapsed >= duration)
        {
            mode = static_cast<Mode>(mode_distribution(generator));
            if (last_mode == Quiet && mode == Quiet)
                mode = Sine;
            duration = 1000 * duration_distribution(generator);
            period = 1000 * period_distribution(generator);
            printf("mode: %s dur %lu period %lu\n", mode_names[mode], duration, period);
            start_time = millis();
            elapsed = 0;
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
                if (fraction >= vetinari_threshold)
                {
                    vetinari_first = false;
                    power = 1.0;
                    std::uniform_real_distribution<float> d(0.4, 0.6);
                    vetinari_threshold = d(generator);
                    printf("Next threshold %.2f\n", vetinari_threshold);
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

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, power*255);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        vTaskDelay(granularity / portTICK_PERIOD_MS);
    }
}
