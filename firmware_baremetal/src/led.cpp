/*
 * LED blinker state machine.
 * Platform-independent - uses only hal_pwm, hal_gpio, hal_timer.
 */

#include "led.h"

#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"
#include "hal/hal_timer.h"
#include "baremetal_config.h"
#include "state.h"

#include "constants.hpp"

/* LED PWM channel indices: CH0=Green(PA0), CH1=Blue(PA1), CH2=Red(PA2) */
#define LED_CH_GREEN  0
#define LED_CH_BLUE   1
#define LED_CH_RED    2

/* Triangle wave state: counts 0→509, maps to intensity abs(t - 255) */
static int triangle_counter;
static uint32_t last_step_ms;
static uint32_t last_comms_activity_ms;
static bool comms_activity_active;

void led_init(void) {
    triangle_counter = 0;
    last_step_ms = hal_timer_msec();
    last_comms_activity_ms = 0;
    comms_activity_active = false;

    /* Start with LED off */
    hal_pwm_led_set(LED_CH_GREEN, 0);
    hal_pwm_led_set(LED_CH_BLUE, 0);
    hal_pwm_led_set(LED_CH_RED, 0);

    /* Comms activity LED off (active low: set = off) */
    hal_gpio_set(LED_Y_PORT, LED_Y_PIN);
}

void led_set_comms_activity(void) {
    last_comms_activity_ms = hal_timer_msec();
    comms_activity_active = true;
}

bool led_step(void) {
    uint32_t now = hal_timer_msec();
    if ((now - last_step_ms) < LED_STEP_MS) {
        return false;
    }
    last_step_ms = now;

    /* Compute green triangle wave intensity */
    int intensity = triangle_counter <= 255 ?
                    triangle_counter : 510 - triangle_counter;
    uint8_t g = (uint8_t)intensity;
    uint8_t r = 0;
    uint8_t b = 0;

    /* Read gate driver fault pins (active low) */
    bool fault = !hal_gpio_read(MDRV_NFAULT_PORT, MDRV_NFAULT_PIN);
    bool octw = !hal_gpio_read(MDRV_NOCTW_PORT, MDRV_NOCTW_PIN);

    /* Fault override: flash red when green is dim */
    if (fault) {
        if (g < 50) {
            r = 255;
            g = 0;
        }
        state_parameters.gate_fault = true;
    }

    /* OCTW override: flash blue when green is bright */
    if (octw) {
        if (g > 200) {
            b = 255;
            g = 0;
        }
        state_parameters.gate_fault = true;
    }

    if (!fault && !octw) {
        state_parameters.gate_fault = false;
    }

    /* Apply gamma correction and set PWM */
    hal_pwm_led_set(LED_CH_RED,   motor_driver::consts::led_gamma_table[r]);
    hal_pwm_led_set(LED_CH_GREEN, motor_driver::consts::led_gamma_table[g]);
    hal_pwm_led_set(LED_CH_BLUE,  motor_driver::consts::led_gamma_table[b]);

    /* Advance triangle wave: 0 → 509 → 0 ... */
    triangle_counter = (triangle_counter + 10) % 510;

    /* Comms activity LED (active low): on for 25ms after last activity */
    if (comms_activity_active) {
        uint32_t elapsed = now - last_comms_activity_ms;
        if (elapsed < motor_driver::consts::comms_activity_led_duration) {
            hal_gpio_clear(LED_Y_PORT, LED_Y_PIN);  /* LED on (active low) */
        } else {
            hal_gpio_set(LED_Y_PORT, LED_Y_PIN);    /* LED off */
            comms_activity_active = false;
        }
    }

    return true;
}
