/*
 * Unit tests for LED state machine.
 */

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "hal_mock.h"
#include "led.h"
#include "state.h"
#include "baremetal_config.h"

#include "constants.hpp"

#define LED_CH_GREEN  0
#define LED_CH_BLUE   1
#define LED_CH_RED    2

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { \
    mock_reset_all(); \
    state_init(); \
    /* Set fault pins high (no fault, active low) */ \
    mock_gpio_set_input(MDRV_NFAULT_PORT, MDRV_NFAULT_PIN, true); \
    mock_gpio_set_input(MDRV_NOCTW_PORT, MDRV_NOCTW_PIN, true); \
    led_init(); \
    printf("  %-50s", #name); \
    name(); \
    printf(" PASS\n"); \
    tests_passed++; \
} while (0)

#define ASSERT(cond) do { \
    if (!(cond)) { \
        printf(" FAIL\n    %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        tests_failed++; \
        return; \
    } \
} while (0)

#define ASSERT_EQ(a, b) do { \
    auto _a = (a); auto _b = (b); \
    if (_a != _b) { \
        printf(" FAIL\n    %s:%d: %s == %d, expected %d\n", \
               __FILE__, __LINE__, #a, (int)_a, (int)_b); \
        tests_failed++; \
        return; \
    } \
} while (0)

/* ── Tests ──────────────────────────────────────────────── */

TEST(test_init_leds_off) {
    /* After init, all LED PWMs should be 0 (gamma[0] = 0) */
    ASSERT_EQ(mock_pwm_get_led(LED_CH_GREEN), 0);
    ASSERT_EQ(mock_pwm_get_led(LED_CH_BLUE), 0);
    ASSERT_EQ(mock_pwm_get_led(LED_CH_RED), 0);
}

TEST(test_green_pulse_starts_at_zero) {
    /* First step at t=10ms: triangle_counter starts at 0, steps by 10 */
    mock_timer_advance_ms(LED_STEP_MS);
    bool updated = led_step();
    ASSERT(updated);

    /* After first step: counter was 0, intensity = abs(0-255) = 0 for
     * counter=0 → intensity=0; then counter advances to 10.
     * Actually: intensity = counter (0..255) or 510-counter (256..509)
     * counter starts at 0: intensity = 0 → green = gamma[0] = 0 */
    ASSERT_EQ(mock_pwm_get_led(LED_CH_GREEN), motor_driver::consts::led_gamma_table[0]);
    ASSERT_EQ(mock_pwm_get_led(LED_CH_RED), 0);
    ASSERT_EQ(mock_pwm_get_led(LED_CH_BLUE), 0);
}

TEST(test_green_pulse_ramps_up) {
    /* Step 10 times (100ms): counter goes 0,10,20,...,90
     * On step 10, counter=90 before advance, intensity=90 */
    for (int i = 0; i < 10; i++) {
        mock_timer_advance_ms(LED_STEP_MS);
        led_step();
    }
    /* After 10 steps, last computed intensity was at counter=90 */
    uint16_t expected = motor_driver::consts::led_gamma_table[90];
    ASSERT_EQ(mock_pwm_get_led(LED_CH_GREEN), expected);
}

TEST(test_green_pulse_peak_at_255) {
    /* Step to counter=250, intensity=250 */
    /* Need 26 steps to reach counter=250 (0,10,20,...,250) */
    for (int i = 0; i < 26; i++) {
        mock_timer_advance_ms(LED_STEP_MS);
        led_step();
    }
    /* counter=250, intensity=250 */
    uint16_t expected = motor_driver::consts::led_gamma_table[250];
    ASSERT_EQ(mock_pwm_get_led(LED_CH_GREEN), expected);
}

TEST(test_green_pulse_ramps_down) {
    /* Go past 255: step 30 times, counter=290, intensity=510-290=220 */
    for (int i = 0; i < 30; i++) {
        mock_timer_advance_ms(LED_STEP_MS);
        led_step();
    }
    /* counter=290, intensity=510-290=220 */
    uint16_t expected = motor_driver::consts::led_gamma_table[220];
    ASSERT_EQ(mock_pwm_get_led(LED_CH_GREEN), expected);
}

TEST(test_no_update_before_interval) {
    /* Step without enough time elapsed should return false */
    mock_timer_advance_ms(LED_STEP_MS - 1);
    bool updated = led_step();
    ASSERT(!updated);
}

TEST(test_fault_override_red) {
    /* Set NFAULT low (fault active) */
    mock_gpio_set_input(MDRV_NFAULT_PORT, MDRV_NFAULT_PIN, false);

    /* Advance to where green < 50 (counter=40, intensity=40) */
    for (int i = 0; i < 5; i++) {
        mock_timer_advance_ms(LED_STEP_MS);
        led_step();
    }
    /* counter=40, intensity=40, which is < 50, so red override */
    ASSERT_EQ(mock_pwm_get_led(LED_CH_RED),
              motor_driver::consts::led_gamma_table[255]);
    ASSERT_EQ(mock_pwm_get_led(LED_CH_GREEN),
              motor_driver::consts::led_gamma_table[0]);
    ASSERT(state_parameters.gate_fault);
}

TEST(test_octw_override_blue) {
    /* Set NOCTW low (over-current warning) */
    mock_gpio_set_input(MDRV_NOCTW_PORT, MDRV_NOCTW_PIN, false);

    /* Advance to where green > 200 (counter=210, intensity=210) */
    for (int i = 0; i < 22; i++) {
        mock_timer_advance_ms(LED_STEP_MS);
        led_step();
    }
    /* counter=210, intensity=210, which is > 200, so blue override */
    ASSERT_EQ(mock_pwm_get_led(LED_CH_BLUE),
              motor_driver::consts::led_gamma_table[255]);
    ASSERT_EQ(mock_pwm_get_led(LED_CH_GREEN),
              motor_driver::consts::led_gamma_table[0]);
    ASSERT(state_parameters.gate_fault);
}

TEST(test_no_fault_clears_gate_fault) {
    /* Both pins high = no fault */
    mock_timer_advance_ms(LED_STEP_MS);
    led_step();
    ASSERT(!state_parameters.gate_fault);
}

TEST(test_comms_activity_led_on) {
    /* Trigger comms activity */
    led_set_comms_activity();

    /* Step once to process */
    mock_timer_advance_ms(LED_STEP_MS);
    led_step();

    /* LED_Y should be on (active low: clear = on) */
    /* Port B, Pin 8 */
    bool led_y_output = mock_gpio_get_output(LED_Y_PORT, LED_Y_PIN);
    /* Active low: output false = LED on */
    ASSERT(!led_y_output);
}

TEST(test_comms_activity_led_off_after_timeout) {
    /* Trigger comms activity */
    led_set_comms_activity();

    /* Step to process it */
    mock_timer_advance_ms(LED_STEP_MS);
    led_step();

    /* Advance past the 25ms timeout */
    mock_timer_advance_ms(30);
    led_step();

    /* LED_Y should be off (active low: set = off) */
    bool led_y_output = mock_gpio_get_output(LED_Y_PORT, LED_Y_PIN);
    ASSERT(led_y_output);
}

TEST(test_combined_faults) {
    /* Both NFAULT and NOCTW low */
    mock_gpio_set_input(MDRV_NFAULT_PORT, MDRV_NFAULT_PIN, false);
    mock_gpio_set_input(MDRV_NOCTW_PORT, MDRV_NOCTW_PIN, false);

    /* At counter=40 (intensity < 50): fault takes red */
    for (int i = 0; i < 5; i++) {
        mock_timer_advance_ms(LED_STEP_MS);
        led_step();
    }
    ASSERT_EQ(mock_pwm_get_led(LED_CH_RED),
              motor_driver::consts::led_gamma_table[255]);
    ASSERT(state_parameters.gate_fault);
}

/* ── Main ───────────────────────────────────────────────── */

int main(void) {
    printf("LED tests:\n");

    RUN_TEST(test_init_leds_off);
    RUN_TEST(test_green_pulse_starts_at_zero);
    RUN_TEST(test_green_pulse_ramps_up);
    RUN_TEST(test_green_pulse_peak_at_255);
    RUN_TEST(test_green_pulse_ramps_down);
    RUN_TEST(test_no_update_before_interval);
    RUN_TEST(test_fault_override_red);
    RUN_TEST(test_octw_override_blue);
    RUN_TEST(test_no_fault_clears_gate_fault);
    RUN_TEST(test_comms_activity_led_on);
    RUN_TEST(test_comms_activity_led_off_after_timeout);
    RUN_TEST(test_combined_faults);

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
