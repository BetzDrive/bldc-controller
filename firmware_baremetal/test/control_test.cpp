/*
 * Unit tests for the FOC control loop.
 * Tests Clarke/Park transforms, SVM, PID, state estimation, and control modes.
 * Links against hal_mock instead of hal_stm32.
 */

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "hal_mock.h"
#include "control.h"
#include "state.h"

#include "constants.hpp"
#include "transforms.hpp"
#include "SVM.hpp"
#include "pid.hpp"
#include "fast_math.hpp"

using namespace motor_driver;

/* ── Test helpers ─────────────────────────────────── */

static const float EPSILON = 1e-3f;

static bool approx_eq(float a, float b, float eps = EPSILON) {
    return fabsf(a - b) < eps;
}

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { \
    printf("  %-50s ", #name); \
    name(); \
    printf("[PASS]\n"); \
    tests_passed++; \
} while(0)

#define ASSERT_NEAR(a, b, eps) do { \
    float _a = (a), _b = (b); \
    if (!approx_eq(_a, _b, eps)) { \
        printf("[FAIL]\n    Expected %f ~= %f (eps=%f) at line %d\n", \
               _a, _b, eps, __LINE__); \
        tests_failed++; return; \
    } \
} while(0)

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        printf("[FAIL]\n    Assertion failed: %s at line %d\n", \
               #cond, __LINE__); \
        tests_failed++; return; \
    } \
} while(0)

/* ── Provide state storage (normally in state.cpp) ── */

struct Results state_results;
struct Calibration state_calibration;
struct Parameters state_parameters;

static void reset_state(void) {
    memset(&state_results, 0, sizeof(state_results));
    memset(&state_calibration, 0, sizeof(state_calibration));
    memset(&state_parameters, 0, sizeof(state_parameters));

    /* Set sensible defaults for calibration */
    state_calibration.erevs_per_mrev = 1;
    state_calibration.foc_kp_d = 0.5f;
    state_calibration.foc_ki_d = 0.1f;
    state_calibration.foc_kp_q = 1.0f;
    state_calibration.foc_ki_q = 0.2f;
    state_calibration.current_limit = 2.0f;
    state_calibration.torque_limit = 3.0f;
    state_calibration.motor_resistance = 17.8f;
    state_calibration.motor_torque_const = 1.0f;
    state_calibration.hf_velocity_filter_param = 0.01f;
    state_calibration.lf_velocity_filter_param = 0.0025f;
}

/* ── Stub state functions ────────────────────────── */

void state_init(void) {}
void state_store_calibration(void) {}
void state_load_calibration(void) {}
void state_clear_calibration(void) {}

/* ──────────────────────────────────────────────────
 *  Clarke Transform Tests
 * ────────────────────────────────────────────────── */

TEST(test_clarke_balanced_zero) {
    /* Zero input -> zero output */
    float alpha, beta;
    math::transformClarke(0.0f, 0.0f, 0.0f, alpha, beta);
    ASSERT_NEAR(alpha, 0.0f, EPSILON);
    ASSERT_NEAR(beta, 0.0f, EPSILON);
}

TEST(test_clarke_phase_a_only) {
    /* Only phase A energized -> alpha = 2/3 * a, beta = 0 */
    float alpha, beta;
    math::transformClarke(1.0f, 0.0f, 0.0f, alpha, beta);
    ASSERT_NEAR(alpha, 2.0f / 3.0f, EPSILON);
    ASSERT_NEAR(beta, 0.0f, EPSILON);
}

TEST(test_clarke_balanced_positive) {
    /* Balanced 3-phase with ia=1, ib=-0.5, ic=-0.5 */
    float alpha, beta;
    math::transformClarke(1.0f, -0.5f, -0.5f, alpha, beta);
    ASSERT_NEAR(alpha, 1.0f, EPSILON);
    ASSERT_NEAR(beta, 0.0f, EPSILON);
}

TEST(test_clarke_b_minus_c) {
    /* ib=1, ic=-1 -> alpha = (2*0-1+1)/3 = 0, beta = (1-(-1))/sqrt3 = 2/sqrt3 */
    float alpha, beta;
    math::transformClarke(0.0f, 1.0f, -1.0f, alpha, beta);
    ASSERT_NEAR(alpha, 0.0f, 0.01f);
    ASSERT_NEAR(beta, 2.0f * consts::one_div_sqrt3, 0.01f);
}

/* ──────────────────────────────────────────────────
 *  Park Transform Tests
 * ────────────────────────────────────────────────── */

TEST(test_park_zero_angle) {
    /* At angle=0: d=alpha, q=beta */
    float d, q;
    math::transformPark(1.0f, 0.0f, 1.0f, 0.0f, d, q);
    ASSERT_NEAR(d, 1.0f, EPSILON);
    ASSERT_NEAR(q, 0.0f, EPSILON);
}

TEST(test_park_90_degrees) {
    /* At angle=pi/2: cos=0, sin=1 */
    float d, q;
    math::transformPark(1.0f, 0.0f, 0.0f, 1.0f, d, q);
    ASSERT_NEAR(d, 0.0f, EPSILON);
    ASSERT_NEAR(q, -1.0f, EPSILON);
}

TEST(test_park_inverse_roundtrip) {
    /* Park followed by inverse Park should give back original values */
    float cos_t = cosf(0.7f), sin_t = sinf(0.7f);
    float alpha_in = 1.5f, beta_in = -0.3f;

    float d, q;
    math::transformPark(alpha_in, beta_in, cos_t, sin_t, d, q);

    float alpha_out, beta_out;
    math::transformInversePark(d, q, cos_t, sin_t, alpha_out, beta_out);

    ASSERT_NEAR(alpha_out, alpha_in, EPSILON);
    ASSERT_NEAR(beta_out, beta_in, EPSILON);
}

/* ──────────────────────────────────────────────────
 *  SVM Tests
 * ────────────────────────────────────────────────── */

TEST(test_svm_zero_voltage) {
    controller::SVM svm(controller::SVMStrategy::MIDPOINT_CLAMP);
    float da, db, dc;
    svm.computeDutyCycles(0.0f, 0.0f, da, db, dc);
    ASSERT_NEAR(da, 0.5f, EPSILON);
    ASSERT_NEAR(db, 0.5f, EPSILON);
    ASSERT_NEAR(dc, 0.5f, EPSILON);
}

TEST(test_svm_duties_sum) {
    /* For midpoint clamp, duties should be centered around 0.5 */
    controller::SVM svm(controller::SVMStrategy::MIDPOINT_CLAMP);
    float da, db, dc;
    svm.computeDutyCycles(0.5f, 0.3f, da, db, dc);

    /* All duties should be in [0, 1] */
    ASSERT_TRUE(da >= 0.0f && da <= 1.0f);
    ASSERT_TRUE(db >= 0.0f && db <= 1.0f);
    ASSERT_TRUE(dc >= 0.0f && dc <= 1.0f);
}

TEST(test_svm_sinusoidal_max_amplitude) {
    controller::SVM svm(controller::SVMStrategy::SINUSOIDAL);
    ASSERT_NEAR(svm.getMaxAmplitude(), consts::sqrt3_div_2, EPSILON);
}

TEST(test_svm_midpoint_max_amplitude) {
    controller::SVM svm(controller::SVMStrategy::MIDPOINT_CLAMP);
    ASSERT_NEAR(svm.getMaxAmplitude(), 1.0f, EPSILON);
}

/* ──────────────────────────────────────────────────
 *  PID Tests
 * ────────────────────────────────────────────────── */

TEST(test_pid_proportional) {
    controller::PID pid(1.0f, 0.0f, 0.0f, 0.001f);
    pid.setLimits(-10.0f, 10.0f);
    pid.setTarget(5.0f);

    /* With P=1, error of 5 should give output of 5 */
    float out = pid.compute(0.0f);
    ASSERT_NEAR(out, 5.0f, EPSILON);
}

TEST(test_pid_limits) {
    controller::PID pid(10.0f, 0.0f, 0.0f, 0.001f);
    pid.setLimits(-2.0f, 2.0f);
    pid.setTarget(5.0f);

    /* Output should be clamped to max=2.0 */
    float out = pid.compute(0.0f);
    ASSERT_NEAR(out, 2.0f, EPSILON);
}

TEST(test_pid_convergence) {
    controller::PID pid(0.5f, 0.1f, 0.0f, 0.001f);
    pid.setLimits(-10.0f, 10.0f);
    pid.setTarget(1.0f);

    /* Simulate several steps with feedback */
    float val = 0.0f;
    for (int i = 0; i < 100; i++) {
        float out = pid.compute(val);
        val += out * 0.01f;
    }

    /* Value should approach target */
    ASSERT_TRUE(fabsf(val - 1.0f) < 1.0f);
}

/* ──────────────────────────────────────────────────
 *  Fast Math (LUT sin/cos) Tests
 * ────────────────────────────────────────────────── */

TEST(test_fast_sin_zero) {
    ASSERT_NEAR(math::fast_sin(0.0f), 0.0f, 0.01f);
}

TEST(test_fast_sin_pi_half) {
    ASSERT_NEAR(math::fast_sin(consts::pi / 2.0f), 1.0f, 0.01f);
}

TEST(test_fast_cos_zero) {
    ASSERT_NEAR(math::fast_cos(0.0f), 1.0f, 0.01f);
}

TEST(test_fast_sin_cos_identity) {
    /* sin^2 + cos^2 = 1 for various angles */
    float angles[] = { 0.0f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f, 5.5f };
    for (float a : angles) {
        float s = math::fast_sin(a);
        float c = math::fast_cos(a);
        ASSERT_NEAR(s * s + c * c, 1.0f, 0.02f);
    }
}

/* ──────────────────────────────────────────────────
 *  Estimate State Tests
 * ────────────────────────────────────────────────── */

TEST(test_control_init_primes_encoder) {
    mock_reset_all();
    reset_state();
    mock_spi_set_response(0x1000);

    control_init();

    /* control_init should have called hal_spi_transfer16 to prime pipeline */
    ASSERT_TRUE(mock_spi_get_last_tx() == 0xFFFF);
}

TEST(test_control_step_reads_adc) {
    mock_reset_all();
    reset_state();

    /* Set up mock ADC with known values (zero-current = 2048) */
    mock_adc_set_samples(2048, 2048, 2048, 1000);
    mock_spi_set_response(0x0000);  /* Encoder at 0 */

    control_init();

    /* Run enough steps to fill the rolling average buffer.
     * Prescaler divides by 2, rolling average has 5 slots,
     * so we need 10 calls (5 actual executions). */
    for (int i = 0; i < 10; i++) {
        control_step();
    }

    /* After rolling average fills, currents should be near zero */
    ASSERT_NEAR(state_results.ia, 0.0f, 0.5f);
    ASSERT_NEAR(state_results.ib, 0.0f, 0.5f);
    ASSERT_NEAR(state_results.ic, 0.0f, 0.5f);
}

TEST(test_control_step_encoder_position) {
    mock_reset_all();
    reset_state();
    mock_adc_set_samples(2048, 2048, 2048, 2048);

    /* Encoder at 90 degrees: 16384/4 = 4096 counts */
    mock_spi_set_response(4096);

    control_init();

    /* Run several steps */
    for (int i = 0; i < 4; i++) {
        control_step();
    }

    /* raw_enc_value should be 4096 & 0x3FFF = 4096 */
    ASSERT_TRUE(state_results.raw_enc_value == 4096);
}

/* ──────────────────────────────────────────────────
 *  Control Mode Tests
 * ────────────────────────────────────────────────── */

TEST(test_brake_sets_raw_pwm_mode) {
    mock_reset_all();
    reset_state();
    state_parameters.control_mode = consts::control_mode_velocity;
    state_parameters.phase0 = 0.5f;
    state_parameters.phase1 = 0.5f;
    state_parameters.phase2 = 0.5f;

    control_brake();

    ASSERT_TRUE(state_parameters.control_mode == consts::control_mode_raw_phase_pwm);
    ASSERT_NEAR(state_parameters.phase0, 0.0f, EPSILON);
    ASSERT_NEAR(state_parameters.phase1, 0.0f, EPSILON);
    ASSERT_NEAR(state_parameters.phase2, 0.0f, EPSILON);
}

TEST(test_raw_pwm_mode_sets_duties) {
    mock_reset_all();
    reset_state();
    mock_adc_set_samples(2048, 2048, 2048, 2048);
    mock_spi_set_response(0);

    state_parameters.control_mode = consts::control_mode_raw_phase_pwm;
    state_parameters.phase0 = 0.5f;
    state_parameters.phase1 = 0.3f;
    state_parameters.phase2 = 0.7f;

    control_init();

    /* Run steps to execute control */
    for (int i = 0; i < 4; i++) {
        control_step();
    }

    /* Duties should be phase * max_duty_cycle */
    ASSERT_NEAR(mock_pwm_get_motor_duty(0), 0.5f * consts::max_duty_cycle, 0.01f);
    ASSERT_NEAR(mock_pwm_get_motor_duty(1), 0.3f * consts::max_duty_cycle, 0.01f);
    ASSERT_NEAR(mock_pwm_get_motor_duty(2), 0.7f * consts::max_duty_cycle, 0.01f);
}

TEST(test_timeout_triggers_brake) {
    mock_reset_all();
    reset_state();
    mock_adc_set_samples(2048, 2048, 2048, 2048);
    mock_spi_set_response(0);

    state_calibration.control_timeout = 100;  /* 100ms timeout */
    state_parameters.control_mode = consts::control_mode_foc_current;

    control_init();

    /* Advance time past timeout */
    mock_timer_set_ms(200);

    /* Run control step */
    for (int i = 0; i < 4; i++) {
        control_step();
    }

    /* Should have braked */
    ASSERT_TRUE(state_parameters.control_mode == consts::control_mode_raw_phase_pwm);
    ASSERT_TRUE(state_parameters.timeout_flag);
}

TEST(test_timeout_reset_prevents_brake) {
    mock_reset_all();
    reset_state();
    mock_adc_set_samples(2048, 2048, 2048, 2048);
    mock_spi_set_response(0);

    state_calibration.control_timeout = 100;
    state_parameters.control_mode = consts::control_mode_foc_current;

    control_init();

    /* Advance time but reset timeout */
    mock_timer_set_ms(50);
    control_reset_timeout();
    mock_timer_set_ms(100);

    for (int i = 0; i < 4; i++) {
        control_step();
    }

    /* Should NOT have braked (only 50ms since reset) */
    ASSERT_TRUE(state_parameters.control_mode == consts::control_mode_foc_current);
    ASSERT_TRUE(!state_parameters.timeout_flag);
}

/* ──────────────────────────────────────────────────
 *  Main
 * ────────────────────────────────────────────────── */

int main(void) {
    printf("=== Control Module Unit Tests ===\n\n");

    printf("Clarke Transform:\n");
    RUN_TEST(test_clarke_balanced_zero);
    RUN_TEST(test_clarke_phase_a_only);
    RUN_TEST(test_clarke_balanced_positive);
    RUN_TEST(test_clarke_b_minus_c);

    printf("\nPark Transform:\n");
    RUN_TEST(test_park_zero_angle);
    RUN_TEST(test_park_90_degrees);
    RUN_TEST(test_park_inverse_roundtrip);

    printf("\nSVM:\n");
    RUN_TEST(test_svm_zero_voltage);
    RUN_TEST(test_svm_duties_sum);
    RUN_TEST(test_svm_sinusoidal_max_amplitude);
    RUN_TEST(test_svm_midpoint_max_amplitude);

    printf("\nPID:\n");
    RUN_TEST(test_pid_proportional);
    RUN_TEST(test_pid_limits);
    RUN_TEST(test_pid_convergence);

    printf("\nFast Math:\n");
    RUN_TEST(test_fast_sin_zero);
    RUN_TEST(test_fast_sin_pi_half);
    RUN_TEST(test_fast_cos_zero);
    RUN_TEST(test_fast_sin_cos_identity);

    printf("\nState Estimation:\n");
    RUN_TEST(test_control_init_primes_encoder);
    RUN_TEST(test_control_step_reads_adc);
    RUN_TEST(test_control_step_encoder_position);

    printf("\nControl Modes:\n");
    RUN_TEST(test_brake_sets_raw_pwm_mode);
    RUN_TEST(test_raw_pwm_mode_sets_duties);
    RUN_TEST(test_timeout_triggers_brake);
    RUN_TEST(test_timeout_reset_prevents_brake);

    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
