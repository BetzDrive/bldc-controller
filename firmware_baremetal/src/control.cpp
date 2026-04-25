/*
 * FOC motor control loop.
 * Ported from firmware/src/control.cpp (ChibiOS) to baremetal.
 *
 * This is platform-independent application logic. It calls HAL interfaces
 * only (hal_pwm, hal_adc, hal_spi, hal_timer, hal_gpio) and is fully
 * unit-testable on the host.
 */

#include "control.h"
#include "state.h"
#include "baremetal_config.h"

#include "hal/hal_pwm.h"
#include "hal/hal_adc.h"
#include "hal/hal_spi.h"
#include "hal/hal_timer.h"
#include "hal/hal_gpio.h"

#include "constants.hpp"
#include "transforms.hpp"
#include "SVM.hpp"
#include "pid.hpp"
#include "fast_math.hpp"
#include "LUTFunction.hpp"

#include <algorithm>
#include <cmath>

using namespace motor_driver;

/* ── AS5047D encoder command ─────────────────────── */

/*
 * Read ANGLECOM register (0x3FFF).
 * Frame: [parity=1][R/W=1][addr=0x3FFF] = 0xFFFF
 * The response comes back in the NEXT SPI transfer (pipelined).
 */
#define AS5047D_READ_ANGLE  0xFFFF
#define AS5047D_ANGLE_MASK  0x3FFF

/* ── Static control state ────────────────────────── */

static controller::SVM modulator(controller::SVMStrategy::MIDPOINT_CLAMP);

static controller::PID pid_id(0.0f, 0.0f, 0.0f, consts::current_control_interval);
static controller::PID pid_iq(0.0f, 0.0f, 0.0f, consts::current_control_interval);
static controller::PID pid_velocity(0.0f, 0.0f, 0.0f, consts::velocity_control_interval);
static controller::PID pid_position(0.0f, 0.0f, 0.0f, consts::position_control_interval);

static uint32_t last_timeout_reset_ms;
static uint32_t loop_count;
static bool encoder_primed;

/* Rolling ADC average buffers */
static uint16_t roll_ia[consts::ivsense_rolling_average_count];
static uint16_t roll_ib[consts::ivsense_rolling_average_count];
static uint16_t roll_ic[consts::ivsense_rolling_average_count];
static uint16_t roll_vin[consts::ivsense_rolling_average_count];
static uint32_t roll_sum_ia, roll_sum_ib, roll_sum_ic, roll_sum_vin;
static uint16_t roll_idx;

/* Encoder angle correction periodicity */
static const math::LFFlipType enc_corr_flips[] = { math::LFFlipType::NONE };
static const math::LFPeriodicity enc_corr_period = { 1, enc_corr_flips };

/* ── Helper functions ────────────────────────────── */

static inline float clampf(float val, float lo, float hi) {
    if (val > hi) return hi;
    if (val < lo) return lo;
    return val;
}

static float Q_rsqrt(float number) {
    const float x2 = number * 0.5f;
    union { float f; uint32_t i; } conv = { number };
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= (1.5f - (x2 * conv.f * conv.f));
    return conv.f;
}

static inline float adc_to_current(uint16_t count) {
    return ((float)count - consts::isense_count_zero_current) *
           consts::isense_current_per_count;
}

static inline float adc_to_voltage(uint16_t count) {
    return (float)count * consts::vsense_voltage_per_count;
}

static float get_encoder_angle_correction(float raw_enc_pos) {
    if (state_calibration.enc_ang_corr_scale != 0.0f &&
        state_calibration.enc_ang_corr_table_values.size > 0) {
        math::LUTFunction<int8_t> table(
            0, 2 * consts::pi,
            state_calibration.enc_ang_corr_table_values.bytes,
            state_calibration.enc_ang_corr_table_values.size,
            enc_corr_period);
        return (table(raw_enc_pos) * state_calibration.enc_ang_corr_scale) +
               state_calibration.enc_ang_corr_offset;
    }
    return 0.0f;
}

/* ── State estimation ────────────────────────────── */

static void estimate_state(void) {
    /*
     * Read encoder via pipelined SPI.
     * Each transfer sends a read command and receives the result
     * from the PREVIOUS command.
     */
    uint16_t raw_enc_value = hal_spi_transfer16(AS5047D_READ_ANGLE) & AS5047D_ANGLE_MASK;
    state_results.raw_enc_value = raw_enc_value;

    float raw_enc_pos = raw_enc_value * consts::rad_per_enc_tick;
    float enc_pos = raw_enc_pos + get_encoder_angle_correction(raw_enc_pos);

    float prev_enc_pos = state_results.enc_pos;
    state_results.enc_pos = enc_pos;

    /* Track revolutions (wraparound detection) */
    float enc_pos_diff = enc_pos - prev_enc_pos;
    if (enc_pos_diff < -consts::pi) {
        state_results.rotor_revs += 1;
    } else if (enc_pos_diff > consts::pi) {
        state_results.rotor_revs -= 1;
    }

    float prev_rotor_pos = state_results.rotor_pos;
    state_results.rotor_pos =
        enc_pos + (state_results.rotor_revs * 2 * consts::pi) -
        state_calibration.position_offset;
    float rotor_pos_diff = state_results.rotor_pos - prev_rotor_pos;

    /* Velocity estimation with dual low-pass filters */
    float dt_inverse = 10000.0f;  /* ~10kHz actual loop rate */
    float rotor_vel_update = rotor_pos_diff * dt_inverse;

    float hf_alpha = state_calibration.hf_velocity_filter_param;
    state_results.hf_rotor_vel =
        (hf_alpha * rotor_vel_update) +
        ((1.0f - hf_alpha) * state_results.hf_rotor_vel);

    float lf_alpha = state_calibration.lf_velocity_filter_param;
    state_results.lf_rotor_vel =
        (lf_alpha * rotor_vel_update) +
        ((1.0f - lf_alpha) * state_results.lf_rotor_vel);

    /*
     * Read ADC samples and compute rolling average.
     */
    struct AdcSamples adc = hal_adc_get_latest();

    /* Subtract old values before storing new ones */
    if (roll_vin[roll_idx] != 0) {
        roll_sum_ia  -= roll_ia[roll_idx];
        roll_sum_ib  -= roll_ib[roll_idx];
        roll_sum_ic  -= roll_ic[roll_idx];
        roll_sum_vin -= roll_vin[roll_idx];
    }

    roll_ia[roll_idx]  = adc.ia;
    roll_ib[roll_idx]  = adc.ib;
    roll_ic[roll_idx]  = adc.ic;
    roll_vin[roll_idx] = adc.vbus;

    roll_sum_ia  += adc.ia;
    roll_sum_ib  += adc.ib;
    roll_sum_ic  += adc.ic;
    roll_sum_vin += adc.vbus;

    roll_idx = (roll_idx + 1) % consts::ivsense_rolling_average_count;

    /* Convert averaged ADC counts to physical units */
    float avg_ia  = adc_to_current(roll_sum_ia / consts::ivsense_rolling_average_count);
    float avg_ib  = adc_to_current(roll_sum_ib / consts::ivsense_rolling_average_count);
    float avg_ic  = adc_to_current(roll_sum_ic / consts::ivsense_rolling_average_count);
    float avg_vin = adc_to_voltage(roll_sum_vin / consts::ivsense_rolling_average_count);

    state_results.ia  = avg_ia - state_calibration.ia_offset;
    state_results.ib  = avg_ib - state_calibration.ib_offset;
    state_results.ic  = avg_ic - state_calibration.ic_offset;
    state_results.vin = avg_vin;

    state_results.estimation_loops++;
}

/* ── Position control (400Hz) ────────────────────── */

static void run_position_control(void) {
    if (state_parameters.control_mode == consts::control_mode_position ||
        state_parameters.control_mode == consts::control_mode_position_velocity ||
        state_parameters.control_mode == consts::control_mode_position_feed_forward) {

        pid_position.setGains(state_calibration.position_kp, 0.0f,
                              state_calibration.position_kd);
        pid_position.setAlpha(consts::position_control_alpha);
        pid_position.setLimits(-state_calibration.torque_limit,
                               state_calibration.torque_limit);
        pid_position.setTarget(state_parameters.position_sp);
        state_parameters.torque_sp =
            pid_position.compute(state_results.rotor_pos);
    }
}

/* ── Velocity control (2kHz) ─────────────────────── */

static void run_velocity_control(void) {
    if (state_parameters.control_mode == consts::control_mode_velocity ||
        state_parameters.control_mode == consts::control_mode_position_velocity) {

        pid_velocity.setGains(state_calibration.velocity_kp, 0.0f,
                              state_calibration.velocity_kd);
        pid_velocity.setLimits(-state_calibration.torque_limit,
                               state_calibration.torque_limit);
        pid_velocity.setTarget(state_parameters.velocity_sp);
        state_parameters.torque_sp =
            pid_velocity.compute(state_results.hf_rotor_vel);
    }
}

/* ── Current control (10kHz) ─────────────────────── */

static void run_current_control(void) {
    if (state_parameters.control_mode == consts::control_mode_raw_phase_pwm) {
        /* Directly set PWM duty cycles */
        hal_pwm_motor_set_duty(0, state_parameters.phase0 * consts::max_duty_cycle);
        hal_pwm_motor_set_duty(1, state_parameters.phase1 * consts::max_duty_cycle);
        hal_pwm_motor_set_duty(2, state_parameters.phase2 * consts::max_duty_cycle);
        return;
    }

    /* ── Field-oriented control ── */

    /* Clarke transform: 3-phase -> alpha/beta */
    float ialpha, ibeta;
    math::transformClarke(state_results.ia, state_results.ib,
                          state_results.ic, ialpha, ibeta);

    if (state_calibration.flip_phases) {
        ibeta = -ibeta;
    }

    /* Compute electrical angle */
    float mech_pos =
        state_results.enc_pos -
        state_calibration.erev_start * consts::rad_per_enc_tick;
    float elec_pos = mech_pos * state_calibration.erevs_per_mrev;

    float cos_theta = math::fast_cos(elec_pos);
    float sin_theta = math::fast_sin(elec_pos);

    /* Park transform: alpha/beta -> d/q */
    float id, iq;
    math::transformPark(ialpha, ibeta, cos_theta, sin_theta, id, iq);

    /* Update PID gains from calibration */
    pid_id.setGains(state_calibration.foc_kp_d,
                    state_calibration.foc_ki_d, 0.0f);
    pid_iq.setGains(state_calibration.foc_kp_q,
                    state_calibration.foc_ki_q, 0.0f);

    pid_id.setLimits(-state_calibration.current_limit,
                     state_calibration.current_limit);
    pid_iq.setLimits(-state_calibration.current_limit,
                     state_calibration.current_limit);

    /* Determine d/q current setpoints based on control mode */
    float id_sp, iq_sp;
    if (state_parameters.control_mode == consts::control_mode_foc_current) {
        id_sp = state_parameters.foc_d_current_sp;
        iq_sp = state_parameters.foc_q_current_sp;
    } else if (state_parameters.control_mode == consts::control_mode_position_feed_forward) {
        id_sp = 0.0f;
        float kt = state_calibration.motor_torque_const;
        iq_sp = ((kt > 0.0f) ? (state_parameters.torque_sp / kt) : 0.0f) +
                state_parameters.feed_forward;
    } else {
        id_sp = 0.0f;
        float kt = state_calibration.motor_torque_const;
        iq_sp = (kt > 0.0f) ? (state_parameters.torque_sp / kt) : 0.0f;
    }

    /* Compute voltage commands */
    float vd = 0.0f;
    float vq = 0.0f;
    if (state_parameters.control_mode == consts::control_mode_pwm_drive) {
        vd = 0.0f;
        vq = state_parameters.pwm_drive;
    } else {
        pid_id.setTarget(id_sp);
        pid_iq.setTarget(iq_sp);

        state_results.id_output = pid_id.compute(id);
        state_results.iq_output = pid_iq.compute(iq);

        vd = state_results.id_output * state_calibration.motor_resistance;
        vq = state_results.iq_output * state_calibration.motor_resistance;
    }

    /* Normalize voltage vector to bus voltage */
    float mag = Q_rsqrt(vd * vd + vq * vq);
    float div = std::min(1.0f / state_results.vin, mag);
    float vd_norm = vd * div;
    float vq_norm = vq * div;

    /* Inverse Park: d/q -> alpha/beta */
    float valpha_norm, vbeta_norm;
    math::transformInversePark(vd_norm, vq_norm, cos_theta, sin_theta,
                               valpha_norm, vbeta_norm);

    if (state_calibration.flip_phases) {
        vbeta_norm = -vbeta_norm;
    }

    /* Space Vector Modulation: alpha/beta -> duty cycles */
    modulator.computeDutyCycles(valpha_norm, vbeta_norm,
                                state_results.duty_a,
                                state_results.duty_b,
                                state_results.duty_c);

    /* Clamp and apply duty cycles */
    if (state_parameters.gate_active) {
        state_results.duty_a = clampf(state_results.duty_a,
                                      consts::min_duty_cycle,
                                      consts::max_duty_cycle);
        state_results.duty_b = clampf(state_results.duty_b,
                                      consts::min_duty_cycle,
                                      consts::max_duty_cycle);
        state_results.duty_c = clampf(state_results.duty_c,
                                      consts::min_duty_cycle,
                                      consts::max_duty_cycle);
    } else {
        state_results.duty_a = 0.0f;
        state_results.duty_b = 0.0f;
        state_results.duty_c = 0.0f;
    }

    hal_pwm_motor_set_duty(0, state_results.duty_a);
    hal_pwm_motor_set_duty(1, state_results.duty_b);
    hal_pwm_motor_set_duty(2, state_results.duty_c);

    state_results.foc_d_current = id;
    state_results.foc_q_current = iq;
    state_results.foc_d_voltage = vd;
    state_results.foc_q_voltage = vq;
}

/* ── Public API ──────────────────────────────────── */

extern "C" void control_init(void) {
    pid_id.setLimits(-consts::isense_current_max, consts::isense_current_max);
    pid_iq.setLimits(-consts::isense_current_max, consts::isense_current_max);

    loop_count = 0;
    encoder_primed = false;

    roll_idx = 0;
    roll_sum_ia = 0;
    roll_sum_ib = 0;
    roll_sum_ic = 0;
    roll_sum_vin = 0;
    for (unsigned i = 0; i < consts::ivsense_rolling_average_count; i++) {
        roll_ia[i] = 0;
        roll_ib[i] = 0;
        roll_ic[i] = 0;
        roll_vin[i] = 0;
    }

    last_timeout_reset_ms = hal_timer_msec();

    /* Prime the encoder SPI pipeline */
    hal_spi_transfer16(AS5047D_READ_ANGLE);
    encoder_primed = true;
}

extern "C" void control_step(void) {
    /*
     * Called from TIM1 update ISR at 20kHz.
     * Run actual control at 10kHz (every 2nd call).
     */
    static uint8_t prescaler = 0;
    prescaler = (prescaler + 1) % consts::current_control_count_per_motor_cycle;
    if (prescaler != 0) {
        return;
    }

    /* Check communication timeout */
    if (state_calibration.control_timeout != 0) {
        uint32_t now = hal_timer_msec();
        if ((now - last_timeout_reset_ms) >= state_calibration.control_timeout) {
            control_brake();
            state_parameters.timeout_flag = true;
        }
    }

    /* Gate driver management (DRV8312 RST pins, active low) */
    bool fault = !hal_gpio_read(MDRV_NFAULT_PORT, MDRV_NFAULT_PIN);
    if (!state_parameters.gate_active && !fault) {
        /* Start in brake mode (zero duty) before enabling gates */
        control_brake();
        hal_gpio_set(MDRV_RST_A_PORT, MDRV_RST_A_PIN);
        hal_gpio_set(MDRV_RST_B_PORT, MDRV_RST_B_PIN);
        hal_gpio_set(MDRV_RST_C_PORT, MDRV_RST_C_PIN);
        state_parameters.gate_active = true;
    }
    if (state_parameters.gate_active && fault) {
        hal_gpio_clear(MDRV_RST_A_PORT, MDRV_RST_A_PIN);
        hal_gpio_clear(MDRV_RST_B_PORT, MDRV_RST_B_PIN);
        hal_gpio_clear(MDRV_RST_C_PORT, MDRV_RST_C_PIN);
        control_brake();
        state_parameters.gate_active = false;
        state_parameters.gate_fault = true;
    }

    /* State estimation: encoder + ADC */
    estimate_state();

    /* Cascaded control loops with frequency dividers */
    if (loop_count % consts::pos_divider == 0) {
        run_position_control();
    }

    if (loop_count % consts::vel_divider == 0) {
        run_velocity_control();
    }

    run_current_control();

    loop_count = (loop_count + 1) % consts::current_control_freq;
}

extern "C" void control_brake(void) {
    state_parameters.phase0 = 0;
    state_parameters.phase1 = 0;
    state_parameters.phase2 = 0;
    state_parameters.control_mode = consts::control_mode_raw_phase_pwm;
}

extern "C" void control_reset_timeout(void) {
    last_timeout_reset_ms = hal_timer_msec();
}
