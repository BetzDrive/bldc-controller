#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Results {
    float foc_d_current;
    float foc_q_current;
    float foc_d_voltage;
    float foc_q_voltage;
    float id_output;
    float iq_output;
    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t encoder_mode;
    uint16_t raw_enc_value;
    float enc_pos;
    uint32_t encoder_diag;
    int16_t rotor_revs;
    float rotor_pos;
    float hf_rotor_vel;
    float lf_rotor_vel;
    float va, vb, vc;
    float vin;
    float ia, ib, ic;
    int16_t xl_x, xl_y, xl_z;
    float temperature;
    uint32_t estimation_loops;
};

struct Calibration {
    uint16_t start_sequence;
    uint16_t erev_start;
    uint8_t erevs_per_mrev;
    uint8_t flip_phases;
    float foc_kp_d;
    float foc_ki_d;
    float foc_kp_q;
    float foc_ki_q;
    float velocity_kp;
    float velocity_kd;
    float position_kp;
    float position_kd;
    float current_limit;
    float torque_limit;
    float velocity_limit;
    float position_lower_limit;
    float position_upper_limit;
    float motor_resistance;
    float motor_inductance;
    float motor_torque_const;
    uint16_t control_timeout;
    float hf_velocity_filter_param;
    float lf_velocity_filter_param;
    float position_offset;
    float ia_offset;
    float ib_offset;
    float ic_offset;
    /* Encoder angle correction */
    float enc_ang_corr_scale;
    float enc_ang_corr_offset;
    struct {
        int8_t bytes[257];  /* enc_ang_corr_table_size from constants.hpp */
        size_t size;
    } enc_ang_corr_table_values;
};

struct Parameters {
    uint8_t control_mode;
    float foc_q_current_sp;
    float foc_d_current_sp;
    bool override_led_color;
    uint8_t led_red_intensity;
    uint8_t led_green_intensity;
    uint8_t led_blue_intensity;
    float phase0, phase1, phase2;
    float torque_sp;
    float velocity_sp;
    float position_sp;
    float feed_forward;
    float pwm_drive;
    bool gate_active;
    bool gate_fault;
    bool timeout_flag;
};

#define CALIB_START_SEQ  0x5454

extern struct Results state_results;
extern struct Calibration state_calibration;
extern struct Parameters state_parameters;

void state_init(void);
void state_store_calibration(void);
void state_load_calibration(void);
void state_clear_calibration(void);

#ifdef __cplusplus
}
#endif

#endif /* STATE_H */
