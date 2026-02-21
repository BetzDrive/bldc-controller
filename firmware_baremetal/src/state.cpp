/*
 * State management: global state variables and calibration persistence.
 * Platform-independent - uses only hal_flash.
 */

#include "state.h"

#include <string.h>

#include "hal/hal_flash.h"
#include "hal/hal_iwdg.h"
#include "baremetal_config.h"

struct Results state_results;
struct Calibration state_calibration;
struct Parameters state_parameters;

void state_init(void) {
    memset(&state_results, 0, sizeof(state_results));
    memset(&state_parameters, 0, sizeof(state_parameters));

    state_parameters.control_mode = 0;
    state_parameters.gate_active = false;
    state_parameters.gate_fault = false;
    state_parameters.timeout_flag = false;

    /* Default calibration */
    memset(&state_calibration, 0, sizeof(state_calibration));
    state_calibration.start_sequence = CALIB_START_SEQ;
    state_calibration.erevs_per_mrev = 1;
    state_calibration.foc_kp_d = 0.5f;
    state_calibration.foc_ki_d = 0.1f;
    state_calibration.foc_kp_q = 1.0f;
    state_calibration.foc_ki_q = 0.2f;
    state_calibration.velocity_kp = 0.1f;
    state_calibration.velocity_kd = 1e-3f;
    state_calibration.position_kp = 5.0f;
    state_calibration.current_limit = 2.0f;
    state_calibration.torque_limit = 3.0f;
    state_calibration.velocity_limit = 10.0f;
    state_calibration.motor_resistance = 17.8f;
    state_calibration.hf_velocity_filter_param = 0.01f;
    state_calibration.lf_velocity_filter_param = 0.0025f;
}

void state_store_calibration(void) {
    state_calibration.start_sequence = CALIB_START_SEQ;

    hal_iwdg_pause();
    hal_flash_erase(FLASH_CALIB_ADDR, sizeof(struct Calibration));
    hal_iwdg_resume();

    hal_flash_write(FLASH_CALIB_ADDR, &state_calibration,
                    sizeof(struct Calibration));
}

void state_load_calibration(void) {
    uint16_t start_seq;
    hal_flash_read(FLASH_CALIB_ADDR, &start_seq, sizeof(start_seq));

    if (start_seq == CALIB_START_SEQ) {
        hal_flash_read(FLASH_CALIB_ADDR, &state_calibration,
                       sizeof(struct Calibration));
    }
    /* Otherwise keep defaults from state_init() */
}

void state_clear_calibration(void) {
    state_init();
}
