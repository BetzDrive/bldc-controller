#include "comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include <cstring>

namespace motor_driver {

void commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, RegAccessType access_type, comm_errors_t& errors) {
  size_t index = 0;

  float *recordings = nullptr;

  if (start_addr >= 0x8000) { // Recordings address
    recordings = recorder.read();
  }
  for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
    if (addr >= 0x8000) {
      if (recordings != nullptr) {
        uint16_t i = addr - 0x8000;
        uint16_t rec_size = recorder.size();
        if (i < rec_size) {
          handleVarAccess(recordings[i], buf, index, buf_size, access_type, errors);
        } else {
          errors |= COMM_ERRORS_INVALID_ARGS;
        }
      }
    } else {
      switch (addr) {
        case 0x0000: // Register Map Version
          break;
        case 0x0001: // Board ID
          break;
        case 0x0002: // Firmware Version
          break;
        case 0x0003: // Bootloader Version
          break;

        case 0x1000: // Phase A Encoder Angle (rad)
          handleVarAccess(calibration.encoder_zero, buf, index, buf_len, access_type, errors);
          break;
        case 0x1001: // Electrical Revolutions Per Mechanical Revolution
          handleVarAccess(calibration.erevs_per_mrev, buf, index, buf_len, access_type, errors);
          break;
        case 0x1002: // Invert Phases
          handleVarAccess(calibration.flip_phases, buf, index, buf_len, access_type, errors);
          break;
        case 0x1003: // Direct Current Controller Kp
          handleVarAccess(calibration.foc_kp_d, buf, index, buf_len, access_type, errors);
          break;
        case 0x1004: // Direct Current Controller Ki
          handleVarAccess(calibration.foc_ki_d, buf, index, buf_len, access_type, errors);
          break;
        case 0x1005: // Quadrature Current Controller Kp
          handleVarAccess(calibration.foc_kp_q, buf, index, buf_len, access_type, errors);
          break;
        case 0x1006: // Quadrature Current Controller Ki
          handleVarAccess(calibration.foc_ki_q, buf, index, buf_len, access_type, errors);
          break;
        case 0x1010: // Software Endstop Minimum
          handleVarAccess(calibration.sw_endstop_min, buf, index, buf_len, access_type, errors);
          break;
        case 0x1011: // Software Endstop Maximum
          handleVarAccess(calibration.sw_endstop_max, buf, index, buf_len, access_type, errors);
          break;
        case 0x1012: // Software Endstop Slope
          handleVarAccess(calibration.sw_endstop_slope, buf, index, buf_len, access_type, errors);
          break;
        case 0x1020: // Motor Resistance (ohm)
          handleVarAccess(calibration.motor_resistance, buf, index, buf_len, access_type, errors);
          break;
        case 0x1021: // Motor Inductance (H)
          handleVarAccess(calibration.motor_inductance, buf, index, buf_len, access_type, errors);
          break;
        case 0x1022: // Motor Torque Constant (N*m/A)
          handleVarAccess(calibration.motor_torque_const, buf, index, buf_len, access_type, errors);
          break;
        case 0x1040: // Velocity Filter Parameter
          handleVarAccess(calibration.velocity_filter_param, buf, index, buf_len, access_type, errors);
          break;

        case 0x2000: // Control Mode
          handleVarAccess(parameters.raw_pwm_mode, buf, index, buf_len, access_type, errors);
          break;
        case 0x2001: // Direct Current Command (A)
          handleVarAccess(parameters.foc_d_current_sp, buf, index, buf_len, access_type, errors);
          break;
        case 0x2002: // Quadrature Current Command (A)
          handleVarAccess(parameters.foc_q_current_sp, buf, index, buf_len, access_type, errors);
          break;
        case 0x2003: // Phase A Raw PWM Duty Cycle
          handleVarAccess(parameters.phase0, buf, index, buf_len, access_type, errors);
          break;
        case 0x2004: // Phase B Raw PWM Duty Cycle
          handleVarAccess(parameters.phase1, buf, index, buf_len, access_type, errors);
          break;
        case 0x2005: // Phase C Raw PWM Duty Cycle
          handleVarAccess(parameters.phase2, buf, index, buf_len, access_type, errors);
          break;
        case 0x3000: // Rotor Position (rad)
          handleVarAccess(results.encoder_pos_radians, buf, index, buf_size, access_type, errors);
          break;
        case 0x3001: // Rotor Velocity (rad/sec)
          handleVarAccess(results.encoder_vel_radians, buf, index, buf_size, access_type, errors);
          break;
        case 0x3002: // Direct Current Measurement (A)
          handleVarAccess(results.foc_d_current, buf, index, buf_size, access_type, errors);
          break;
        case 0x3003: // Quadrature Current Measurement (A)
          handleVarAccess(results.foc_q_current, buf, index, buf_size, access_type, errors);
          break;
        case 0x3004: // DC Supply Voltage (V)
          handleVarAccess(results.average_vin, buf, index, buf_size, access_type, errors);
          break;
        case 0x3005: // Board Temperature (degrees C)
          handleVarAccess(results.temperature, buf, index, buf_size, access_type, errors);
          break;
        case 0x3006: // Accelerometer X (m/s^2)
          handleVarAccess(results.xl_x, buf, index, buf_size, access_type, errors);
          break;
        case 0x3007: // Accelerometer Y (m/s^2)
          handleVarAccess(results.xl_y, buf, index, buf_size, access_type, errors);
          break;
        case 0x3008: // Accelerometer Z (m/s^2)
          handleVarAccess(results.xl_z, buf, index, buf_size, access_type, errors);
          break;
        case 0x3009: { // Recorder start
          uint8_t success = (uint8_t)recorder.startRecord();
          handleVarAccess(success, buf, index, buf_size, access_type, errors);
          break;
        }
        case 0x300a: { // Recorder buffer length / ready
          uint16_t rec_size = recorder.size();
          handleVarAccess(rec_size, buf, index, buf_size, access_type, errors);
          break;
        }
        case 0x300b: { // Recorder reset
          recorder.reset();
          uint8_t dummy = 1;
          handleVarAccess(dummy, buf, index, buf_size, access_type, errors);
          break;
        }
        default:
          errors |= COMM_ERRORS_INVALID_ARGS;
          return;
      }
    }

    if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) {
      break;
    }
  }

  // TODO: check if there is still data left

  if (access_type == RegAccessType::READ) {
    buf_len = index;
  }
}

} // namespace motor_driver
