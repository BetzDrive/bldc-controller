#include "comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include <cstring>

namespace motor_driver {
namespace comms {

size_t commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t buf_size, RegAccessType access_type, comm_errors_t& errors) {
  size_t index = 0;

  float *recordings = nullptr;

  if (start_addr >= 0x8000) { // Recordings address
    recordings = state::recorder.read();
  }
  for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
    if (addr >= 0x8000) {
      if (recordings != nullptr) {
        uint16_t i = addr - 0x8000;
        uint16_t rec_size = state::recorder.size();
        if (i < rec_size) {
          handleVarAccess(recordings[i], buf, index, buf_size, access_type, errors);
        } else {
          errors |= COMM_ERRORS_INVALID_ARGS;
        }
      }
    } else if (addr >= 0x1200 && addr < 0x1200 + consts::enc_ang_corr_table_size) {
      // Encoder Angle Correction Table Values
      handleVarAccess(state::calibration.enc_ang_corr_table_values[addr - 0x1200], buf, index, buf_size, access_type, errors);
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
        case 0x0004: // Store Motor Calibration to Memory
          state::storeCalibration();
          break;
        case 0x0005: // Clear Motor Calibration from Memory
          state::clearCalibration();
          break;

        case 0x1000: // Electrical Revolution Start
          handleVarAccess(state::calibration.erev_start, buf, index, buf_size, access_type, errors);
          break;
        case 0x1001: // Electrical Revolutions Per Mechanical Revolution
          handleVarAccess(state::calibration.erevs_per_mrev, buf, index, buf_size, access_type, errors);
          break;
        case 0x1002: // Invert Phases
          handleVarAccess(state::calibration.flip_phases, buf, index, buf_size, access_type, errors);
          break;
        case 0x1003: // Direct Current Controller Kp
          handleVarAccess(state::calibration.foc_kp_d, buf, index, buf_size, access_type, errors);
          break;
        case 0x1004: // Direct Current Controller Ki
          handleVarAccess(state::calibration.foc_ki_d, buf, index, buf_size, access_type, errors);
          break;
        case 0x1005: // Quadrature Current Controller Kp
          handleVarAccess(state::calibration.foc_kp_q, buf, index, buf_size, access_type, errors);
          break;
        case 0x1006: // Quadrature Current Controller Ki
          handleVarAccess(state::calibration.foc_ki_q, buf, index, buf_size, access_type, errors);
          break;
        case 0x1007: // Velocity Controller Kp
          handleVarAccess(state::calibration.velocity_kp, buf, index, buf_size, access_type, errors);
          break;
        case 0x1008: // Velocity Controller Ki
          handleVarAccess(state::calibration.velocity_ki, buf, index, buf_size, access_type, errors);
          break;
        case 0x1009: // Position Controller Kp
          handleVarAccess(state::calibration.position_kp, buf, index, buf_size, access_type, errors);
          break;
        case 0x100a: // Position Controller Ki
          handleVarAccess(state::calibration.position_ki, buf, index, buf_size, access_type, errors);
          break;
        case 0x1010: // Current Limit (A)
          handleVarAccess(state::calibration.current_limit, buf, index, buf_size, access_type, errors);
          break;
        case 0x1011: // Torque Limit (N*m)
          handleVarAccess(state::calibration.torque_limit, buf, index, buf_size, access_type, errors);
          break;
        case 0x1012: // Velocity Limit (rad/s)
          handleVarAccess(state::calibration.velocity_limit, buf, index, buf_size, access_type, errors);
          break;
        case 0x1013: // Position Lower Limit (rad)
          handleVarAccess(state::calibration.position_lower_limit, buf, index, buf_size, access_type, errors);
          break;
        case 0x1014: // Position Upper Limit (rad)
          handleVarAccess(state::calibration.position_upper_limit, buf, index, buf_size, access_type, errors);
          break;
        case 0x1015: // Position Offset
          handleVarAccess(state::calibration.position_offset, buf, index, buf_size, access_type, errors);
          break;
        case 0x1020: // Motor Resistance (ohm)
          handleVarAccess(state::calibration.motor_resistance, buf, index, buf_size, access_type, errors);
          break;
        case 0x1021: // Motor Inductance (H)
          handleVarAccess(state::calibration.motor_inductance, buf, index, buf_size, access_type, errors);
          break;
        case 0x1022: // Motor Torque Constant (N*m/A)
          handleVarAccess(state::calibration.motor_torque_const, buf, index, buf_size, access_type, errors);
          break;
        case 0x1030: // Control Timeout (ms)
          handleVarAccess(state::calibration.control_timeout, buf, index, buf_size, access_type, errors);
          break;
        case 0x1040: // HF Velocity Filter Parameter
          handleVarAccess(state::calibration.hf_velocity_filter_param, buf, index, buf_size, access_type, errors);
          break;
        case 0x1041: // LF Velocity Filter Parameter
          handleVarAccess(state::calibration.lf_velocity_filter_param, buf, index, buf_size, access_type, errors);
          break;
        case 0x1050: // Current Phase A Offset 
          handleVarAccess(state::calibration.ia_offset, buf, index, buf_size, access_type, errors);
          break;
        case 0x1051: // Current Phase B Offset 
          handleVarAccess(state::calibration.ib_offset, buf, index, buf_size, access_type, errors);
          break;
        case 0x1052: // Current Phase C Offset 
          handleVarAccess(state::calibration.ic_offset, buf, index, buf_size, access_type, errors);
          break;
        case 0x1100: // Encoder Angle Correction Scale (rad)
          handleVarAccess(state::calibration.enc_ang_corr_scale, buf, index, buf_size, access_type, errors);
          break;
        case 0x1101: // Encoder Angle Correction Offset (rad)
          handleVarAccess(state::calibration.enc_ang_corr_offset, buf, index, buf_size, access_type, errors);
          break;

        case 0x2000: // Control Mode
          handleVarAccess(state::parameters.control_mode, buf, index, buf_size, access_type, errors);
          break;
        case 0x2001: // Direct Current Command (A)
          handleVarAccess(state::parameters.foc_d_current_sp, buf, index, buf_size, access_type, errors);
          break;
        case 0x2002: // Quadrature Current Command (A)
          handleVarAccess(state::parameters.foc_q_current_sp, buf, index, buf_size, access_type, errors);
          break;
        case 0x2003: // Phase A Raw PWM Duty Cycle
          handleVarAccess(state::parameters.phase0, buf, index, buf_size, access_type, errors);
          break;
        case 0x2004: // Phase B Raw PWM Duty Cycle
          handleVarAccess(state::parameters.phase1, buf, index, buf_size, access_type, errors);
          break;
        case 0x2005: // Phase C Raw PWM Duty Cycle
          handleVarAccess(state::parameters.phase2, buf, index, buf_size, access_type, errors);
          break;
        case 0x2006: // Torque Setpoint (N*m)
          handleVarAccess(state::parameters.torque_sp, buf, index, buf_size, access_type, errors);
          break;
        case 0x2007: // Velocity Setpoint (rad/s)
          handleVarAccess(state::parameters.velocity_sp, buf, index, buf_size, access_type, errors);
          break;
        case 0x2008: // Position Setpoint (rad)
          handleVarAccess(state::parameters.position_sp, buf, index, buf_size, access_type, errors);
          break;
        case 0x2009: // PWM Drive (V)
          handleVarAccess(state::parameters.pwm_drive, buf, index, buf_size, access_type, errors);
          break;


        case 0x3000: // Rotor Position (rad)
          handleVarAccess(state::results.rotor_pos, buf, index, buf_size, access_type, errors);
          break;
        case 0x3001: // Rotor Velocity (rad/sec)
          handleVarAccess(state::results.lf_rotor_vel, buf, index, buf_size, access_type, errors);
          break;
        case 0x3002: // Direct Current Measurement (A)
          handleVarAccess(state::results.foc_d_current, buf, index, buf_size, access_type, errors);
          break;
        case 0x3003: // Quadrature Current Measurement (A)
          handleVarAccess(state::results.foc_q_current, buf, index, buf_size, access_type, errors);
          break;
        case 0x3004: // DC Supply Voltage (V)
          handleVarAccess(state::results.average_vin, buf, index, buf_size, access_type, errors);
          break;
        case 0x3005: // Board Temperature (degrees C)
          handleVarAccess(state::results.temperature, buf, index, buf_size, access_type, errors);
          break;
        case 0x3006: // Accelerometer X (m/s^2)
          handleVarAccess(state::results.xl_x, buf, index, buf_size, access_type, errors);
          break;
        case 0x3007: // Accelerometer Y (m/s^2)
          handleVarAccess(state::results.xl_y, buf, index, buf_size, access_type, errors);
          break;
        case 0x3008: // Accelerometer Z (m/s^2)
          handleVarAccess(state::results.xl_z, buf, index, buf_size, access_type, errors);
          break;
        case 0x3009: { // Recorder start
          uint8_t success = (uint8_t) state::recorder.startRecording();
          handleVarAccess(success, buf, index, buf_size, access_type, errors);
          break;
        }
        case 0x300a: { // Recorder buffer length / ready
          uint16_t rec_size = state::recorder.size();
          handleVarAccess(rec_size, buf, index, buf_size, access_type, errors);
          break;
        }
        case 0x300b: { // Recorder reset
          state::recorder.reset();
          uint8_t flag = 1;
          handleVarAccess(flag, buf, index, buf_size, access_type, errors);
          break;
        }
        case 0x3010: // Rotor Position (raw) 
          handleVarAccess(state::results.raw_enc_value, buf, index, buf_size, access_type, errors);
          break;
        case 0x3011: // iq output from PID controller (A) 
          handleVarAccess(state::results.iq_output, buf, index, buf_size, access_type, errors);
          break;
       
 
 
        default:
          errors |= COMM_ERRORS_INVALID_ARGS;
          return 0;
      }
    }

    if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) {
      break;
    }
  }

  // TODO: check if there is still data left

  return index; // Number of bytes read/written
}

} // namespace comms
} // namespace motor_driver
