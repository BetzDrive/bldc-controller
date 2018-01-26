#include "comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include <cstring>

namespace motor_driver {

void commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, RegAccessType access_type, comm_errors_t& errors) {
  size_t index = 0;
  float temp;
  uint8_t flag;

  for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
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
        break;
      case 0x1004: // Direct Current Controller Ki
        break;
      case 0x1005: // Quadrature Current Controller Kp
        break;
      case 0x1006: // Quadrature Current Controller Ki
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
      case 0x1022: // Motor Velocity Constant (rad/s/V)
        handleVarAccess(calibration.motor_vel_const, buf, index, buf_len, access_type, errors);
        break;

      case 0x2000: // Control Mode
        handleVarAccess(parameters.raw_pwm_mode, buf, index, buf_len, access_type, errors);
        break;
      case 0x2001: // Direct Current Command (A)
        break;
      case 0x2002: // Quadrature Current Command (A)
        handleVarAccess(parameters.cmd_duty_cycle, buf, index, buf_len, access_type, errors);
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
        handleVarAccess(results.encoder_radian_angle, buf, index, buf_size, access_type, errors);
        break;
      // case 0x3001: // Rotor Velocity (rad/sec)
      //   break;
      // case 0x3002: // Direct Current Measurement (A)
      //   break;
      // case 0x3003: // Quadrature Current Measurement (A)
      //   break;
      case 0x3004: // DC Supply Voltage (V)
        handleVarAccess(results.average_vin, buf, index, buf_size, access_type, errors);
        break;
      case 0x3005: // Board Temperature (degrees C)
        if (temp_sensor.getTemperature(&temp)) {
          handleVarAccess(temp, buf, index, buf_size, access_type, errors);
        } else {
          errors |= COMM_ERRORS_OP_FAILED;
        }
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

      case 0x3009: // Recorder start
        flag = (uint8_t) recorder.startRecord();
        handleVarAccess(flag, buf, index, buf_size, access_type, errors);
        break;
      case 0x300a: // Recorder ready
        flag = (uint8_t) recorder.readyToRead();
        handleVarAccess(flag, buf, index, buf_size, access_type, errors);
        break;
      case 0x300b: // Recorder read
        // buffer = recorder.read();
        // handleVarAccess(buffer, buf, index, buf_size, access_type, errors);
        break;

      default:
        errors |= COMM_ERRORS_INVALID_ARGS;
        return;
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
