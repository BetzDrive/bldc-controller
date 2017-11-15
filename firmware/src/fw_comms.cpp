#include "comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include <cstring>

namespace motor_driver {

void commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, RegAccessType access_type, comm_errors_t& errors) {
  size_t index = 0;

  for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
    switch (addr) {
      case 0x0000: //Board ID
        break;
      case 0x0001: //Firmware version
        break;
      case 0x0002: //Bootloader version
        break;

      case 0x1000: //Phase A Encoder Angle (radians)
        handleVarAccess(calibration.encoder_zero, buf, index, buf_len, access_type, errors);
        break;
      case 0x1001: //Electronic Revolutions Per Mechanical Revolutions
        handleVarAccess(calibration.erevs_per_mrev, buf, index, buf_len, access_type, errors);
        break;
      case 0x1002: //Invert Phases
        handleVarAccess(calibration.flip_phases, buf, index, buf_len, access_type, errors);
        break;
      case 0x1003: //FOC Kp (d)
        break;
      case 0x1004: //FOC Ki (d)
        break;
      case 0x1005: //FOC Kp (q)
        break;
      case 0x1006: //FOC Ki (q)
        break;

      case 0x2000: //Mode
        handleVarAccess(parameters.raw_pwm_mode, buf, index, buf_len, access_type, errors);
        break;
      case 0x2001: //Current Command
        handleVarAccess(parameters.cmd_duty_cycle, buf, index, buf_len, access_type, errors);
        break;
      case 0x2002: //Raw PWM A
        handleVarAccess(parameters.phase0, buf, index, buf_len, access_type, errors);
        break;
      case 0x2003: //Raw PWM B
        handleVarAccess(parameters.phase1, buf, index, buf_len, access_type, errors);
        break;
      case 0x2004: //Raw PWM C
        handleVarAccess(parameters.phase2, buf, index, buf_len, access_type, errors);
        break;

      case 0x3000: //Rotor Position (radians)
        handleVarAccess(results.encoder_radian_angle, buf, index, buf_size, access_type, errors);
        break;
      // case 0x3001: //Rotor Velocity (rad/sec)
      //   break;
      // case 0x3002: //Motor Current (amps)
      //   break;
      // case 0x3003: //Battery Current (amps)
      //   break;
      // case 0x3004: //Accelerometer X (m/sec^2)
      //   break;
      // case 0x3005: //Accelerometer Y (m/sec^2)
      //   break;
      // case 0x3006: //Accelerometer Z (m/sec^2)
      //   break;
      // case 0x3007: //Temperature
      //   break;
      case 0x3008: //Battery Voltage
        handleVarAccess(results.average_vin, buf, index, buf_size, access_type, errors);
        break;

      case 0x8000:
        handleVarAccess(results.debug_u16, buf, index, buf_size, access_type, errors);
        break;
      case 0x8001:
        handleVarAccess(results.debug_f, buf, index, buf_size, access_type, errors);
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
