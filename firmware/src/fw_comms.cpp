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

  for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
    switch (addr) {
      case 0x0100: //256
        handleVarAccess(results.encoder_angle, buf, index, buf_size, access_type, errors);
        break;
      case 0x0101: //257
        handleVarAccess(calibration.encoder_zero, buf, index, buf_len, access_type, errors);
        break;
      case 0x0102: //258
        handleVarAccess(parameters.raw_pwm_mode, buf, index, buf_len, access_type, errors);
        break;
      case 0x0103: //259
        handleVarAccess(parameters.phase0, buf, index, buf_len, access_type, errors);
        break;
      case 0x0104: //260
        handleVarAccess(parameters.phase1, buf, index, buf_len, access_type, errors);
        break;
      case 0x0105: //261
        handleVarAccess(parameters.phase2, buf, index, buf_len, access_type, errors);
        break;
      case 0x0106: //262
        handleVarAccess(parameters.cmd_duty_cycle, buf, index, buf_len, access_type, errors);
        break;
      case 0x0107: //263
        handleVarAccess(results.angle, buf, index, buf_size, access_type, errors);
        break;
      case 0x0108: //264
        handleVarAccess(temp, buf, index, buf_len, access_type, errors);
        motor_pwm_config.period = static_cast<pwmcnt_t>(motor_pwm_clock_freq / temp);
        pwmStart(&PWMD1, &motor_pwm_config);
        break;
      case 0x0109: //265
        handleVarAccess(calibration.flip_phases, buf, index, buf_len, access_type, errors);
        break;
      case 0x010a: //266
        handleVarAccess(calibration.erevs_per_mrev, buf, index, buf_len, access_type, errors);
        break;
      case 0x010b: //267
        handleVarAccess(results.encoder_radian_angle, buf, index, buf_size, access_type, errors);
        break;
      case 0x010c: //268
        if (temp_sensor.getTemperature(&temp)) {
          handleVarAccess(temp, buf, index, buf_size, access_type, errors);
        } else {
          errors |= COMM_ERRORS_OP_FAILED;
        }
        break;
      case 0x0200:
        handleVarAccess(results.average_ia, buf, index, buf_size, access_type, errors);
        break;
      case 0x0201:
        handleVarAccess(results.average_ib, buf, index, buf_size, access_type, errors);
        break;
      case 0x0202:
        handleVarAccess(results.average_ic, buf, index, buf_size, access_type, errors);
        break;
      case 0x0203:
        handleVarAccess(results.average_va, buf, index, buf_size, access_type, errors);
        break;
      case 0x0204:
        handleVarAccess(results.average_vb, buf, index, buf_size, access_type, errors);
        break;
      case 0x0205:
        handleVarAccess(results.average_vc, buf, index, buf_size, access_type, errors);
        break;
      case 0x0206:
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
