#include "comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include <cstring>

namespace motor_driver {

void commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, RegAccessType access_type, comm_errors_t& errors) {
  size_t index = 0;

  for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
    switch (addr) {
      case 5:
        if (access_type == RegAccessType::WRITE) {
          if (buf_len - index >= 3) {
            setStatusLEDColor(buf[index], buf[index + 1], buf[index + 2]);
            index += 3;
          } else {
            errors |= COMM_ERRORS_BUF_LEN_MISMATCH;
            return;
          }
        }
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
