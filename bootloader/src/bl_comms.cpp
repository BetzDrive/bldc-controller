#include <cstring>

#include "comms.hpp"

#include "hal.h"

#include "ch.h"
#include "peripherals.hpp"

namespace motor_driver {
namespace comms {

size_t commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count,
                             uint8_t *buf, size_t buf_size,
                             RegAccessType access_type, comm_errors_t &errors) {
  (void)buf;
  (void)buf_size;
  (void)access_type;

  size_t index = 0;

  for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
    switch (addr) {
      // No registers

    default:
      errors |= COMM_ERRORS_INVALID_ARGS;
      return 0;
    }

    if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) {
      break;
    }
  }

  // TODO(gbalke): check if there is still data left

  return index; // Number of bytes read/written
}

} // namespace comms
} // namespace motor_driver
