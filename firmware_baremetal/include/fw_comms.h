#ifndef FW_COMMS_H
#define FW_COMMS_H

#include <stddef.h>
#include <stdint.h>

#include "comms_defs.hpp"

enum class RegAccessType { READ, WRITE };

using motor_driver::comms::comm_addr_t;
using motor_driver::comms::comm_errors_t;

/* Function pointers for critical section (replaceable for testing).
 * On STM32: __disable_irq / __enable_irq.
 * In tests: no-ops (default). */
extern void (*critical_section_enter)(void);
extern void (*critical_section_exit)(void);

template <typename T>
void handleVarAccess(T &var, uint8_t *buf, size_t &index, size_t buf_size,
                     RegAccessType access_type, comm_errors_t &errors);

size_t commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count,
                             uint8_t *buf, size_t buf_size,
                             RegAccessType access_type,
                             comm_errors_t &errors);

#endif /* FW_COMMS_H */
