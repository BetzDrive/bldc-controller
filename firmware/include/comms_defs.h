#ifndef _COMMS_DEFS_H_
#define _COMMS_DEFS_H_

#include <stdint.h>

namespace motor_driver {

using comm_errors_t = uint16_t;
constexpr comm_errors_t COMM_ERRORS_MALFORMED = 1;
constexpr comm_errors_t COMM_ERRORS_INVALID_FC = 2;
constexpr comm_errors_t COMM_ERRORS_INVALID_ADDR = 4;
constexpr comm_errors_t COMM_ERRORS_BUF_LEN_MISMATCH = 8;

using comm_id_t = uint8_t;
constexpr comm_id_t COMM_ID_BROADCAST = 0;
constexpr comm_id_t COMM_ID_MIN = 1;
constexpr comm_id_t COMM_ID_MAX = UINT8_MAX;

using comm_fc_t = uint8_t;
constexpr comm_fc_t COMM_FC_NOP = 0;
constexpr comm_fc_t COMM_FC_READ_REGS = 1;
constexpr comm_fc_t COMM_FC_WRITE_REGS = 2;

using comm_addr_t = uint16_t;

using comm_reg_count_t = uint8_t;

} // namespace motor_driver

#endif /* _COMMS_DEFS_H_ */
