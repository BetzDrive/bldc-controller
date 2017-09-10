#ifndef _COMMS_DEFS_H_
#define _COMMS_DEFS_H_

#include <stdint.h>

namespace motor_driver {

using comm_errors_t = uint16_t;
constexpr comm_errors_t COMM_ERRORS_NONE = 0;
constexpr comm_errors_t COMM_ERRORS_OP_FAILED = 1;
constexpr comm_errors_t COMM_ERRORS_MALFORMED = 2;
constexpr comm_errors_t COMM_ERRORS_INVALID_FC = 4;
constexpr comm_errors_t COMM_ERRORS_INVALID_ARGS = 8;
constexpr comm_errors_t COMM_ERRORS_BUF_LEN_MISMATCH = 16;

using comm_id_t = uint8_t;
constexpr comm_id_t COMM_ID_BROADCAST = 0;
constexpr comm_id_t COMM_ID_MIN = 1;
constexpr comm_id_t COMM_ID_MAX = UINT8_MAX;

using comm_fc_t = uint8_t;
constexpr comm_fc_t COMM_FC_NOP = 0x00;
constexpr comm_fc_t COMM_FC_READ_REGS = 0x01;
constexpr comm_fc_t COMM_FC_WRITE_REGS = 0x02;
constexpr comm_fc_t COMM_FC_READ_REGS_SYNCED = 0x03;
constexpr comm_fc_t COMM_FC_WRITE_REGS_SYNCED = 0x04;
constexpr comm_fc_t COMM_FC_SYNC_READS = 0x05;
constexpr comm_fc_t COMM_FC_SYNC_WRITES = 0x06;
constexpr comm_fc_t COMM_FC_ENTER_BOOTLOADER = 0x80;
constexpr comm_fc_t COMM_FC_LEAVE_BOOTLOADER = 0x81;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_COUNT = 0x82;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_START = 0x83;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_SIZE = 0x84;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_ERASE = 0x85;
constexpr comm_fc_t COMM_FC_FLASH_PROGRAM = 0x86;
constexpr comm_fc_t COMM_FC_FLASH_READ = 0x87;
constexpr comm_fc_t COMM_FC_FLASH_VERIFY = 0x88;
constexpr comm_fc_t COMM_FC_FLASH_VERIFY_ERASED = 0x89;

using comm_addr_t = uint16_t;

using comm_reg_count_t = uint8_t;

} // namespace motor_driver

#endif /* _COMMS_DEFS_H_ */
