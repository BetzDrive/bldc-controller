#ifndef COMMS_DEFS_HPP_
#define COMMS_DEFS_HPP_

#include <stdint.h>

namespace motor_driver {
namespace comms {

constexpr uint8_t COMM_VERSION = 0xfe;

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

// Flag Bits (These are toggle bits so each flag changes one bit).
using comm_fg_t = uint8_t;
constexpr comm_fg_t COMM_FG_COMP = 0b00000000;
constexpr comm_fg_t COMM_FG_BOARD = 0b00000001;
constexpr comm_fg_t COMM_FG_RESET = 0b00000010;
constexpr comm_fg_t COMM_FG_TIMEOUT = 0b00000100;

using comm_fc_t = uint8_t;
constexpr comm_fc_t COMM_FC_NOP = 0x00;
constexpr comm_fc_t COMM_FC_REG_READ = 0x01;
constexpr comm_fc_t COMM_FC_REG_WRITE = 0x02;
constexpr comm_fc_t COMM_FC_REG_READ_WRITE = 0x03;
constexpr comm_fc_t COMM_FC_CLEAR_IWDGRST = 0x10;
constexpr comm_fc_t COMM_FC_SYSTEM_RESET = 0x80;
constexpr comm_fc_t COMM_FC_JUMP_TO_ADDR = 0x81;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_COUNT = 0x82;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_START = 0x83;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_SIZE = 0x84;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_ERASE = 0x85;
constexpr comm_fc_t COMM_FC_FLASH_PROGRAM = 0x86;
constexpr comm_fc_t COMM_FC_FLASH_READ = 0x87;
constexpr comm_fc_t COMM_FC_FLASH_VERIFY = 0x88;
constexpr comm_fc_t COMM_FC_FLASH_VERIFY_ERASED = 0x89;
constexpr comm_fc_t COMM_FC_CONFIRM_ID = 0xFE;
constexpr comm_fc_t COMM_FC_ENUMERATE = 0xFF;

using comm_addr_t = uint16_t;

using comm_reg_count_t = uint8_t;

} // namespace comms
} // namespace motor_driver

#endif // COMMS_DEFS_HPP_
