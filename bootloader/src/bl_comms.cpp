#include "bl_comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include <cstring>
#include "helper.h"
#include "flash.h"

namespace motor_driver {

static uint32_t jump_addr = 0;

void ProtocolFSM::handleRequest(uint8_t *datagram, size_t datagram_len, comm_errors_t& errors) {
  if (state_ != State::IDLE) {
    return;
  }

  static_assert(sizeof(comm_id_t) == 1, "Assuming comm_id_t is uint8_t");
  static_assert(sizeof(comm_fc_t) == 1, "Assuming comm_fc_t is uint8_t");
  static_assert(sizeof(comm_addr_t) == 2, "Assuming comm_addr_t is uint16_t");
  static_assert(sizeof(comm_reg_count_t) == 1, "Assuming comm_reg_count_t is uint8_t");

  size_t index = 0;

  if (datagram_len - index < 2) {
    return;
  }

  comm_id_t id = datagram[index++];
  function_code_ = datagram[index++];

  if (id != server_->getID()) {
    /* This datagram is not meant for us, ignore it */
    return;
  }

  /* Clear errors */
  errors = 0;

  /* Temporary variables */
  uint32_t sector_num, dest_addr;
  size_t dest_len;
  bool success;

  switch (function_code_) {
    case COMM_FC_NOP:
      /* No operation */

      state_ = State::RESPONDING;

      break;

    case COMM_FC_LEAVE_BOOTLOADER:
      /* Leave the bootloader and jump to our application */

      if (datagram_len - index < 4) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      jump_addr = (uint32_t)datagram[index++];
      jump_addr |= (uint32_t)datagram[index++] << 8;
      jump_addr |= (uint32_t)datagram[index++] << 16;
      jump_addr |= (uint32_t)datagram[index++] << 24;

      state_ = State::RESPONDING;

      break;

    case COMM_FC_FLASH_SECTOR_COUNT:
      /* Respond with the total number of flash sectors */

      u32_value_ = FLASH_SECTOR_COUNT;

      state_ = State::RESPONDING_U32;

      break;

    case COMM_FC_FLASH_SECTOR_START:
      /* Respond with the start address of a flash sector */

      if (datagram_len - index < 4) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      sector_num = (uint32_t)datagram[index++];
      sector_num |= (uint32_t)datagram[index++] << 8;
      sector_num |= (uint32_t)datagram[index++] << 16;
      sector_num |= (uint32_t)datagram[index++] << 24;

      u32_value_ = flashSectorBegin(sector_num);

      state_ = State::RESPONDING_U32;

      break;

    case COMM_FC_FLASH_SECTOR_SIZE:
      /* Respond with the size of a flash sector, in bytes */

      if (datagram_len - index < 4) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      sector_num = (uint32_t)datagram[index++];
      sector_num |= (uint32_t)datagram[index++] << 8;
      sector_num |= (uint32_t)datagram[index++] << 16;
      sector_num |= (uint32_t)datagram[index++] << 24;

      u32_value_ = flashSectorSize(sector_num);

      state_ = State::RESPONDING_U32;

      break;

    case COMM_FC_FLASH_SECTOR_ERASE:
      /* Erase a flash sector */

      if (datagram_len - index < 4) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      sector_num = (uint32_t)datagram[index++];
      sector_num |= (uint32_t)datagram[index++] << 8;
      sector_num |= (uint32_t)datagram[index++] << 16;
      sector_num |= (uint32_t)datagram[index++] << 24;

      success = (flashSectorErase(sector_num) == FLASH_RETURN_SUCCESS);

      if (!success) {
        errors |= COMM_ERRORS_OP_FAILED;
      }

      state_ = State::RESPONDING;

      break;

    case COMM_FC_FLASH_WRITE:
      /* Write values to flash memory */

      if (datagram_len - index < 4) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      dest_addr = (uint32_t)datagram[index++];
      dest_addr |= (uint32_t)datagram[index++] << 8;
      dest_addr |= (uint32_t)datagram[index++] << 16;
      dest_addr |= (uint32_t)datagram[index++] << 24;

      dest_len = datagram_len - index;

      success = (flashWrite(dest_addr, (char *)&datagram[index], dest_len) == FLASH_RETURN_SUCCESS);

      if (!success) {
        errors |= COMM_ERRORS_OP_FAILED;
      }

      state_ = State::RESPONDING;

      break;

    case COMM_FC_FLASH_READ:
      /* Read values from flash memory */

      if (datagram_len - index < 8) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      src_addr_ = (uint32_t)datagram[index++];
      src_addr_ |= (uint32_t)datagram[index++] << 8;
      src_addr_ |= (uint32_t)datagram[index++] << 16;
      src_addr_ |= (uint32_t)datagram[index++] << 24;

      src_len_ = (size_t)datagram[index++];
      src_len_ |= (size_t)datagram[index++] << 8;
      src_len_ |= (size_t)datagram[index++] << 16;
      src_len_ |= (size_t)datagram[index++] << 24;

      state_ = State::RESPONDING_MEM;

      break;

    case COMM_FC_FLASH_VERIFY:
      /* Compare values in flash memory with expected values */

      if (datagram_len - index < 4) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      dest_addr = (uint32_t)datagram[index++];
      dest_addr |= (uint32_t)datagram[index++] << 8;
      dest_addr |= (uint32_t)datagram[index++] << 16;
      dest_addr |= (uint32_t)datagram[index++] << 24;

      dest_len = datagram_len - index;

      success = (std::memcmp((void *)dest_addr, &datagram[index], dest_len) == 0);

      if (!success) {
        errors |= COMM_ERRORS_OP_FAILED;
      }

      state_ = State::RESPONDING;

      break;

    case COMM_FC_FLASH_VERIFY_ERASED:
      /* Verify that a region of flash memory is erased (all ones) */

      if (datagram_len - index < 8) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      dest_addr = (uint32_t)datagram[index++];
      dest_addr |= (uint32_t)datagram[index++] << 8;
      dest_addr |= (uint32_t)datagram[index++] << 16;
      dest_addr |= (uint32_t)datagram[index++] << 24;

      dest_len = (size_t)datagram[index++];
      dest_len |= (size_t)datagram[index++] << 8;
      dest_len |= (size_t)datagram[index++] << 16;
      dest_len |= (size_t)datagram[index++] << 24;

      success = true;

      for (size_t i = 0; i < dest_len; i++) {
        if (((uint8_t *)dest_addr)[i] != 0xff) {
          success = false;
          break;
        }
      }

      if (!success) {
        errors |= COMM_ERRORS_OP_FAILED;
      }

      state_ = State::RESPONDING;

      break;

    default:
      /* Invalid function code */

      errors |= COMM_ERRORS_INVALID_FC;
      state_ = State::RESPONDING;

      break;
  }
}

template<typename T>
static void handleVarAccess(T& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors) {
  constexpr size_t var_size = sizeof(var);

  if (buf_size - index < var_size) {
    errors |= COMM_ERRORS_BUF_LEN_MISMATCH;
    return;
  }

  uint8_t *u8_var = reinterpret_cast<uint8_t *>(&var);

  switch (access_type) {
    case RegAccessType::READ:
      std::memcpy(buf + index, u8_var, var_size);
      index += var_size;
      break;
    case RegAccessType::WRITE:
      std::memcpy(u8_var, buf + index, var_size);
      index += var_size;
      break;
    default:
      break;
  }
}

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

void runComms() {
  comms_endpoint.waitReceive();

  if (!comms_endpoint.hasReceiveError()) {
    /* Received valid datagram */
    comm_errors_t errors = COMM_ERRORS_NONE;

    size_t receive_len = comms_endpoint.getReceiveLength();
    comms_protocol_fsm.handleRequest(comms_endpoint.getReceiveBufferPtr(), receive_len, errors);

    size_t transmit_len;
    comms_protocol_fsm.composeResponse(comms_endpoint.getTransmitBufferPtr(), transmit_len, comms_endpoint.getTransmitBufferSize(), errors);

    if (transmit_len > 0) {
      comms_endpoint.setTransmitLength(transmit_len);
      comms_endpoint.startTransmit();
    }

    /* Exit the bootloader if a jump address is set */
    if (jump_addr != 0) {
      flashJumpApplication(jump_addr);
    }
  }
}

} // namespace motor_driver
