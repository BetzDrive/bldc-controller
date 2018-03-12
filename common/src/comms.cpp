#include "comms.h"

#include <cstring>
#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "constants.h"
#include "helper.h"
#include "flash.h"

namespace motor_driver {

static uint32_t jump_addr = 0;

static bool should_reset = false;

void UARTEndpoint::start() {
  uartStart(uart_driver_, &uart_config_);
  gptStart(gpt_driver_, &gpt_config_);

  chSysLock();

  changeStateI(State::INITIALIZING);

  chSysUnlock();
}

void UARTEndpoint::transmit() {
  tx_buf_[0] = 0xff; // Sync flag
  tx_buf_[1] = 0xff; // Protocol version
  tx_buf_[2] = tx_len_ & 0xff;
  tx_buf_[3] = (tx_len_ >> 8) & 0xff;

  uint16_t crc = computeCRC(tx_buf_, header_len + tx_len_);
  tx_buf_[header_len + tx_len_] = crc & 0xff;
  tx_buf_[header_len + tx_len_ + 1] = (crc >> 8) & 0xff;

  chSysLock();

  if (state_ == State::IDLE) {
    palSetPad(dir_.port, dir_.pin);
    uartStartSendI(uart_driver_, header_len + tx_len_ + crc_length, tx_buf_);
    changeStateI(State::TRANSMITTING);
  }

  chSysUnlock();

  chBSemWait(&tx_bsem_);
}

void UARTEndpoint::receive() {
  chBSemWait(&rx_bsem_);

  uint16_t computed_crc = computeCRC(rx_buf_, header_len + rx_len_);
  uint16_t expected_crc = ((uint16_t)rx_buf_[header_len + rx_len_ + 1] << 8) | (uint16_t)rx_buf_[header_len + rx_len_];
  if (computed_crc != expected_crc) {
    rx_error_ = true;
  }
}

uint8_t *UARTEndpoint::getReceiveBufferPtr() {
  return rx_buf_ + header_len;
}

size_t UARTEndpoint::getReceiveLength() const {
  return rx_len_;
}

bool UARTEndpoint::hasReceiveError() const {
  return rx_error_;
}

uint8_t *UARTEndpoint::getTransmitBufferPtr() {
  return tx_buf_ + header_len;
}

void UARTEndpoint::setTransmitLength(size_t len) {
  tx_len_ = len;
}

size_t UARTEndpoint::getTransmitBufferSize() const {
  return max_dg_payload_len;
}

void UARTEndpoint::changeStateI(State new_state) {
  if (new_state == State::INITIALIZING ||
      new_state == State::RECEIVING_PROTOCOL_VERSION ||
      new_state == State::RECEIVING_LENGTH_L ||
      new_state == State::RECEIVING_LENGTH_H ||
      new_state == State::RECEIVING) {
    gptStopTimerI(gpt_driver_);
    gptStartOneShotI(gpt_driver_, idle_time_ticks_);
  }

  if (new_state == State::RECEIVING_PROTOCOL_VERSION) {
    rx_error_ = false; // Clear receive error flag
  }

  state_ = new_state;
}

void UARTEndpoint::uartTransmitCompleteCallback() {
  chSysLockFromIsr();

  palClearPad(dir_.port, dir_.pin);
  chBSemSignalI(&tx_bsem_);
  changeStateI(State::IDLE);

  chSysUnlockFromIsr();
}

void UARTEndpoint::uartReceiveCompleteCallback() {
  chSysLockFromIsr();

  switch (state_) {
    case State::RECEIVING:
      /* Finished receiving a datagram */
      gptStopTimerI(gpt_driver_);
      chBSemSignalI(&rx_bsem_);
      changeStateI(State::IDLE);
      break;
    default:
      break;
  }

  chSysUnlockFromIsr();
}

void UARTEndpoint::uartCharReceivedCallback(uint16_t c) {
  chSysLockFromIsr();

  switch (state_) {
    case State::INITIALIZING:
      /* Reset idle timeout */
      gptStopTimerI(gpt_driver_);
      gptStartOneShotI(gpt_driver_, idle_time_ticks_);
      break;
    case State::IDLE:
      /* Possible start of packet */
      rx_buf_[0] = (uint8_t)c;
      if (c == 0xff) {
        changeStateI(State::RECEIVING_PROTOCOL_VERSION);
      } else {
        changeStateI(State::INITIALIZING);
      }
      break;
    case State::RECEIVING_PROTOCOL_VERSION:
      /* Check protocol version */
      rx_buf_[1] = (uint8_t)c;
      if (c == 0xff) {
        changeStateI(State::RECEIVING_LENGTH_L);
      } else {
        changeStateI(State::INITIALIZING);
      }
      break;
    case State::RECEIVING_LENGTH_L:
      /* Store lower byte of packet length */
      rx_buf_[2] = (uint8_t)c;
      changeStateI(State::RECEIVING_LENGTH_H);
      break;
    case State::RECEIVING_LENGTH_H:
      /* Store upper byte of packet length and start receiving data */
      rx_buf_[3] = (uint8_t)c;
      rx_len_ = ((size_t)rx_buf_[3] << 8) | rx_buf_[2];
      // TODO: what if rx_len_ is too big? add some bounds checking
      uartStartReceiveI(uart_driver_, rx_len_ + crc_length, rx_buf_ + header_len);
      changeStateI(State::RECEIVING);
      break;
    default:
      break;
  }

  chSysUnlockFromIsr();
}

void UARTEndpoint::uartReceiveErrorCallback(uartflags_t e) {
  (void)e;

  chSysLockFromIsr();

  rx_error_ = true;
  changeStateI(State::INITIALIZING);

  chSysUnlockFromIsr();
}

void UARTEndpoint::gptCallback() {
  chSysLockFromIsr();

  uartStopReceiveI(uart_driver_);
  changeStateI(State::IDLE);

  chSysUnlockFromIsr();
}

uint16_t UARTEndpoint::computeCRC(const uint8_t *buf, size_t len) {
  (void)buf;
  (void)len;

  return 0; // TODO: implement
}

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

  if (id != 0 && id != server_->getID()) {
    /* This datagram is not meant for us, ignore it */
    return;
  }

  broadcast_ = (id == 0);

  if (activity_callback_ != nullptr) {
    activity_callback_();
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

    case COMM_FC_REG_READ:
      /* Read registers */

      if (datagram_len - index < 3) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      start_addr_ = (comm_addr_t)datagram[index++];
      start_addr_ |= (comm_addr_t)datagram[index++] << 8;
      reg_count_ = datagram[index++];

      state_ = State::RESPONDING_READ;

      break;

    case COMM_FC_REG_WRITE:
      /* Write registers */

      if (datagram_len - index < 3) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      start_addr_ = (comm_addr_t)datagram[index++];
      start_addr_ |= (comm_addr_t)datagram[index++] << 8;
      reg_count_ = datagram[index++];

      server_->writeRegisters(start_addr_, reg_count_, &datagram[index], datagram_len - index, errors);

      state_ = State::RESPONDING;

      break;

    case COMM_FC_REG_READ_WRITE:
      /* Simultaneous register read/write */

      if (datagram_len - index < 6) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      // Do the write first, so we can re-use the start_addr_ and reg_count_ variables for the read
      index += 3;
      start_addr_ = (comm_addr_t)datagram[index++];
      start_addr_ |= (comm_addr_t)datagram[index++] << 8;
      reg_count_ = datagram[index++];
      server_->writeRegisters(start_addr_, reg_count_, &datagram[index], datagram_len - index, errors);

      // Shift index back to perform the read
      index -= 6;
      start_addr_ = (comm_addr_t)datagram[index++];
      start_addr_ |= (comm_addr_t)datagram[index++] << 8;
      reg_count_ = datagram[index++];

      state_ = State::RESPONDING_READ;

      break;

    case COMM_FC_SYSTEM_RESET:
      /* Perform system reset */

      should_reset = true;

      state_ = State::RESPONDING;

      break;

    case COMM_FC_JUMP_TO_ADDR:
      /* Jump to an arbitrary address (only supported by bootloader) */

#ifdef BOOTLOADER
      if (datagram_len - index < 4) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      jump_addr = (uint32_t)datagram[index++];
      jump_addr |= (uint32_t)datagram[index++] << 8;
      jump_addr |= (uint32_t)datagram[index++] << 16;
      jump_addr |= (uint32_t)datagram[index++] << 24;
#endif // #ifdef BOOTLOADER

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

    case COMM_FC_FLASH_PROGRAM:
      /* Program values into flash memory */

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

void ProtocolFSM::composeResponse(uint8_t *datagram, size_t& datagram_len, size_t max_datagram_len, comm_errors_t errors) {
  if (state_ == State::IDLE) {
    /* No response to send */
    datagram_len = 0;
    return;
  }

  static_assert(sizeof(comm_id_t) == 1, "Assuming comm_id_t is uint8_t");
  static_assert(sizeof(comm_fc_t) == 1, "Assuming comm_fc_t is uint8_t");
  static_assert(sizeof(comm_addr_t) == 2, "Assuming comm_addr_t is uint16_t");
  static_assert(sizeof(comm_reg_count_t) == 1, "Assuming comm_reg_count_t is uint8_t");
  static_assert(sizeof(comm_errors_t) == 2, "Assuming comm_errors_t is uint16_t");

  size_t index = 0;

  if (max_datagram_len - index < 2) {
    datagram_len = index;
    state_ = State::IDLE;
    return;
  }

  if (broadcast_) {
    datagram[index++] = 0;
  } else {
    datagram[index++] = server_->getID();
  }

  datagram[index++] = function_code_;

  size_t error_index;
  size_t buf_len;

  switch (state_) {
    case State::RESPONDING:
      /* Send a minimal response */

      datagram[index++] = (uint8_t)(errors & 0xff);
      datagram[index++] = (uint8_t)((errors >> 8) & 0xff);

      datagram_len = index;
      state_ = State::IDLE;

      break;

    case State::RESPONDING_READ:
      /* Send a response with register contents */

      /* Save space for the error code */
      error_index = index;
      index += 2;

      server_->readRegisters(start_addr_, reg_count_, &datagram[index], buf_len, max_datagram_len - index, errors);
      index += buf_len;

      /* Copy error code into response */
      datagram[error_index++] = (uint8_t)(errors & 0xff);
      datagram[error_index++] = (uint8_t)((errors >> 8) & 0xff);

      datagram_len = index;
      state_ = State::IDLE;

      break;

    case State::RESPONDING_MEM:
      /* Send a response with memory contents */

      /* Save space for the error code */
      error_index = index;
      index += 2;

      if (max_datagram_len - index >= src_len_) {
        std::memcpy(&datagram[index], (void *)src_addr_, src_len_);
        index += src_len_;
      } else {
        /* Not enough space in response datagram */
        errors |= COMM_ERRORS_INVALID_ARGS;
      }

      /* Copy error code into response */
      datagram[error_index++] = (uint8_t)(errors & 0xff);
      datagram[error_index++] = (uint8_t)((errors >> 8) & 0xff);

      datagram_len = index;
      state_ = State::IDLE;

      break;

    case State::RESPONDING_U32:
      /* Send a response with a uint32_t value */

      datagram[index++] = (uint8_t)(errors & 0xff);
      datagram[index++] = (uint8_t)((errors >> 8) & 0xff);

      datagram[index++] = (uint8_t)(u32_value_ & 0xff);
      datagram[index++] = (uint8_t)((u32_value_ >> 8) & 0xff);
      datagram[index++] = (uint8_t)((u32_value_ >> 16) & 0xff);
      datagram[index++] = (uint8_t)((u32_value_ >> 24) & 0xff);

      datagram_len = index;
      state_ = State::IDLE;

      break;

    case State::RESPONDING_U8:
      /* Send a response with a uint8_t value */

      datagram[index++] = (uint8_t)(errors & 0xff);
      datagram[index++] = (uint8_t)((errors >> 8) & 0xff);

      datagram[index++] = u8_value_;

      datagram_len = index;
      state_ = State::IDLE;

      break;

    default:
      /* Don't send a response */

      datagram_len = 0;
      state_ = State::IDLE;

      break;
  }
}

template<typename T>
void handleVarAccess(T& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors) {
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

template void handleVarAccess<uint8_t>(uint8_t& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

template void handleVarAccess<uint16_t>(uint16_t& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

template void handleVarAccess<float>(float& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

template void handleVarAccess<int32_t>(int32_t& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

void startComms() {
  comms_endpoint.start();
}

void runComms() {
  comms_endpoint.receive(); // Blocks until packet is received

  if (!comms_endpoint.hasReceiveError()) {
    /* Received valid datagram */
    comm_errors_t errors = COMM_ERRORS_NONE;

    size_t receive_len = comms_endpoint.getReceiveLength();
    comms_protocol_fsm.handleRequest(comms_endpoint.getReceiveBufferPtr(), receive_len, errors);

    size_t transmit_len;
    comms_protocol_fsm.composeResponse(comms_endpoint.getTransmitBufferPtr(), transmit_len, comms_endpoint.getTransmitBufferSize(), errors);

    if (transmit_len > 0) {
      /* Send a response */
      comms_endpoint.setTransmitLength(transmit_len);
      comms_endpoint.transmit(); // Blocks until packet is fully transmitted
    }

    if (jump_addr != 0) {
      /* Jump to an address if requested */
      flashJumpApplication(jump_addr); // Does not return
    } else if (should_reset) {
      /* Reset system if requested */
      NVIC_SystemReset(); // Does not return
    }
  }
}

UARTEndpoint comms_endpoint(UARTD1, GPTD2, {GPIOD, GPIOD_RS485_DIR}, rs485_baud);

Server comms_server(*board_id_ptr, commsRegAccessHandler);

ProtocolFSM comms_protocol_fsm(comms_server);

} // namespace motor_driver
