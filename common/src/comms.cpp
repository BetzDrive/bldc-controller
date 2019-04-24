#include "comms.h"

#include <cstring>
#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "constants.h"
#include "helper.h"
#include "flash.h"
#include "crc16.h"

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
  tx_buf_[1] = COMM_VERSION; // Protocol version
  tx_buf_[2] = COMM_FG_SEND; // Flag byte
  tx_buf_[3] = (tx_len_ + 2) & 0xff;
  tx_buf_[4] = ((tx_len_ + 2) >> 8) & 0xff;
  tx_buf_[5] = tx_len_ & 0xff;
  tx_buf_[6] = (tx_len_ >> 8) & 0xff;

  uint16_t crc = computeCRC(header_len + tx_buf_, tx_len_ + sub_msg_len);
  tx_buf_[header_len + sub_msg_len + tx_len_] = crc & 0xff;
  tx_buf_[header_len + sub_msg_len + tx_len_ + 1] = (crc >> 8) & 0xff;

  chSysLock();

  uartStopReceiveI(uart_driver_);
  palSetPad(dir_.port, dir_.pin);
  uartStartSendI(uart_driver_, header_len + sub_msg_len + tx_len_ + crc_len, tx_buf_);
  changeStateI(State::TRANSMITTING);

  chSysUnlock();

  chBSemWait(&tx_bsem_);
}

void UARTEndpoint::receive() {
  chBSemWait(&rx_bsem_);

  uint16_t computed_crc = computeCRC(rx_buf_ + header_len, rx_len_);
  uint16_t expected_crc = ((uint16_t)rx_buf_[header_len + rx_len_ + 1] << 8) | (uint16_t)rx_buf_[header_len + rx_len_];
  if (computed_crc != expected_crc) {
    rx_error_ = true;
  }
}

uint8_t UARTEndpoint::getFlags() {
  return rx_flags_;
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
  return tx_buf_ + header_len + sub_msg_len;
}

void UARTEndpoint::setTransmitLength(size_t len) {
  tx_len_ = len;
}

size_t UARTEndpoint::getTransmitBufferSize() const {
  return max_dg_payload_len;
}

void UARTEndpoint::changeStateI(State new_state) {
  gptStopTimerI(gpt_driver_);

  if (new_state == State::INITIALIZING ||
      new_state == State::RECEIVING_PROTOCOL_VERSION ||
      new_state == State::RECEIVING_LENGTH_L ||
      new_state == State::RECEIVING_LENGTH_H ||
      new_state == State::RECEIVING) {
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
      if (((uint8_t) c) == COMM_VERSION) {
        changeStateI(State::RECEIVING_FLAGS);
      } else {
        changeStateI(State::INITIALIZING);
      }
      break;
    case State::RECEIVING_FLAGS:
      /* Check protocol version */
      rx_buf_[2] = (uint8_t)c;
      rx_flags_ = (uint8_t)c;
      changeStateI(State::RECEIVING_LENGTH_L);
      break;
    case State::RECEIVING_LENGTH_L:
      /* Store lower byte of packet length */
      rx_buf_[3] = (uint8_t)c;
      changeStateI(State::RECEIVING_LENGTH_H);
      break;
    case State::RECEIVING_LENGTH_H:
      /* Store upper byte of packet length and start receiving data */
      rx_buf_[4] = (uint8_t)c;
      rx_len_ = ((size_t)rx_buf_[4] << 8) | rx_buf_[3];
      if (rx_len_ <= max_dg_payload_len) {
        uartStartReceiveI(uart_driver_, rx_len_ + crc_len, rx_buf_ + header_len);
        changeStateI(State::RECEIVING);
      } else {
        changeStateI(State::INITIALIZING);
      }
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
  crc16_t crc = crc16_init();
  crc = crc16_update(crc, buf, len);
  return crc16_finalize(crc);
}

/*          Server Functions            */
void Server::initDisco() {
  // Initialize disco bus to have output low according to spec
  palWritePad(disco_out_.port, disco_out_.pin, PAL_HIGH);
}

void Server::setDisco() {
  palWritePad(disco_out_.port, disco_out_.pin, PAL_LOW);
}

bool Server::getDisco() {
    return (PAL_LOW == palReadPad(disco_in_.port, disco_in_.pin));
}

/*       Protocol FSM Functions         */

void ProtocolFSM::handleRequest(uint8_t *datagram, size_t datagram_len, comm_fg_t flags, comm_errors_t& errors) {
  /* If message from another board, decrement counter and exit. */
  if (flags & COMM_FG_SEND) {
    resp_count_--;
    return;
  } else {
    resp_count_ = 0;
    state_ = State::IDLE;
  }

  static_assert(sizeof(comm_id_t) == 1, "Assuming comm_id_t is uint8_t");
  static_assert(sizeof(comm_fg_t) == 1, "Assuming comm_fg_t is uint8_t");
  static_assert(sizeof(comm_fc_t) == 1, "Assuming comm_fc_t is uint8_t");
  static_assert(sizeof(comm_addr_t) == 2, "Assuming comm_addr_t is uint16_t");
  static_assert(sizeof(comm_reg_count_t) == 1, "Assuming comm_reg_count_t is uint8_t");

  size_t index = 0, next_msg = 0;
  bool found_board = false;

  comm_id_t id;


  // Loop through full packet for the message intended for this board
  while ( ((datagram_len - index) >= sub_msg_header_len_) && !found_board ) {
    uint16_t sub_msg_len = (uint16_t)datagram[index] | ((uint16_t)datagram[index + 1] << 8);
    index += 2;
    next_msg += sub_msg_len + sizeof(sub_msg_len);

    id = datagram[index++];

    if (id != COMM_ID_BROADCAST && id != server_->getID()) {
      // We only wish to increment as long as we have not received our packet.
      if (state_ == State::IDLE) resp_count_++;

      index = next_msg;
      continue;
    }
    found_board = true;
    datagram_len = next_msg;
  }

  // None of the packets were meant for us.
  if (!found_board) {
    resp_count_ = 0;
    return;
  }

  function_code_ = datagram[index++];

  broadcast_ = (id == 0);

  /* Blink Com LED */
  if (activity_callback_ != nullptr) {
    activity_callback_();
  }

  /* Clear errors */
  errors = 0;

  /* Temporary variables */
  uint32_t sector_num, dest_addr;
  size_t dest_len;
  bool success;
  
  /* For Enumeration */
  uint8_t target_id;
#ifdef BOOTLOADER
  uint32_t id_addr;
#endif

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

    case COMM_FC_ENUMERATE:
      /* Enumerate board ID */
      target_id = (uint8_t)datagram[index++];

      if (server_->getID() == target_id) {
        u8_value_ = server_->getID();
        state_ = State::RESPONDING_U8;
      }

      /* Do nothing if not bootloader */
#ifdef BOOTLOADER
      // If the board has not been initialized yet (received an ID), check the state of the disco bus:
      //    if the input is high, pass through the command and set the id!
      //    if the input is low, ignore the command
      if (server_->getDisco() && (server_->getID() == 0)) {
        // Only update the flash if the IDs are different!
        success = true;
        if (target_id != *board_id_ptr && target_id != COMM_ID_BROADCAST) {
          // Write the ID to the board!
          id_addr = reinterpret_cast<uintptr_t>(board_id_ptr);
          success &= (flashErase(id_addr, sizeof(target_id)) == FLASH_RETURN_SUCCESS);
          success &= (flashWrite(id_addr, (char *)&target_id, sizeof(target_id)) == FLASH_RETURN_SUCCESS);
        }
        if (success) {
          server_->setID(target_id);
          server_->setDisco();
          u8_value_ = server_->getID();
          state_ = State::RESPONDING_U8;
        }
      }
#endif

      break;

    default:
      /* Invalid function code */

      errors |= COMM_ERRORS_INVALID_FC;
      state_ = State::RESPONDING;

      break;
  }
}

void ProtocolFSM::composeResponse(uint8_t *datagram, size_t& datagram_len, size_t max_datagram_len, comm_errors_t errors) {
  if (state_ == State::IDLE || resp_count_ != 0) {
    /* No response to send */
    datagram_len = 0;
    return;
  }

  static_assert(sizeof(comm_id_t) == 1, "Assuming comm_id_t is uint8_t");
  static_assert(sizeof(comm_fg_t) == 1, "Assuming comm_fg_t is uint8_t");
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
  size_t read_len;

  resp_count_ = 1; // Reset response counter

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

      read_len = server_->readRegisters(start_addr_, reg_count_, &datagram[index], max_datagram_len - index, errors);
      index += read_len;

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

template void handleVarAccess<int8_t>(int8_t& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

template void handleVarAccess<uint16_t>(uint16_t& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

template void handleVarAccess<uint32_t>(uint32_t& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

template void handleVarAccess<int32_t>(int32_t& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

template void handleVarAccess<float>(float& var, uint8_t *buf, size_t& index, size_t buf_size, RegAccessType access_type, comm_errors_t& errors);

void startComms() {
  comms_endpoint.start();
#ifdef BOOTLOADER
  comms_server.initDisco();
#else
  comms_server.setDisco();
#endif
}

void runComms() {
  comms_endpoint.receive(); // Blocks until packet is received

  if (!comms_endpoint.hasReceiveError()) {
    /* Received valid datagram */
    comm_errors_t errors = COMM_ERRORS_NONE;

    size_t receive_len = comms_endpoint.getReceiveLength();
    comms_protocol_fsm.handleRequest(comms_endpoint.getReceiveBufferPtr(), receive_len, comms_endpoint.getFlags(), errors);

    /* Wait for other boards */
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

#ifdef BOOTLOADER
// Wait in bootloader for a id to be assigned from the disco bus
Server comms_server(COMM_ID_BROADCAST, commsRegAccessHandler,
                  {GPIOB, GPIOB_DISCO_BUS_IN},
                  {GPIOB, GPIOB_DISCO_BUS_OUT});
#else
// Out of the bootloader, use whatever ID is stored in memory
Server comms_server(*board_id_ptr, commsRegAccessHandler,
                  {GPIOB, GPIOB_DISCO_BUS_IN},
                  {GPIOB, GPIOB_DISCO_BUS_OUT});
#endif
ProtocolFSM comms_protocol_fsm(comms_server);

} // namespace motor_driver
