#include "comms.h"

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "peripherals.h"
#include "state.h"
#include <cstring>

namespace motor_driver {

void UARTEndpoint::start() {
  uartStart(uart_driver_, &uart_config_);
  gptStart(gpt_driver_, &gpt_config_);

  chSysLock();

  state_ = State::INITIALIZING;
  gptStartOneShotI(gpt_driver_, idle_time_ticks_);

  chSysUnlock();
}

void UARTEndpoint::startTransmit() {
  tx_buf_[0] = (uint8_t)tx_len_;
  uint16_t crc = computeCRC(tx_buf_, tx_len_);
  tx_buf_[tx_len_ + 1] = crc & 0xff;
  tx_buf_[tx_len_ + 2] = (crc >> 8) & 0xff;

  chSysLock();

  if (state_ == State::IDLE) {
    palSetPad(dir_.port, dir_.pin);
    uartStartSendI(uart_driver_, 1 + tx_len_ + crc_length, tx_buf_);
    state_ = State::TRANSMITTING;
  }

  chSysUnlock();
}

void UARTEndpoint::waitReceive() {
  chBSemWait(&bsem_);

  uint16_t computed_crc = computeCRC(rx_buf_, rx_len_);
  uint16_t expected_crc = ((uint16_t)rx_buf_[rx_len_ + 1] << 8) | (uint16_t)rx_buf_[rx_len_];
  if (computed_crc != expected_crc) {
    rx_error_ = true;
  }
}

bool UARTEndpoint::pollReceive() {
  msg_t result = chBSemWaitTimeout(&bsem_, TIME_INFINITE);

  if (result == RDY_OK) {
    uint16_t computed_crc = computeCRC(rx_buf_, rx_len_);
    uint16_t expected_crc = ((uint16_t)rx_buf_[rx_len_ + 1] << 8) | (uint16_t)rx_buf_[rx_len_];
    if (computed_crc != expected_crc) {
      rx_error_ = true;
    }

    return true;
  } else {
    return false;
  }
}

uint8_t *UARTEndpoint::getReceiveBufferPtr() {
  return rx_buf_;
}

size_t UARTEndpoint::getReceiveLength() const {
  return rx_len_;
}

bool UARTEndpoint::hasReceiveError() const {
  return rx_error_;
}

uint8_t *UARTEndpoint::getTransmitBufferPtr() {
  return tx_buf_ + 1;
}

void UARTEndpoint::setTransmitLength(size_t len) {
  tx_len_ = len;
}

size_t UARTEndpoint::getTransmitBufferSize() const {
  return max_dg_payload_len;
}

void UARTEndpoint::uartTransmitCompleteCallback() {
  chSysLockFromIsr();

  palClearPad(dir_.port, dir_.pin);
  state_ = State::IDLE;

  chSysUnlockFromIsr();
}

void UARTEndpoint::uartReceiveCompleteCallback() {
  chSysLockFromIsr();

  switch (state_) {
    case State::RECEIVING:
    case State::RECEIVING_ERROR:
      /* Finished receiving a datagram */
      rx_error_ = (state_ == State::RECEIVING_ERROR);
      gptStopTimerI(gpt_driver_);
      chBSemSignalI(&bsem_);
      state_ = State::IDLE;
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
      /* Start of datagram */
      rx_len_ = (size_t)c;
      uartStartReceiveI(uart_driver_, rx_len_ + crc_length, rx_buf_);
      gptStartOneShotI(gpt_driver_, ((1 + rx_len_ + crc_length + 4) * 2) * 10);
      state_ = State::RECEIVING;
      break;
    default:
      break;
  }

  chSysUnlockFromIsr();
}

void UARTEndpoint::uartReceiveErrorCallback(uartflags_t e) {
  (void)e;

  chSysLockFromIsr();

  switch (state_) {
    case State::RECEIVING:
      state_ = State::RECEIVING_ERROR;
      break;
    default:
      break;
  }

  chSysUnlockFromIsr();
}

void UARTEndpoint::gptCallback() {
  chSysLockFromIsr();

  switch (state_) {
    case State::INITIALIZING:
      /* Bus has been idle for the required period of time */
      state_ = State::IDLE;
      break;
    case State::RECEIVING:
    case State::RECEIVING_ERROR:
      /* Timed out before receiving the expected number of bytes */
      uartStopReceiveI(uart_driver_);
      state_ = State::IDLE;
      break;
    default:
      break;
  }

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

  if (id != server_->getID()) {
    /* This datagram is not meant for us, ignore it */
    return;
  }

  /* Clear errors */
  errors = 0;

  switch (function_code_) {
    case COMM_FC_NOP:
      /* No operation */

      state_ = State::RESPONDING;

      break;

    case COMM_FC_READ_REGS:
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

    case COMM_FC_WRITE_REGS:
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

  datagram[index++] = server_->getID();
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

    default:
      /* Don't send a response */

      datagram_len = 0;
      state_ = State::IDLE;

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

static void commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, RegAccessType access_type, comm_errors_t& errors) {
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
      case 7:
        buf[index] = 0xdd;
        buf[index + 1] = 0xdd;
        handleVarAccess(results.encoder_angle, buf, index, buf_size, access_type, errors);
        break;
      default:
        errors |= COMM_ERRORS_INVALID_ADDR;
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

UARTEndpoint comms_endpoint(UARTD1, GPTD2, {GPIOD, GPIOD_RS485_DIR}, 115200);

Server comms_server(1, commsRegAccessHandler);

ProtocolFSM comms_protocol_fsm(comms_server);

void startComms() {
  comms_endpoint.start();
}

void runComms() {
  comms_endpoint.waitReceive();

  if (!comms_endpoint.hasReceiveError()) {
    /* Received valid datagram */
    comm_errors_t errors;

    size_t receive_len = comms_endpoint.getReceiveLength();
    comms_protocol_fsm.handleRequest(comms_endpoint.getReceiveBufferPtr(), receive_len, errors);

    size_t transmit_len;
    comms_protocol_fsm.composeResponse(comms_endpoint.getTransmitBufferPtr(), transmit_len, comms_endpoint.getTransmitBufferSize(), errors);

    if (transmit_len > 0) {
      comms_endpoint.setTransmitLength(transmit_len);
      comms_endpoint.startTransmit();
    }
  }
}

} // namespace motor_driver
