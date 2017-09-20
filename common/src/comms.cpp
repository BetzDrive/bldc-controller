#include "comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "constants.h"
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

      server_->readRegisters(start_addr_, reg_count_, &datagram[index], buf_len, max_datagram_len - index, errors, synced_);
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

    default:
      /* Don't send a response */

      datagram_len = 0;
      state_ = State::IDLE;

      break;
  }
}

void startComms() {
  comms_endpoint.start();
}

UARTEndpoint comms_endpoint(UARTD1, GPTD2, {GPIOD, GPIOD_RS485_DIR}, rs485_baud);

Server comms_server(2, commsRegAccessHandler);

ProtocolFSM comms_protocol_fsm(comms_server);

} // namespace motor_driver
