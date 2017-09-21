#ifndef _COMMS_H_
#define _COMMS_H_

#include "hal.h"
#include "utils.h"
#include "comms_defs.h"

namespace motor_driver {

class UARTEndpoint;

struct UARTEndpointUARTConfig : UARTConfig {
  UARTEndpoint *endpoint;
};

struct UARTEndpointGPTConfig : GPTConfig {
  UARTEndpoint *endpoint;
};

class UARTEndpoint {
public:
  static constexpr size_t crc_length = 2;
  static constexpr size_t max_dg_payload_len = 255;

  UARTEndpoint(UARTDriver& uart_driver, GPTDriver& gpt_driver, IOPin dir, uint32_t baud)
    : uart_driver_(&uart_driver),
      gpt_driver_(&gpt_driver),
      dir_(dir),
      state_(State::STOPPED) {
    uart_config_.txend2_cb = uartTransmitCompleteCallbackStatic;
    uart_config_.rxend_cb = uartReceiveCompleteCallbackStatic;
    uart_config_.rxchar_cb = uartCharReceivedCallbackStatic;
    uart_config_.rxerr_cb = uartReceiveErrorCallbackStatic;
    uart_config_.speed = baud;
    uart_config_.endpoint = this;

    gpt_config_.frequency = baud * 2;
    gpt_config_.callback = gptCallbackStatic;
    gpt_config_.endpoint = this;

    idle_time_ticks_ = 7 * 10; // 3.5 character times at 10 bits/character

    chBSemInit(&bsem_, 0);
  }

  void start();

  void startTransmit();

  void waitReceive();

  bool pollReceive();

  uint8_t *getReceiveBufferPtr();

  size_t getReceiveLength() const;

  bool hasReceiveError() const;

  uint8_t *getTransmitBufferPtr();

  void setTransmitLength(size_t len);

  size_t getTransmitBufferSize() const;

private:
  enum class State {
    STOPPED,
    INITIALIZING,
    IDLE,
    RECEIVING,
    RECEIVING_ERROR,
    TRANSMITTING
  };

  UARTDriver * const uart_driver_;
  GPTDriver * const gpt_driver_;
  const IOPin dir_;
  UARTEndpointUARTConfig uart_config_;
  UARTEndpointGPTConfig gpt_config_;
  State state_;
  gptcnt_t idle_time_ticks_;
  BinarySemaphore bsem_;
  uint8_t rx_buf_[max_dg_payload_len + crc_length];
  size_t rx_len_;
  bool rx_error_;
  uint8_t tx_buf_[1 + max_dg_payload_len + crc_length];
  size_t tx_len_;

  static uint16_t computeCRC(const uint8_t *buf, size_t len);

  static void uartTransmitCompleteCallbackStatic(UARTDriver *uartp) {
    ((UARTEndpointUARTConfig *)uartp->config)->endpoint->uartTransmitCompleteCallback();
  }

  static void uartReceiveCompleteCallbackStatic(UARTDriver *uartp) {
    ((UARTEndpointUARTConfig *)uartp->config)->endpoint->uartReceiveCompleteCallback();
  }

  static void uartCharReceivedCallbackStatic(UARTDriver *uartp, uint16_t c) {
    ((UARTEndpointUARTConfig *)uartp->config)->endpoint->uartCharReceivedCallback(c);
  }

  static void uartReceiveErrorCallbackStatic(UARTDriver *uartp, uartflags_t e) {
    ((UARTEndpointUARTConfig *)uartp->config)->endpoint->uartReceiveErrorCallback(e);
  }

  static void gptCallbackStatic(GPTDriver *gptp) {
    ((UARTEndpointGPTConfig *)gptp->config)->endpoint->gptCallback();
  }

  void uartTransmitCompleteCallback();

  void uartReceiveCompleteCallback();

  void uartCharReceivedCallback(uint16_t c);

  void uartReceiveErrorCallback(uartflags_t e);

  void gptCallback();
};

enum class RegAccessType {
  READ,
  WRITE
};

using RegAccessHandler = void (*)(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, RegAccessType access_type, comm_errors_t& errors, bool synced);

class Server {
public:
  Server(uint8_t id, RegAccessHandler access_handler) : id_(id), access_handler_(access_handler) {}

  uint8_t getID() const {
    return id_;
  }

  void setID(uint8_t id) {
    id_ = id;
  }

  void readRegisters(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, comm_errors_t& errors, bool synced) {
    access_handler_(start_addr, reg_count, buf, buf_len, buf_size, RegAccessType::READ, errors, synced);
  }

  void writeRegisters(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t buf_len, comm_errors_t& errors, bool synced) {
    access_handler_(start_addr, reg_count, buf, buf_len, 0, RegAccessType::WRITE, errors, synced);
  }

private:
  uint8_t id_;
  RegAccessHandler access_handler_;
};

class ProtocolFSM {
public:
  ProtocolFSM(Server& server) : server_(&server) {
    state_ = State::IDLE;
  }

  void handleRequest(uint8_t *datagram, size_t datagram_len, comm_errors_t& errors);

  void composeResponse(uint8_t *datagram, size_t& datagram_len, size_t max_datagram_len, comm_errors_t errors);

private:
  enum class State {
    IDLE,
    RESPONDING,
    RESPONDING_READ,
    RESPONDING_MEM,
    RESPONDING_U32
  };

  Server * const server_;
  State state_;
  comm_fc_t function_code_;
  comm_errors_t errors_;
  comm_addr_t start_addr_;
  size_t reg_count_;
  uint32_t u32_value_;
  uint32_t src_addr_;
  size_t src_len_;
  bool synced_;
};

void commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, RegAccessType access_type, comm_errors_t& errors, bool synced);

void startComms();

extern UARTEndpoint comms_endpoint;

extern Server comms_server;

extern ProtocolFSM comms_protocol_fsm;

extern const uint8_t *comms_id_ptr;

} // namespace motor_driver

#endif /* _COMMS_H_ */
