#ifndef _COMMS_H_
#define _COMMS_H_

#include "hal.h"

#include "chmtx.h"
#include "comms_defs.h"
#include "utils.h"

namespace motor_driver {
namespace peripherals {
extern Mutex var_access_mutex;
}
namespace comms {

class UARTEndpoint;

struct UARTEndpointUARTConfig : UARTConfig {
  UARTEndpoint *endpoint;
};

struct UARTEndpointGPTConfig : GPTConfig {
  UARTEndpoint *endpoint;
};

class UARTEndpoint {
public:
  static constexpr size_t header_len = 5;
  static constexpr size_t sub_msg_len = 2;
  static constexpr size_t crc_len = 2;
  static constexpr size_t max_dg_payload_len = 255;

  UARTEndpoint(UARTDriver &uart_driver, GPTDriver &gpt_driver, IOPin dir,
               uint32_t baud)
      : uart_driver_(&uart_driver), gpt_driver_(&gpt_driver), dir_(dir),
        state_(State::STOPPED) {
    uart_config_.txend2_cb = uartTransmitCompleteCallbackStatic;
    uart_config_.rxend_cb = uartReceiveCompleteCallbackStatic;
    uart_config_.rxchar_cb = uartCharReceivedCallbackStatic;
    uart_config_.rxerr_cb = uartReceiveErrorCallbackStatic;
    uart_config_.speed = baud;
    uart_config_.endpoint = this;

    // Timeouts are measured in microseconds.
    gpt_config_.frequency = 1000000;
    gpt_config_.callback = gptCallbackStatic;
    gpt_config_.endpoint = this;

    // 5 ms.
    idle_time_ticks_ = 5000;

    chBSemInit(&rx_bsem_, 0);
    chBSemInit(&tx_bsem_, 0);
  }

  void start();

  void transmit();

  void receive();

  uint8_t getFlags();

  uint8_t *getReceiveBufferPtr();

  size_t getReceiveLength() const;

  bool hasReceiveError() const;

  uint8_t *getTransmitBufferPtr();

  void setTransmitLength(size_t len);

  size_t getTransmitBufferSize() const;

private:
  enum class State {
    // Inactive, not responding to any requests.
    STOPPED,
    // Waiting for bus to become idle.
    INITIALIZING,
    // Waiting for sync flag.
    IDLE,
    // Waiting for protocol version byte.
    RECEIVING_PROTOCOL_VERSION,
    // Waiting for flags bits.
    RECEIVING_FLAGS,
    // Waiting for lower byte of length word.
    RECEIVING_LENGTH_L,
    // Waiting for upper byte of length word.
    RECEIVING_LENGTH_H,
    // Receiving data.
    RECEIVING,
    // Transmitting data.
    TRANSMITTING
  };

  UARTDriver *const uart_driver_;
  GPTDriver *const gpt_driver_;
  const IOPin dir_;
  UARTEndpointUARTConfig uart_config_;
  UARTEndpointGPTConfig gpt_config_;
  State state_;
  gptcnt_t idle_time_ticks_;
  BinarySemaphore rx_bsem_;
  BinarySemaphore tx_bsem_;

  // Receive DMA buffer.
  uint8_t rx_buf_[header_len + max_dg_payload_len + crc_len];
  size_t rx_len_;
  bool rx_error_;
  comm_fg_t rx_flags_;

  // Transmit DMA buffer.
  uint8_t tx_buf_[header_len + max_dg_payload_len + crc_len];
  size_t tx_len_;

  static uint16_t computeCRC(const uint8_t *buf, size_t len);

  static void uartTransmitCompleteCallbackStatic(UARTDriver *uartp) {
    ((UARTEndpointUARTConfig *)uartp->config)
        ->endpoint->uartTransmitCompleteCallback();
  }

  static void uartReceiveCompleteCallbackStatic(UARTDriver *uartp) {
    ((UARTEndpointUARTConfig *)uartp->config)
        ->endpoint->uartReceiveCompleteCallback();
  }

  static void uartCharReceivedCallbackStatic(UARTDriver *uartp, uint16_t c) {
    ((UARTEndpointUARTConfig *)uartp->config)
        ->endpoint->uartCharReceivedCallback(c);
  }

  static void uartReceiveErrorCallbackStatic(UARTDriver *uartp, uartflags_t e) {
    ((UARTEndpointUARTConfig *)uartp->config)
        ->endpoint->uartReceiveErrorCallback(e);
  }

  static void gptCallbackStatic(GPTDriver *gptp) {
    ((UARTEndpointGPTConfig *)gptp->config)->endpoint->gptCallback();
  }

  void changeStateI(State new_state);

  void uartTransmitCompleteCallback();

  void uartReceiveCompleteCallback();

  void uartCharReceivedCallback(uint16_t c);

  void uartReceiveErrorCallback(uartflags_t e);

  void gptCallback();
};

enum class RegAccessType { READ, WRITE };

using RegAccessHandler = size_t (*)(comm_addr_t start_addr, size_t reg_count,
                                    uint8_t *buf, size_t buf_size,
                                    RegAccessType access_type,
                                    comm_errors_t &errors);

class Server {
public:
  Server(comm_id_t id, RegAccessHandler access_handler, IOPin disco_in,
         IOPin disco_out)
      : id_(id), access_handler_(access_handler), disco_in_(disco_in),
        disco_out_(disco_out) {}

  comm_id_t getID() const { return id_; }

  void setID(comm_id_t id) { id_ = id; }

  // Initialize the disco bus according to spec
  void initDisco();
  // Set up following board to receive id
  void setDisco();
  // Check the state of our disco input
  bool getDisco();

  size_t readRegisters(comm_addr_t start_addr, size_t reg_count, uint8_t *buf,
                       size_t buf_size, comm_errors_t &errors) {
    return access_handler_(start_addr, reg_count, buf, buf_size,
                           RegAccessType::READ, errors);
  }

  size_t writeRegisters(comm_addr_t start_addr, size_t reg_count, uint8_t *buf,
                        size_t buf_size, comm_errors_t &errors) {
    return access_handler_(start_addr, reg_count, buf, buf_size,
                           RegAccessType::WRITE, errors);
  }

private:
  comm_id_t id_;
  RegAccessHandler access_handler_;
  // Disco Bus Pins
  const IOPin disco_in_, disco_out_;
};

class ProtocolFSM {
public:
  static constexpr size_t sub_msg_header_len_ = 4;

  ProtocolFSM(Server &server) : server_(&server) {
    state_ = State::IDLE;
    resp_count_ = 1;
  }

  void handleRequest(uint8_t *datagram, size_t datagram_len, comm_fg_t flags,
                     comm_errors_t &errors);

  void composeResponse(uint8_t *datagram, size_t &datagram_len,
                       size_t max_datagram_len, comm_errors_t errors);

  void setActivityCallback(void (*activity_callback)()) {
    activity_callback_ = activity_callback;
  }

  uint8_t getRespCount() const { return resp_count_; }

private:
  enum class State {
    IDLE,
    RESPONDING,
    RESPONDING_READ,
    RESPONDING_MEM,
    RESPONDING_U32,
    RESPONDING_U8
  };

  Server *const server_;
  State state_;
  comm_fc_t function_code_;
  comm_errors_t errors_;
  comm_addr_t start_addr_;
  size_t reg_count_;
  uint32_t u32_value_;
  uint8_t u8_value_;
  uint32_t src_addr_;
  size_t src_len_;
  bool broadcast_;
  void (*activity_callback_)() = nullptr;
  uint8_t resp_count_;
};

template <typename T>
void handleVarAccess(T &var, uint8_t *buf, size_t &index, size_t buf_size,
                     RegAccessType access_type, comm_errors_t &errors);

size_t commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count,
                             uint8_t *buf, size_t buf_size,
                             RegAccessType access_type, comm_errors_t &errors);

void startComms();

void runComms();

void setWDGTimeout();

void clearWDGTimeout();

extern UARTEndpoint comms_endpoint;

extern Server comms_server;

extern ProtocolFSM comms_protocol_fsm;

} // namespace comms
} // namespace motor_driver

#endif // _COMMS_H_.
