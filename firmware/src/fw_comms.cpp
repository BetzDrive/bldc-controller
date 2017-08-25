#include "fw_comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include <cstring>

namespace motor_driver {

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
