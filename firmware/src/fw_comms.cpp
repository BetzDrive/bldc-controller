#include "fw_comms.h"

#include "ch.h"
#include "hal.h"
#include "peripherals.h"
#include "state.h"
#include <cstring>

namespace motor_driver {

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
    case COMM_FC_READ_REGS_SYNCED:
      /* Read registers */

      if (datagram_len - index < 3) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      start_addr_ = (comm_addr_t)datagram[index++];
      start_addr_ |= (comm_addr_t)datagram[index++] << 8;
      reg_count_ = datagram[index++];

      synced_ = (function_code_ == COMM_FC_READ_REGS_SYNCED);

      state_ = State::RESPONDING_READ;

      break;

    case COMM_FC_WRITE_REGS:
    case COMM_FC_WRITE_REGS_SYNCED:
      /* Write registers */

      if (datagram_len - index < 3) {
        errors |= COMM_ERRORS_MALFORMED;
        state_ = State::RESPONDING;
        break;
      }

      start_addr_ = (comm_addr_t)datagram[index++];
      start_addr_ |= (comm_addr_t)datagram[index++] << 8;
      reg_count_ = datagram[index++];

      synced_ = (function_code_ == COMM_FC_WRITE_REGS_SYNCED);

      server_->writeRegisters(start_addr_, reg_count_, &datagram[index], datagram_len - index, errors, synced_);

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

void commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count, uint8_t *buf, size_t& buf_len, size_t buf_size, RegAccessType access_type, comm_errors_t& errors, bool synced) {
  size_t index = 0;

  Results& results = synced ? sync_results : active_results;
  Parameters& parameters = synced ? sync_parameters : active_parameters;

  for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
    switch (addr) {
      case 0x0100:
        handleVarAccess(results.encoder_angle, buf, index, buf_size, access_type, errors);
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
  }
}

} // namespace motor_driver
