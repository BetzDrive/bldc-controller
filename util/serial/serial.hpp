#pragma once

/* This driver manages searching for a serial packet from within a given
 * buffer. The design is to be robust to the following failure modes:
 *  1. Packet is partially sent.
 *  2. We begin processing in the middle of a packet and see an invalid nonce.
 *  3. We see
 */
class SerialDriver() {
public:
  struct config_t {
    uint8_t* buffer_ptr;
    uint8_t* nonce;
    uint8_t nonce_length;
    :
  };

  SerialDriver(const config_t config): config_(config) {}

  // Returns a pointer to the start of the available bytes. Available bytes is
  // set to the number of bytes available up to num_bytes.
  const uint8_t* getBytes(size_t num_bytes, size_t& available_bytes);

private:
  enum state_t {
    find_nonce = 0,
    get_header,
  };

  // Returns false if no nonce found in the current buffer. Else, buffer index
  // will be set to the latest occurence of a nonce.
  bool findNonce();

  const config_t config_;
  state_t state_;
  uint8_t nonce_index_;
};
