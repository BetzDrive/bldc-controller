#include "serial.hpp"

bool SerialDriver::findNonce() {
  const uint8_t* buf_ptr;
  size_t bytes_available;
  uint8_t nonce_left = config_.nonce_length - nonce_index_;
  int cmp_result;

  ptr = getBytes(nonce_left, bytes_avaiable);
  cmp_result = memcmp(ptr, config_.nonce + nonce_index, bytes_available);
  if (0 == cmp_result) {
    if (bytes_availabe = nonce_left) {
      state_ = state_t::get_header;
      return true;
    } else {
      nonce_index += bytes_available;
      return false;
    }
  }
  else {
    nonce_index_ = 0;
    return false;
  }
}

bool SerialDriver::getPacket() {
  
}
