/* File: Buffer.h
 * Class meant to simplify adding data to a tx/rx buffer on embedded systems.
 * Operates similarly to stream operators.
 */
#include <cstring>
#include "stdint.h"

class Buffer {
  public:
    Buffer(size_t len) {
      max_len_ = len;
      head_ = 0, tail_ = 0;
      empty_ = true, full_ = false;
      buf_ = new uint8_t[max_len_];
    }

    // Operators for loading in data
    template <typename T>
    Buffer& operator<<(const T val);
    // Operators for unloading data
    template <typename T>
    Buffer& operator>>(T& val);

    // Jump over bytes we don't care about
    void jump (size_t dist);

  private:
    size_t max_len_;
    size_t head_;
    size_t tail_;
    uint8_t* buf_;
    bool full_, empty_;
};
