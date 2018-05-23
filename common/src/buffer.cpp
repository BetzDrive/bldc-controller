#include <cstring>
#include "stdint.h"
#include "buffer.h"

template <typename T>
Buffer& Buffer::operator<<(const T val) {
  for (int i = 0; i < sizeof(T); i++) {
    if (full_)
      throw 0;

    head_ %= max_len_;                         // Circular buffer :)
    buf_[head_++] = (uint8_t)(val >> (8 * i)); // Shifting for each byte
    empty_ = false;

    if (head_ == tail_)
      full_ = true;
  }
  return *this;
}

template <typename T>
Buffer& Buffer::operator>>(T& val) {
  for (int i = 0; i < sizeof(T); i++) {
    if (empty_)
      throw 0;

    tail_ %= max_len_;
    val = val | (((T) buf_[tail_++]) << (8 * i));
    full_ = false;

    if (tail_ == head_)
      empty_ = true;
  }
  return *this; 
}

void Buffer::jump (size_t dist) {
  // Check to make sure we don't pass head.
  if ((tail_ + dist) % max_len_ < head_ || 
      ((tail_ > head_) && (tail_ + dist) % max_len_ > head_) ||
      (full_ && dist < max_len_))
    tail_ = tail_ + dist % max_len_;
  else
    tail_ = head_;
}

/*
// Test code expected to print out:
// d
// 10000
// 100000000
#include <iostream>
int main () {
  Buffer rx(1024);
  rx << (uint8_t) 100 << (uint16_t) 10000 << (uint32_t) 100000000;
  uint8_t a = 0;
  uint16_t b = 0;
  uint32_t c = 0;
  rx >> a >> b >> c;
  std::cout << a << std::endl << b << std::endl << c << std::endl;
}
*/
