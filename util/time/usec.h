#pragma once

#include "hal.h"

namespace time {

inline uint32_t msecISR() {
  return ST2MS(chTimeNow());
}

// Copied from https://stackoverflow.com/a/71784908
// As a 32 bit value, this only counts up to 4294 seconds before rolling over.
// As such, this has been extended to a 64 bit value.
inline uint64_t usecISR() {
  uint64_t ms;
  uint64_t st;

  // Read UptimeMillis and SysTick->VAL until
  // UptimeMillis doesn't rollover.
  do {
    ms = msecISR();
    st = SysTick->VAL;
  } while (ms != msecISR());
  // NOTE(greg): The original implementation uses 32 bit values. This
  // implementation prevents the discarding of the upper ~10 bits when
  // multiplying ms by 1000..
  return ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);
}

inline float usecToSeconds(uint64_t usecs) {
  return (float)usecs / 1000000.0f;
}
}  // namespace time
