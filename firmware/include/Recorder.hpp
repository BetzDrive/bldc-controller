#ifndef RECORDER_HPP_
#define RECORDER_HPP_

#include <stddef.h>

#include "constants.hpp"

namespace motor_driver {
namespace state {

class Recorder {
public:
  Recorder() : state_(State::READY), index_(0) {}

  bool startRecording();

  void recordSample(float *recorder_new_data);

  float *read();

  void reset();

  uint16_t size();

private:
  enum class State { READY, RECORDING, FINISHED };

  State state_;
  size_t index_;
  float record_buf_[(consts::recorder_max_samples *
                     consts::recorder_channel_count)];
};

} // namespace state
} // namespace motor_driver

#endif // RECORDER_HPP_
