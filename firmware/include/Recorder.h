#ifndef _RECORDER_H_
#define _RECORDER_H_

#include <stddef.h>
#include "constants.h"

namespace motor_driver {

class Recorder {
public:
  Recorder() : cur_state_(State::READY), index_(0) {}

  bool startRecord();

  void recordSample(float* recorder_new_data);

  float* read();

  void reset();

  uint16_t size();

private:
  enum class State {
    READY,
    RECORDING,
    FINISHED
  };

  State cur_state_;
  size_t index_;
  float record_buf_[recorder_max_samples * recorder_channel_count];
};

} // namespace motor_driver

#endif /* _RECORDER_H_ */
