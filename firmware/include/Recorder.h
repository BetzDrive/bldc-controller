#ifndef _RECORDER_H_
#define _RECORDER_H_
#include <stddef.h>
#include "constants.h"

constexpr size_t NUM_SAMPLES = 1000;

enum RecordState {
  READY,
  RECORDING,
  FINISHED
};

namespace motor_driver {


class Recorder {
public:
  Recorder(): cur_state_(READY), index_(0) {}
  bool startRecord();
  void recordSample(float* recorder_new_data);
  float* read();
  void reset();
  uint16_t size();

private:
  RecordState cur_state_;
  size_t index_;
  float record_buf_[NUM_SAMPLES * recorder_channel_count];
};

} // namespace motor_driver

#endif /* _RECORDER_H_ */
