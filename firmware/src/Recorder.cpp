#include "Recorder.h"

namespace motor_driver {

bool Recorder::startRecord() {
  // TODO: start another record as soon as it is over, if already recording
  if (cur_state_ == READY) {
    cur_state_ = RECORDING;
    return true;
  }
  return false;
}

void Recorder::recordSample(float *recorder_new_data) {
  if (cur_state_ == RECORDING) {
    for (size_t j = 0; j < recorder_channel_count; j++) {
      record_buf_[index_ + j] = recorder_new_data[j];
    }
    index_ += recorder_channel_count;
    if (index_ >= NUM_SAMPLES * recorder_channel_count) {
      cur_state_ = FINISHED;
    }
  }
}

float* Recorder::read() {
  if (cur_state_ == FINISHED) {
    return record_buf_;
  } else {
    return nullptr;
  }
}

void Recorder::reset() {
  index_ = 0;
  cur_state_ = READY;
}

uint16_t Recorder::size() {
  if (cur_state_ == FINISHED) {
    return NUM_SAMPLES * recorder_channel_count;
  }
  return 0;
}

} // namespace motor_driver
