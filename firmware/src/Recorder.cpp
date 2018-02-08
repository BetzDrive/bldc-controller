#include "Recorder.h"

namespace motor_driver {

bool Recorder::startRecord() {
  // TODO: start another record as soon as it is over, if already recording
  if (cur_state_ == State::READY) {
    cur_state_ = State::RECORDING;
    return true;
  }
  return false;
}

void Recorder::recordSample(float *recorder_new_data) {
  if (cur_state_ == State::RECORDING) {
    for (size_t j = 0; j < recorder_channel_count; j++) {
      record_buf_[index_ + j] = recorder_new_data[j];
    }
    index_ += recorder_channel_count;
    if (index_ >= recorder_max_samples * recorder_channel_count) {
      cur_state_ = State::FINISHED;
    }
  }
}

float *Recorder::read() {
  if (cur_state_ == State::FINISHED) {
    return record_buf_;
  } else {
    return nullptr;
  }
}

void Recorder::reset() {
  index_ = 0;
  cur_state_ = State::READY;
}

uint16_t Recorder::size() {
  if (cur_state_ == State::FINISHED) {
    return recorder_max_samples * recorder_channel_count;
  } else {
    return 0;
  }
}

} // namespace motor_driver
