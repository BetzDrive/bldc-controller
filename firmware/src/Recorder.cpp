#include "Recorder.h"

namespace motor_driver {

bool Recorder::startRecording() {
  // TODO: start another record as soon as it is over, if already recording
  if (state_ == State::READY) {
    state_ = State::RECORDING;
    return true;
  }
  return false;
}

void Recorder::recordSample(float *recorder_new_data) {
  if (state_ == State::RECORDING) {
    for (size_t j = 0; j < recorder_channel_count; j++) {
      record_buf_[index_ + j] = recorder_new_data[j];
    }
    index_ += recorder_channel_count;
    if (index_ >= recorder_max_samples * recorder_channel_count) {
      state_ = State::FINISHED;
    }
  }
}

float *Recorder::read() {
  if (state_ == State::FINISHED) {
    return record_buf_;
  } else {
    return nullptr;
  }
}

void Recorder::reset() {
  index_ = 0;
  state_ = State::READY;
}

uint16_t Recorder::size() {
  if (state_ == State::FINISHED) {
    return recorder_max_samples * recorder_channel_count;
  } else {
    return 0;
  }
}

} // namespace motor_driver
