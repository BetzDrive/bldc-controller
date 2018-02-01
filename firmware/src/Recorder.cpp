#include "Recorder.h"

namespace motor_driver {

bool Recorder::readyToRead() {
  return readReady;
}

bool Recorder::startRecord() {
  // TODO: start another record as soon as it is over, if already recording
  if (!readReady) {
    recording = true;
    return true;
  }
  return false;
}

void Recorder::recordSample(float *recorder_new_data) {
  if (recording && !readReady) {
    for (int j = 0; j < 8; j++) {
      // bad caching?
      buf[j*RECORD_BUF_SIZE + index] = recorder_new_data[j];
    }
    index++;
    if (index >= RECORD_BUF_SIZE) {
      index = 0;
      readReady = true;
      recording = false;
    }
  }
}

float *Recorder::read() {
  if (readReady) {
    readReady = false;
    return buf;
  } else {
    return nullptr;
  }
}

} // namespace motor_driver
