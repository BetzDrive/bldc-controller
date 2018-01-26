#ifndef _RECORDER_H_
#define _RECORDER_H_

namespace motor_driver {

constexpr size_t RECORD_BUF_SIZE = 1000;

class Recorder {
public:
  Recorder() {
    readReady = false;
    recording = false;
    index = 0;
  }
  bool readyToRead();
  void startRecord();
  void recordSample();
  float *read();

private:
  bool readReady;
  bool recording;
  size_t index;
  float buf[RECORD_BUF_SIZE * 8];
};

} // namespace motor_driver

#endif /* _RECORDER_H_ */
