#ifndef _RECORDER_H_
#define _RECORDER_H_

namespace motor_driver {

constexpr unsigned int RECORD_BUF_SIZE = 1000;

class Recorder {
public:
  Recorder() {
    readReady = false;
    recording = false;
    index = 0;
  }
  bool readyToRead();
  bool startRecord();
  void recordSample(float* recorder_new_data);
  float *read();

private:
  bool readReady;
  bool recording;
  unsigned int index;
  float buf[RECORD_BUF_SIZE * 8];
};

} // namespace motor_driver

#endif /* _RECORDER_H_ */
