#include "state.hpp"

#include <cstring>

#include "flash.h"
#include "helper.h"

namespace motor_driver {
namespace state {

Results results;

Calibration calibration;

Parameters parameters;

Recorder recorder;

volatile bool should_copy_results = false;

volatile bool should_copy_parameters = false;

void initState() {}

void storeCalibration() {
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);

  struct IWDG_Values save = pauseIWDG();
  flashErase(addr, sizeof(state::Calibration));
  resumeIWDG(save);

  flashWrite(addr, reinterpret_cast<char *>(&state::calibration),
             sizeof(state::Calibration));
}

void loadCalibration() {
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);
  uint16_t start_sequence = 0;
  // Retry loading flash until successful load. This is critical to read.
  while (!(flashRead(addr, reinterpret_cast<char *>(&start_sequence),
                     sizeof(uint16_t)) == FLASH_RETURN_SUCCESS)) {
  }
  if (start_sequence == consts::calib_ss) {
    while (!(flashRead(addr, reinterpret_cast<char *>(&state::calibration),
                       sizeof(state::Calibration)) == FLASH_RETURN_SUCCESS)) {
    }
  }
}

void clearCalibration() {
  // Copy default values into calibration.
  state::Calibration temp_calib;
  std::memcpy(&state::calibration, &temp_calib, sizeof(state::Calibration));
}

} // namespace state
} // namespace motor_driver
