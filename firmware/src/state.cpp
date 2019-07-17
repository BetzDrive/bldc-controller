#include "state.h"

#include "flash.h"
#include <cstring>

namespace motor_driver {
namespace state {

Results results;

Calibration calibration;

Parameters parameters;

Recorder recorder;

volatile bool should_copy_results = false;

volatile bool should_copy_parameters = false;

void initState() {
}

void storeCalibration() {
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);
  flashWrite(addr, (char *)&state::calibration, sizeof(state::Calibration));
}

void loadCalibration() {
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);
  uint16_t start_sequence = 0;
  // Retry loading flash until successful load. This is critical to read.
  while (not (flashRead(addr, (char *)&start_sequence, sizeof(uint16_t)) == FLASH_RETURN_SUCCESS));
  if (start_sequence == consts::calib_ss) {
    while (not (flashRead(addr, (char *)&state::calibration, sizeof(state::Calibration)) == FLASH_RETURN_SUCCESS));
  }
}

void clearCalibration() {
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);
  flashErase(addr, sizeof(state::Calibration));

  // Copy default values into calibration.
  state::Calibration temp_calib;
  std::memcpy(&state::calibration, &temp_calib, sizeof(state::Calibration));
}

} // namespace state
} // namespace motor_driver
