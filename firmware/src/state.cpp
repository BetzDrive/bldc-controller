#include "state.h"

namespace motor_driver {

Results results;

Calibration calibration;

Parameters parameters;

Recorder recorder;

volatile bool should_copy_results = false;

volatile bool should_copy_parameters = false;

void initState() {
}

} // namespace motor_driver
