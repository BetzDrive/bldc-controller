#include "state.h"

namespace motor_driver {

Results results;

RolledADC rolladc;

Calibration calibration;

Parameters parameters;

Recorder recorder;

volatile bool should_copy_results = false;

volatile bool should_copy_parameters = false;

void initState() {
}

} // namespace motor_driver
