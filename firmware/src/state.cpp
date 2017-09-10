#include "state.h"

namespace motor_driver {

Results active_results;

Results sync_results;

Parameters active_parameters;

Parameters sync_parameters;

volatile bool should_copy_results = false;

volatile bool should_copy_parameters = false;

void initState() {
}

} // namespace motor_driver
