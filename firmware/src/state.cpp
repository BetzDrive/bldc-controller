#include "state.h"

namespace motor_driver {

Results results;

Results sync_results;

Parameters parameters;

Parameters sync_parameters;

Mutex state_mutex;

void initState() {
    chMtxInit(&state_mutex);
}

} // namespace motor_driver
