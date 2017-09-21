#ifndef _BL_COMMS_H_
#define _BL_COMMS_H_

#include "comms.h"

namespace motor_driver {

void runComms();

extern const uint8_t comms_id __attribute__((section(".id")));

} // namespace motor_driver

#endif /* _BL_COMMS_H_ */
