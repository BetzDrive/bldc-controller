#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IIS328DQ accelerometer (I2C address 0x18) */
#define ACC_I2C_ADDR       0x18
#define ACC_WHO_AM_I       0x0F
#define ACC_WHO_AM_I_VAL   0x32
#define ACC_CTRL_REG1      0x20
#define ACC_CTRL_REG4      0x23
#define ACC_OUT_X_L        0x28
#define ACC_AUTO_INC       0x80

/* MCP9808 temperature sensor (I2C address 0x19) */
#define TEMP_I2C_ADDR      0x19
#define TEMP_AMBIENT_REG   0x05

/* Reset sensor state machine. Call before entering the main loop.
 * Actual sensor configuration happens incrementally via sensor_step(). */
void sensor_init(void);

/* Non-blocking sensor state machine. Call every main loop iteration.
 * Returns true when a complete read cycle (accel + temp) finishes. */
bool sensor_step(void);

/* Check accelerometer WHO_AM_I. Returns true if present.
 * Note: uses blocking I2C — only call during startup, not in the main loop. */
bool acc_check_id(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H */
