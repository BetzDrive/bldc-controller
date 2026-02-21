/*
 * Sensor polling logic for accelerometer and temperature sensor.
 * Non-blocking state machine using async I2C.
 * Platform-independent - uses only hal_i2c and hal_timer.
 */

#include "sensor.h"

#include "hal/hal_i2c.h"
#include "hal/hal_timer.h"
#include "baremetal_config.h"
#include "state.h"

#define SENSOR_MAX_INIT_ERRORS 3

typedef enum {
    SENSOR_UNINIT,
    SENSOR_INIT_WAIT_1,
    SENSOR_INIT_WAIT_2,
    SENSOR_IDLE,
    SENSOR_WAIT_ACCEL,
    SENSOR_WAIT_TEMP,
} sensor_state_t;

static sensor_state_t state;
static uint32_t last_poll_ms;
static uint8_t init_error_count;

/* Static buffers for async I2C transfers (must persist across calls) */
static const uint8_t init_ctrl1[] = {ACC_CTRL_REG1, 0x27};
static const uint8_t init_ctrl4[] = {ACC_CTRL_REG4, 0x80};
static uint8_t accel_reg;
static uint8_t accel_buf[6];
static uint8_t temp_reg;
static uint8_t temp_buf[2];

void sensor_init(void) {
    state = SENSOR_UNINIT;
    last_poll_ms = 0;
    init_error_count = 0;
}

bool acc_check_id(void) {
    static uint8_t reg;
    static uint8_t id;
    reg = ACC_WHO_AM_I;
    id = 0;
    if (!hal_i2c_start_write_read(ACC_I2C_ADDR, &reg, 1, &id, 1))
        return false;
    uint32_t start = hal_timer_usec();
    while (hal_i2c_busy()) {
        if ((hal_timer_usec() - start) > 2000)
            return false;
    }
    if (hal_i2c_error() != HAL_I2C_OK)
        return false;
    return id == ACC_WHO_AM_I_VAL;
}

bool sensor_step(void) {
    switch (state) {
    case SENSOR_UNINIT:
        if (hal_i2c_start_write(ACC_I2C_ADDR, init_ctrl1, sizeof(init_ctrl1))) {
            state = SENSOR_INIT_WAIT_1;
        }
        return false;

    case SENSOR_INIT_WAIT_1:
        if (hal_i2c_busy()) return false;
        if (hal_i2c_error() != HAL_I2C_OK) {
            if (++init_error_count >= SENSOR_MAX_INIT_ERRORS) {
                state = SENSOR_IDLE;
                last_poll_ms = hal_timer_msec();
            } else {
                state = SENSOR_UNINIT;
            }
            return false;
        }
        if (hal_i2c_start_write(ACC_I2C_ADDR, init_ctrl4, sizeof(init_ctrl4))) {
            state = SENSOR_INIT_WAIT_2;
        }
        return false;

    case SENSOR_INIT_WAIT_2:
        if (hal_i2c_busy()) return false;
        if (hal_i2c_error() != HAL_I2C_OK) {
            if (++init_error_count >= SENSOR_MAX_INIT_ERRORS) {
                state = SENSOR_IDLE;
                last_poll_ms = hal_timer_msec();
            } else {
                state = SENSOR_UNINIT;
            }
            return false;
        }
        init_error_count = 0;
        last_poll_ms = hal_timer_msec();
        state = SENSOR_IDLE;
        return false;

    case SENSOR_IDLE: {
        uint32_t now = hal_timer_msec();
        if ((now - last_poll_ms) < SENSOR_POLL_MS) return false;
        accel_reg = ACC_AUTO_INC | ACC_OUT_X_L;
        if (hal_i2c_start_write_read(ACC_I2C_ADDR, &accel_reg, 1,
                                      accel_buf, sizeof(accel_buf))) {
            last_poll_ms = now;
            state = SENSOR_WAIT_ACCEL;
        }
        return false;
    }

    case SENSOR_WAIT_ACCEL:
        if (hal_i2c_busy()) return false;
        if (hal_i2c_error() == HAL_I2C_OK) {
            state_results.xl_x = (int16_t)(accel_buf[0] | (accel_buf[1] << 8));
            state_results.xl_y = (int16_t)(accel_buf[2] | (accel_buf[3] << 8));
            state_results.xl_z = (int16_t)(accel_buf[4] | (accel_buf[5] << 8));
        }
        temp_reg = TEMP_AMBIENT_REG;
        if (hal_i2c_start_write_read(TEMP_I2C_ADDR, &temp_reg, 1,
                                      temp_buf, sizeof(temp_buf))) {
            state = SENSOR_WAIT_TEMP;
        } else {
            state = SENSOR_IDLE;
        }
        return false;

    case SENSOR_WAIT_TEMP:
        if (hal_i2c_busy()) return false;
        if (hal_i2c_error() == HAL_I2C_OK) {
            uint16_t raw = (uint16_t)((temp_buf[0] << 8) | temp_buf[1]);
            raw &= 0x1FFF;
            if (raw & 0x1000) {
                raw = raw & 0x0FFF;
                state_results.temperature = -(float)(raw) / 16.0f;
            } else {
                state_results.temperature = (float)(raw) / 16.0f;
            }
        }
        state = SENSOR_IDLE;
        return true;
    }
    return false;
}
