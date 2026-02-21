/*
 * Unit tests for sensor state machine (async I2C).
 */

#include <cassert>
#include <cstdio>
#include <cstring>

#include "hal_mock.h"
#include "sensor.h"
#include "state.h"
#include "baremetal_config.h"

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { \
    mock_reset_all(); \
    state_init(); \
    sensor_init(); \
    printf("  %-50s", #name); \
    name(); \
    printf(" PASS\n"); \
    tests_passed++; \
} while (0)

#define ASSERT(cond) do { \
    if (!(cond)) { \
        printf(" FAIL\n    %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        tests_failed++; \
        return; \
    } \
} while (0)

#define ASSERT_EQ(a, b) do { \
    auto _a = (a); auto _b = (b); \
    if (_a != _b) { \
        printf(" FAIL\n    %s:%d: %s == %d, expected %d\n", \
               __FILE__, __LINE__, #a, (int)_a, (int)_b); \
        tests_failed++; \
        return; \
    } \
} while (0)

#define ASSERT_FLOAT_NEAR(a, b, eps) do { \
    float _a = (a); float _b = (b); \
    float _diff = _a - _b; \
    if (_diff < 0) _diff = -_diff; \
    if (_diff > (eps)) { \
        printf(" FAIL\n    %s:%d: %s == %f, expected %f\n", \
               __FILE__, __LINE__, #a, (double)_a, (double)_b); \
        tests_failed++; \
        return; \
    } \
} while (0)

/* ── Helpers ─────────────────────────────────────────────── */

/* Drive sensor through the full init sequence (2 async writes). */
static void drive_init(void) {
    sensor_step();          /* UNINIT → start ctrl1 write → INIT_WAIT_1 */
    mock_i2c_complete();
    sensor_step();          /* INIT_WAIT_1 → start ctrl4 write → INIT_WAIT_2 */
    mock_i2c_complete();
    sensor_step();          /* INIT_WAIT_2 → IDLE */
}

/* Set up mock responses for accel and temp sensors. */
static void setup_sensor_responses(const uint8_t *accel_data, size_t accel_len,
                                   const uint8_t *temp_data, size_t temp_len) {
    mock_i2c_set_response(ACC_I2C_ADDR,
                          (uint8_t)(ACC_AUTO_INC | ACC_OUT_X_L),
                          accel_data, accel_len);
    mock_i2c_set_response(TEMP_I2C_ADDR, TEMP_AMBIENT_REG,
                          temp_data, temp_len);
}

/* ── Tests ───────────────────────────────────────────────── */

TEST(test_init_writes_ctrl_reg1) {
    /* First sensor_step from UNINIT should write CTRL_REG1 */
    sensor_step();

    uint8_t buf[MOCK_I2C_MAX_DATA];
    size_t len = mock_i2c_get_last_write(ACC_I2C_ADDR, buf, sizeof(buf));
    ASSERT_EQ(len, 2);
    ASSERT_EQ(buf[0], ACC_CTRL_REG1);
    ASSERT_EQ(buf[1], 0x27);
}

TEST(test_init_writes_ctrl_reg4) {
    /* After ctrl1 completes, next step should write CTRL_REG4 */
    sensor_step();
    mock_i2c_complete();
    sensor_step();

    uint8_t buf[MOCK_I2C_MAX_DATA];
    size_t len = mock_i2c_get_last_write(ACC_I2C_ADDR, buf, sizeof(buf));
    ASSERT_EQ(len, 2);
    ASSERT_EQ(buf[0], ACC_CTRL_REG4);
    ASSERT_EQ(buf[1], 0x80);
}

TEST(test_init_returns_false) {
    /* All init steps should return false (no sensor data yet) */
    ASSERT(!sensor_step());
    mock_i2c_complete();
    ASSERT(!sensor_step());
    mock_i2c_complete();
    ASSERT(!sensor_step());
}

TEST(test_no_read_before_poll_interval) {
    drive_init();
    /* Timer at 0, last_poll set at 0 → no read until 100ms */
    ASSERT(!sensor_step());
    mock_timer_advance_ms(SENSOR_POLL_MS - 1);
    ASSERT(!sensor_step());
}

TEST(test_returns_false_while_busy) {
    drive_init();
    mock_timer_advance_ms(SENSOR_POLL_MS);

    sensor_step();           /* Start accel read → WAIT_ACCEL */
    ASSERT(!sensor_step());  /* Still busy, returns false */
}

TEST(test_accel_data_parsed) {
    /* xl_x=0x1234, xl_y=0x5678, xl_z=0x9ABC (negative signed) */
    uint8_t accel[] = {0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A};
    uint8_t temp[] = {0x01, 0x90};  /* 25.0C */
    setup_sensor_responses(accel, 6, temp, 2);

    drive_init();
    mock_timer_advance_ms(SENSOR_POLL_MS);

    sensor_step();          /* IDLE → start accel → WAIT_ACCEL */
    mock_i2c_complete();
    sensor_step();          /* WAIT_ACCEL → parse accel, start temp → WAIT_TEMP */
    mock_i2c_complete();
    bool done = sensor_step();  /* WAIT_TEMP → parse temp → IDLE */

    ASSERT(done);
    ASSERT_EQ(state_results.xl_x, 0x1234);
    ASSERT_EQ(state_results.xl_y, 0x5678);
    ASSERT_EQ(state_results.xl_z, (int16_t)0x9ABC);
}

TEST(test_temperature_positive) {
    /* MCP9808: 25.0C = raw 400 = 0x0190, buf = {0x01, 0x90} */
    uint8_t accel[] = {0, 0, 0, 0, 0, 0};
    uint8_t temp[] = {0x01, 0x90};
    setup_sensor_responses(accel, 6, temp, 2);

    drive_init();
    mock_timer_advance_ms(SENSOR_POLL_MS);

    sensor_step();  mock_i2c_complete();
    sensor_step();  mock_i2c_complete();
    sensor_step();

    ASSERT_FLOAT_NEAR(state_results.temperature, 25.0f, 0.01f);
}

TEST(test_temperature_negative) {
    /* MCP9808: -5.0C = raw 80 | 0x1000 = 0x1050, buf = {0x10, 0x50} */
    uint8_t accel[] = {0, 0, 0, 0, 0, 0};
    uint8_t temp[] = {0x10, 0x50};
    setup_sensor_responses(accel, 6, temp, 2);

    drive_init();
    mock_timer_advance_ms(SENSOR_POLL_MS);

    sensor_step();  mock_i2c_complete();
    sensor_step();  mock_i2c_complete();
    sensor_step();

    ASSERT_FLOAT_NEAR(state_results.temperature, -5.0f, 0.01f);
}

TEST(test_init_error_retries) {
    /* First init write fails → retries from UNINIT */
    sensor_step();                                   /* Start ctrl1 write */
    mock_i2c_complete_with_error(HAL_I2C_ERR_NACK);  /* Fail */
    sensor_step();                                   /* Error → back to UNINIT */

    /* Should retry ctrl1 write */
    sensor_step();
    uint8_t buf[MOCK_I2C_MAX_DATA];
    size_t len = mock_i2c_get_last_write(ACC_I2C_ADDR, buf, sizeof(buf));
    ASSERT_EQ(len, 2);
    ASSERT_EQ(buf[0], ACC_CTRL_REG1);
}

TEST(test_init_error_max_skips_to_idle) {
    /* After 3 consecutive init errors, sensor should skip to IDLE */
    for (int i = 0; i < 3; i++) {
        sensor_step();                                   /* Start ctrl1 write */
        mock_i2c_complete_with_error(HAL_I2C_ERR_NACK);  /* Fail */
        sensor_step();                                   /* Error handled */
    }

    /* Should now be in IDLE — poll interval triggers accel read */
    mock_timer_advance_ms(SENSOR_POLL_MS);
    sensor_step();

    uint8_t buf[MOCK_I2C_MAX_DATA];
    size_t len = mock_i2c_get_last_write(ACC_I2C_ADDR, buf, sizeof(buf));
    ASSERT_EQ(len, 1);
    ASSERT_EQ(buf[0], (uint8_t)(ACC_AUTO_INC | ACC_OUT_X_L));
}

TEST(test_accel_error_still_reads_temp) {
    uint8_t accel[] = {0, 0, 0, 0, 0, 0};
    uint8_t temp[] = {0x01, 0x90};  /* 25.0C */
    setup_sensor_responses(accel, 6, temp, 2);

    drive_init();
    mock_timer_advance_ms(SENSOR_POLL_MS);

    sensor_step();                                   /* Start accel read */
    mock_i2c_complete_with_error(HAL_I2C_ERR_NACK);  /* Accel fails */
    sensor_step();                                   /* Skip accel, start temp */
    mock_i2c_complete();                             /* Temp succeeds */
    bool done = sensor_step();                       /* Parse temp → IDLE */

    ASSERT(done);
    /* Accel values unchanged (0 from state_init) */
    ASSERT_EQ(state_results.xl_x, 0);
    ASSERT_EQ(state_results.xl_y, 0);
    ASSERT_EQ(state_results.xl_z, 0);
    /* Temperature should be read */
    ASSERT_FLOAT_NEAR(state_results.temperature, 25.0f, 0.01f);
}

TEST(test_multiple_poll_cycles) {
    /* First cycle */
    uint8_t accel1[] = {0x10, 0x00, 0x20, 0x00, 0x30, 0x00};
    uint8_t temp1[] = {0x01, 0x40};  /* 20.0C */
    setup_sensor_responses(accel1, 6, temp1, 2);

    drive_init();
    mock_timer_advance_ms(SENSOR_POLL_MS);

    sensor_step();  mock_i2c_complete();
    sensor_step();  mock_i2c_complete();
    sensor_step();

    ASSERT_EQ(state_results.xl_x, 0x0010);
    ASSERT_FLOAT_NEAR(state_results.temperature, 20.0f, 0.01f);

    /* Second cycle with different values */
    uint8_t accel2[] = {0xFF, 0x7F, 0x00, 0x80, 0x01, 0x00};
    uint8_t temp2[] = {0x02, 0x80};  /* 40.0C */
    mock_i2c_set_response(ACC_I2C_ADDR,
                          (uint8_t)(ACC_AUTO_INC | ACC_OUT_X_L),
                          accel2, 6);
    mock_i2c_set_response(TEMP_I2C_ADDR, TEMP_AMBIENT_REG,
                          temp2, 2);

    mock_timer_advance_ms(SENSOR_POLL_MS);

    sensor_step();  mock_i2c_complete();
    sensor_step();  mock_i2c_complete();
    bool done = sensor_step();

    ASSERT(done);
    ASSERT_EQ(state_results.xl_x, 0x7FFF);
    ASSERT_EQ(state_results.xl_y, (int16_t)0x8000);
    ASSERT_EQ(state_results.xl_z, 0x0001);
    ASSERT_FLOAT_NEAR(state_results.temperature, 40.0f, 0.01f);
}

/* ── Main ───────────────────────────────────────────────── */

int main(void) {
    printf("Sensor tests:\n");

    RUN_TEST(test_init_writes_ctrl_reg1);
    RUN_TEST(test_init_writes_ctrl_reg4);
    RUN_TEST(test_init_returns_false);
    RUN_TEST(test_no_read_before_poll_interval);
    RUN_TEST(test_returns_false_while_busy);
    RUN_TEST(test_accel_data_parsed);
    RUN_TEST(test_temperature_positive);
    RUN_TEST(test_temperature_negative);
    RUN_TEST(test_init_error_retries);
    RUN_TEST(test_init_error_max_skips_to_idle);
    RUN_TEST(test_accel_error_still_reads_temp);
    RUN_TEST(test_multiple_poll_cycles);

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
