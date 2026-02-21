/*
 * Unit tests for state management.
 */

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "hal_mock.h"
#include "hal/hal_flash.h"
#include "state.h"
#include "baremetal_config.h"

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { \
    mock_reset_all(); \
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

#define ASSERT_FLOAT_EQ(a, b) do { \
    float _a = (a); float _b = (b); \
    if (fabsf(_a - _b) > 1e-6f) { \
        printf(" FAIL\n    %s:%d: %s == %f, expected %f\n", \
               __FILE__, __LINE__, #a, (double)_a, (double)_b); \
        tests_failed++; \
        return; \
    } \
} while (0)

/* ── Tests ──────────────────────────────────────────────── */

TEST(test_init_defaults) {
    state_init();

    /* Results zeroed */
    ASSERT_FLOAT_EQ(state_results.foc_d_current, 0.0f);
    ASSERT_FLOAT_EQ(state_results.temperature, 0.0f);
    ASSERT(state_results.xl_x == 0);

    /* Parameters defaults */
    ASSERT(state_parameters.control_mode == 0);
    ASSERT(!state_parameters.gate_active);
    ASSERT(!state_parameters.gate_fault);
    ASSERT(!state_parameters.timeout_flag);

    /* Calibration defaults */
    ASSERT(state_calibration.start_sequence == CALIB_START_SEQ);
    ASSERT(state_calibration.erevs_per_mrev == 1);
    ASSERT_FLOAT_EQ(state_calibration.foc_kp_d, 0.5f);
    ASSERT_FLOAT_EQ(state_calibration.foc_ki_d, 0.1f);
    ASSERT_FLOAT_EQ(state_calibration.foc_kp_q, 1.0f);
    ASSERT_FLOAT_EQ(state_calibration.foc_ki_q, 0.2f);
    ASSERT_FLOAT_EQ(state_calibration.current_limit, 2.0f);
    ASSERT_FLOAT_EQ(state_calibration.motor_resistance, 17.8f);
}

TEST(test_store_load_roundtrip) {
    state_init();

    /* Modify some calibration values */
    state_calibration.erev_start = 1234;
    state_calibration.foc_kp_d = 3.14f;
    state_calibration.current_limit = 5.0f;
    state_calibration.motor_resistance = 22.0f;

    /* Store to mock flash */
    state_store_calibration();

    /* Corrupt the in-memory copy */
    state_calibration.erev_start = 0;
    state_calibration.foc_kp_d = 0.0f;
    state_calibration.current_limit = 0.0f;

    /* Load back from mock flash */
    state_load_calibration();

    /* Verify round-trip */
    ASSERT(state_calibration.start_sequence == CALIB_START_SEQ);
    ASSERT(state_calibration.erev_start == 1234);
    ASSERT_FLOAT_EQ(state_calibration.foc_kp_d, 3.14f);
    ASSERT_FLOAT_EQ(state_calibration.current_limit, 5.0f);
    ASSERT_FLOAT_EQ(state_calibration.motor_resistance, 22.0f);
}

TEST(test_load_from_erased_flash_keeps_defaults) {
    state_init();

    /* Mock flash is initialized to 0xFF (erased state) */
    /* Load should keep defaults since start_sequence won't match */
    state_load_calibration();

    ASSERT(state_calibration.start_sequence == CALIB_START_SEQ);
    ASSERT(state_calibration.erevs_per_mrev == 1);
    ASSERT_FLOAT_EQ(state_calibration.foc_kp_d, 0.5f);
}

TEST(test_clear_calibration_resets_to_defaults) {
    state_init();

    /* Modify calibration */
    state_calibration.foc_kp_d = 99.0f;
    state_calibration.erev_start = 5000;

    /* Clear should reset to defaults */
    state_clear_calibration();

    ASSERT(state_calibration.erev_start == 0);
    ASSERT_FLOAT_EQ(state_calibration.foc_kp_d, 0.5f);
}

TEST(test_store_preserves_start_sequence) {
    state_init();
    state_store_calibration();

    /* Read raw from mock flash */
    uint16_t seq;
    hal_flash_read(FLASH_CALIB_ADDR, &seq, sizeof(seq));
    ASSERT(seq == CALIB_START_SEQ);
}

/* ── Main ───────────────────────────────────────────────── */

int main(void) {
    printf("State tests:\n");

    RUN_TEST(test_init_defaults);
    RUN_TEST(test_store_load_roundtrip);
    RUN_TEST(test_load_from_erased_flash_keeps_defaults);
    RUN_TEST(test_clear_calibration_resets_to_defaults);
    RUN_TEST(test_store_preserves_start_sequence);

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
