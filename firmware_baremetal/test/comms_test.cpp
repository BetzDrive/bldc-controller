/*
 * Unit tests for comms protocol FSM.
 */

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "hal_mock.h"
#include "baremetal_config.h"
#include "comms.h"
#include "comms_defs.hpp"
#include "crc16.h"
#include "fw_comms.h"
#include "hal/hal_flash.h"
#include "state.h"

using namespace motor_driver::comms;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { \
    mock_reset_all(); \
    state_init(); \
    comms_init(); \
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

/* ── Packet building helpers ─────────────────────────────── */

static uint16_t compute_crc(const uint8_t *buf, size_t len) {
    crc16_t crc = crc16_init();
    crc = crc16_update(crc, buf, len);
    return (uint16_t)crc16_finalize(crc);
}

/*
 * Build a wire packet and inject it into the mock UART RX buffer.
 *
 * Wire format:
 *   [0xFF] [COMM_VERSION] [flags] [len_lo] [len_hi] [payload...] [crc_lo] [crc_hi]
 *
 * CRC is computed over the payload bytes only.
 */
static void inject_packet(uint8_t flags, const uint8_t *payload, size_t payload_len) {
    uint8_t pkt[5 + 255 + 2];
    pkt[0] = 0xFF;
    pkt[1] = COMM_VERSION;
    pkt[2] = flags;
    pkt[3] = (uint8_t)(payload_len & 0xFF);
    pkt[4] = (uint8_t)((payload_len >> 8) & 0xFF);
    memcpy(pkt + 5, payload, payload_len);

    uint16_t crc = compute_crc(payload, payload_len);
    pkt[5 + payload_len] = (uint8_t)(crc & 0xFF);
    pkt[5 + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    mock_uart_inject_rx(pkt, 5 + payload_len + 2);
}

/*
 * Build a sub-message payload.
 * Sub-message format: [sub_len_lo] [sub_len_hi] [board_id] [function_code] [data...]
 * sub_len is the length of board_id + function_code + data (excluding the 2-byte length).
 */
static size_t build_submsg(uint8_t *buf, uint8_t board_id, uint8_t fc,
                           const uint8_t *data, size_t data_len) {
    uint16_t sub_len = 1 + 1 + (uint16_t)data_len;  /* id + fc + data */
    buf[0] = (uint8_t)(sub_len & 0xFF);
    buf[1] = (uint8_t)((sub_len >> 8) & 0xFF);
    buf[2] = board_id;
    buf[3] = fc;
    if (data_len > 0) {
        memcpy(buf + 4, data, data_len);
    }
    return 2 + sub_len;  /* total bytes written */
}

/* Set mock board ID in flash and re-init comms */
static void set_board_id(uint8_t id) {
    hal_flash_write(FLASH_BOARD_ID_ADDR, &id, 1);
    comms_init();
}

/* ── Tests ─────────────────────────────────────────────────── */

TEST(test_no_data_returns_zero) {
    size_t consumed = comms_step(32);
    ASSERT(consumed == 0);
}

TEST(test_partial_sync_no_crash) {
    /* Just a sync byte, no version - should not crash */
    uint8_t sync = 0xFF;
    mock_uart_inject_rx(&sync, 1);
    size_t consumed = comms_step(32);
    ASSERT(consumed == 1);

    /* No TX should be generated */
    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len == 0);
}

TEST(test_bad_version_resets_fsm) {
    uint8_t data[] = {0xFF, 0x00};  /* sync + bad version */
    mock_uart_inject_rx(data, 2);
    comms_step(32);

    /* FSM should reset, no response */
    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) == 0);
}

TEST(test_nop_broadcast_response) {
    set_board_id(5);

    /* Build NOP sub-message to broadcast (id=0) */
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 0, COMM_FC_NOP, NULL, 0);

    inject_packet(0x00, payload, plen);
    comms_step(256);

    /* Should get a response */
    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Verify wire format: sync, version, flags (COMM_FG_BOARD set) */
    ASSERT(tx_buf[0] == 0xFF);
    ASSERT(tx_buf[1] == COMM_VERSION);
    ASSERT((tx_buf[2] & COMM_FG_BOARD) != 0);

    /* Parse response payload length */
    size_t resp_payload_len = (size_t)tx_buf[3] | ((size_t)tx_buf[4] << 8);
    ASSERT(resp_payload_len > 0);

    /* Verify CRC of response */
    uint16_t resp_crc = compute_crc(tx_buf + 5, resp_payload_len);
    uint16_t pkt_crc = (uint16_t)tx_buf[5 + resp_payload_len] |
                       ((uint16_t)tx_buf[5 + resp_payload_len + 1] << 8);
    ASSERT(resp_crc == pkt_crc);

    /* Parse sub-message: [sub_len_lo][sub_len_hi][board_id=0][fc=NOP][err_lo][err_hi] */
    uint8_t *resp_body = tx_buf + 5;
    uint16_t sub_len = (uint16_t)resp_body[0] | ((uint16_t)resp_body[1] << 8);
    ASSERT(sub_len >= 4);  /* id + fc + 2 error bytes */
    ASSERT(resp_body[2] == 0);   /* broadcast response id = 0 */
    ASSERT(resp_body[3] == COMM_FC_NOP);
    /* No errors */
    uint16_t resp_errors = (uint16_t)resp_body[4] | ((uint16_t)resp_body[5] << 8);
    ASSERT(resp_errors == 0);
}

TEST(test_nop_addressed_response) {
    set_board_id(3);

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 3, COMM_FC_NOP, NULL, 0);

    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Response sub-message should have our board ID */
    uint8_t *resp_body = tx_buf + 5;
    ASSERT(resp_body[2] == 3);
    ASSERT(resp_body[3] == COMM_FC_NOP);
}

TEST(test_wrong_board_id_no_response) {
    set_board_id(5);

    /* Send to board 7, we are board 5 */
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 7, COMM_FC_NOP, NULL, 0);

    inject_packet(0x00, payload, plen);
    comms_step(256);

    /* Should NOT get a response */
    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) == 0);
}

TEST(test_bad_crc_no_response) {
    set_board_id(1);

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 0, COMM_FC_NOP, NULL, 0);

    /* Build packet manually with bad CRC */
    uint8_t pkt[64];
    pkt[0] = 0xFF;
    pkt[1] = COMM_VERSION;
    pkt[2] = 0x00;
    pkt[3] = (uint8_t)(plen & 0xFF);
    pkt[4] = (uint8_t)((plen >> 8) & 0xFF);
    memcpy(pkt + 5, payload, plen);
    pkt[5 + plen] = 0xDE;      /* Bad CRC */
    pkt[5 + plen + 1] = 0xAD;

    mock_uart_inject_rx(pkt, 5 + plen + 2);
    comms_step(256);

    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) == 0);
}

TEST(test_idle_timeout_resets_fsm) {
    /* Send partial packet (just sync + version) */
    uint8_t data[] = {0xFF, COMM_VERSION};
    mock_uart_inject_rx(data, 2);
    comms_step(32);

    /* Advance time past idle timeout */
    mock_timer_advance_us(COMMS_IDLE_TIMEOUT_US + 1);

    /* Now send a complete valid packet */
    set_board_id(1);
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 0, COMM_FC_NOP, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    /* Should get a response (FSM was reset, not stuck) */
    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) > 0);
}

TEST(test_reg_read_rotor_pos) {
    set_board_id(1);

    /* Set a known rotor position */
    state_results.rotor_pos = 1.234f;

    /* REG_READ: addr=0x3000 (rotor_pos), count=1 */
    uint8_t sub_data[] = {
        0x00, 0x30,  /* addr = 0x3000 */
        0x01,        /* count = 1 */
    };
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_REG_READ, sub_data, sizeof(sub_data));

    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Parse response: header(5) + sub_len(2) + id(1) + fc(1) + errors(2) + float(4) + crc(2) */
    uint8_t *resp_body = tx_buf + 5;
    ASSERT(resp_body[3] == COMM_FC_REG_READ);

    /* Check errors = 0 */
    uint16_t errors = (uint16_t)resp_body[4] | ((uint16_t)resp_body[5] << 8);
    ASSERT(errors == 0);

    /* Check float value */
    float val;
    memcpy(&val, resp_body + 6, sizeof(float));
    ASSERT_FLOAT_EQ(val, 1.234f);
}

TEST(test_reg_write_control_mode) {
    set_board_id(1);

    /* REG_WRITE: addr=0x2000 (control_mode), count=1, data=0x03 */
    uint8_t sub_data[] = {
        0x00, 0x20,  /* addr = 0x2000 */
        0x01,        /* count = 1 */
        0x03,        /* value = 3 (velocity mode) */
    };
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_REG_WRITE, sub_data, sizeof(sub_data));

    inject_packet(0x00, payload, plen);
    comms_step(256);

    /* Verify state was updated */
    ASSERT(state_parameters.control_mode == 3);
    ASSERT(state_parameters.timeout_flag == false);

    /* Should get a response */
    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) > 0);
}

TEST(test_reg_read_write_simultaneous) {
    set_board_id(1);

    state_results.rotor_pos = 2.5f;

    /* REG_READ_WRITE: read addr=0x3000 count=1, write addr=0x2000 count=1, data=0x04 */
    uint8_t sub_data[] = {
        /* Read spec: */
        0x00, 0x30,  /* read addr = 0x3000 */
        0x01,        /* read count = 1 */
        /* Write spec: */
        0x00, 0x20,  /* write addr = 0x2000 */
        0x01,        /* write count = 1 */
        0x04,        /* write data = 4 (position mode) */
    };
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_REG_READ_WRITE, sub_data, sizeof(sub_data));

    inject_packet(0x00, payload, plen);
    comms_step(256);

    /* Verify write took effect */
    ASSERT(state_parameters.control_mode == 4);

    /* Verify response contains read data */
    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    uint8_t *resp_body = tx_buf + 5;
    ASSERT(resp_body[3] == COMM_FC_REG_READ_WRITE);

    float val;
    memcpy(&val, resp_body + 6, sizeof(float));
    ASSERT_FLOAT_EQ(val, 2.5f);
}

TEST(test_system_reset_flag) {
    set_board_id(1);

    ASSERT(!comms_should_reset());

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_SYSTEM_RESET, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    ASSERT(comms_should_reset());
}

TEST(test_flash_sector_count) {
    set_board_id(1);

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_FLASH_SECTOR_COUNT, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Parse: id + fc + errors(2) + u32(4) */
    uint8_t *resp_body = tx_buf + 5;
    uint32_t count;
    memcpy(&count, resp_body + 6, sizeof(uint32_t));
    ASSERT(count == 12);
}

TEST(test_invalid_fc_returns_error) {
    set_board_id(1);

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, 0x7F /* invalid fc */, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    uint8_t *resp_body = tx_buf + 5;
    uint16_t errors = (uint16_t)resp_body[4] | ((uint16_t)resp_body[5] << 8);
    ASSERT((errors & COMM_ERRORS_INVALID_FC) != 0);
}

TEST(test_other_board_message_decrements_resp_count) {
    set_board_id(1);

    /* First, send a message from "another board" (COMM_FG_BOARD flag set).
     * This should decrement the resp_count but not generate a response. */
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 2, COMM_FC_NOP, NULL, 0);
    inject_packet(COMM_FG_BOARD, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) == 0);
}

TEST(test_multi_submsg_resp_count_ordering) {
    set_board_id(3);

    /* Build payload with two sub-messages:
     * Sub1: board_id=1, NOP (not for us, but increments resp_count)
     * Sub2: board_id=3, NOP (for us) */
    uint8_t payload[32];
    size_t offset = build_submsg(payload, 1, COMM_FC_NOP, NULL, 0);
    offset += build_submsg(payload + offset, 3, COMM_FC_NOP, NULL, 0);

    inject_packet(0x00, payload, offset);
    comms_step(256);

    /* resp_count was 1 (skipped board 1 before finding us), so we should
     * NOT respond immediately. But after the packet from board 1... actually
     * let's verify no response since resp_count > 0 */
    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));

    /* We have resp_count=1 because one board came before us.
     * So we should NOT respond yet. */
    ASSERT(tx_len == 0);

    /* Now simulate a response from board 1 (COMM_FG_BOARD flag) */
    uint8_t payload2[16];
    size_t plen2 = build_submsg(payload2, 1, COMM_FC_NOP, NULL, 0);
    inject_packet(COMM_FG_BOARD, payload2, plen2);
    comms_step(256);

    /* Now resp_count==0 and proto_state is still RESPONDING (it was
     * preserved when the first compose_and_send_response was suppressed).
     * The board message's process_packet calls compose_and_send_response
     * again, which now finds resp_count==0 and sends the deferred response.
     * This matches the original ChibiOS behavior. */
    tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Verify it's a valid NOP response */
    ASSERT(tx_buf[0] == 0xFF);
    ASSERT(tx_buf[1] == COMM_VERSION);
    ASSERT((tx_buf[2] & COMM_FG_BOARD) != 0);
    uint8_t *resp_body = tx_buf + 5;
    ASSERT(resp_body[2] == 3);  /* our board ID */
    ASSERT(resp_body[3] == COMM_FC_NOP);
}

TEST(test_wdg_timeout_flag_in_response) {
    set_board_id(1);
    comms_set_watchdog_timeout(true);

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_NOP, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Check COMM_FG_TIMEOUT flag is set */
    ASSERT((tx_buf[2] & COMM_FG_TIMEOUT) != 0);
}

TEST(test_wdg_reset_flag_in_response) {
    set_board_id(1);
    comms_set_wdg_reset_flag(true);

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_NOP, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Check COMM_FG_RESET flag is set */
    ASSERT((tx_buf[2] & COMM_FG_RESET) != 0);
}

TEST(test_max_bytes_limits_processing) {
    set_board_id(1);

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_NOP, NULL, 0);
    inject_packet(0x00, payload, plen);

    /* Only consume 2 bytes */
    size_t consumed = comms_step(2);
    ASSERT(consumed == 2);

    /* No response yet (incomplete packet) */
    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) == 0);

    /* Consume the rest */
    comms_step(256);
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) > 0);
}

TEST(test_flash_read_via_comms) {
    set_board_id(1);

    /* Write known data to mock flash */
    uint8_t flash_data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    hal_flash_write(FLASH_BOARD_ID_ADDR + 4, flash_data, 4);

    /* FLASH_READ: addr=FLASH_BOARD_ID_ADDR+4, len=4 */
    uint32_t addr = FLASH_BOARD_ID_ADDR + 4;
    uint32_t len = 4;
    uint8_t sub_data[8];
    memcpy(sub_data, &addr, 4);
    memcpy(sub_data + 4, &len, 4);

    uint8_t payload[32];
    size_t plen = build_submsg(payload, 1, COMM_FC_FLASH_READ, sub_data, 8);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Parse response: id + fc + errors(2) + data(4) */
    uint8_t *resp_body = tx_buf + 5;
    ASSERT(resp_body[3] == COMM_FC_FLASH_READ);

    uint16_t errors = (uint16_t)resp_body[4] | ((uint16_t)resp_body[5] << 8);
    ASSERT(errors == 0);

    ASSERT(resp_body[6] == 0xDE);
    ASSERT(resp_body[7] == 0xAD);
    ASSERT(resp_body[8] == 0xBE);
    ASSERT(resp_body[9] == 0xEF);
}

TEST(test_enumerate_matching_id) {
    set_board_id(5);

    uint8_t sub_data[] = {5};  /* target_id = 5 (matches) */
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 5, COMM_FC_ENUMERATE, sub_data, 1);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    size_t tx_len = mock_uart_read_tx(tx_buf, sizeof(tx_buf));
    ASSERT(tx_len > 0);

    /* Response should contain our board ID as u8 */
    uint8_t *resp_body = tx_buf + 5;
    ASSERT(resp_body[3] == COMM_FC_ENUMERATE);
    ASSERT(resp_body[6] == 5);  /* u8_value = our board_id */
}

TEST(test_enumerate_non_matching_id) {
    set_board_id(5);

    uint8_t sub_data[] = {7};  /* target_id = 7 (doesn't match) */
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 5, COMM_FC_ENUMERATE, sub_data, 1);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    /* No response (proto_state goes to IDLE) */
    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) == 0);
}

TEST(test_consecutive_packets) {
    set_board_id(1);

    /* Send first NOP */
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_NOP, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) > 0);

    /* Send second NOP immediately */
    inject_packet(0x00, payload, plen);
    comms_step(256);

    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) > 0);
}

TEST(test_oversized_length_resets_fsm) {
    /* Build a packet header with length > 255 */
    uint8_t pkt[] = {0xFF, COMM_VERSION, 0x00, 0x00, 0x02};  /* length = 512 */
    mock_uart_inject_rx(pkt, 5);
    comms_step(32);

    /* FSM should reset. Now send valid packet. */
    set_board_id(1);
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 0, COMM_FC_NOP, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) > 0);
}

TEST(test_unprogrammed_board_responds_to_broadcast) {
    /* Don't call set_board_id — flash is 0xFF (erased).
     * comms_init() maps 0xFF → board_id=0 = COMM_ID_BROADCAST.
     * Board should respond to broadcast (id=0) but NOT to id=1. */

    /* Broadcast NOP → should respond */
    uint8_t payload[16];
    size_t plen = build_submsg(payload, 0, COMM_FC_NOP, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) > 0);
}

TEST(test_unprogrammed_board_ignores_addressed) {
    /* Same unprogrammed state: board_id=0.
     * Packet addressed to id=1 should be ignored. */

    uint8_t payload[16];
    size_t plen = build_submsg(payload, 1, COMM_FC_NOP, NULL, 0);
    inject_packet(0x00, payload, plen);
    comms_step(256);

    uint8_t tx_buf[256];
    ASSERT(mock_uart_read_tx(tx_buf, sizeof(tx_buf)) == 0);
}

/* ── Main ──────────────────────────────────────────────────── */

int main(void) {
    printf("Comms tests:\n");

    RUN_TEST(test_no_data_returns_zero);
    RUN_TEST(test_partial_sync_no_crash);
    RUN_TEST(test_bad_version_resets_fsm);
    RUN_TEST(test_nop_broadcast_response);
    RUN_TEST(test_nop_addressed_response);
    RUN_TEST(test_wrong_board_id_no_response);
    RUN_TEST(test_bad_crc_no_response);
    RUN_TEST(test_idle_timeout_resets_fsm);
    RUN_TEST(test_reg_read_rotor_pos);
    RUN_TEST(test_reg_write_control_mode);
    RUN_TEST(test_reg_read_write_simultaneous);
    RUN_TEST(test_system_reset_flag);
    RUN_TEST(test_flash_sector_count);
    RUN_TEST(test_invalid_fc_returns_error);
    RUN_TEST(test_other_board_message_decrements_resp_count);
    RUN_TEST(test_multi_submsg_resp_count_ordering);
    RUN_TEST(test_wdg_timeout_flag_in_response);
    RUN_TEST(test_wdg_reset_flag_in_response);
    RUN_TEST(test_max_bytes_limits_processing);
    RUN_TEST(test_flash_read_via_comms);
    RUN_TEST(test_enumerate_matching_id);
    RUN_TEST(test_enumerate_non_matching_id);
    RUN_TEST(test_consecutive_packets);
    RUN_TEST(test_oversized_length_resets_fsm);
    RUN_TEST(test_unprogrammed_board_responds_to_broadcast);
    RUN_TEST(test_unprogrammed_board_ignores_addressed);

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
