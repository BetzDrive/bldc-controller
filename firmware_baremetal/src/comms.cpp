/*
 * Non-blocking comms protocol processor.
 * Port of common/src/comms.cpp, using:
 * - hal_uart.h instead of ChibiOS UART driver
 * - hal_timer.h instead of GPT timer
 * - hal_flash.h instead of ChibiOS flash driver
 * - Step-based polling instead of blocking receive/transmit
 */

#include "comms.h"

#include <cstring>

#include "baremetal_config.h"
#include "comms_defs.hpp"
#include "crc16.h"
#include "fw_comms.h"
#include "hal/hal_flash.h"
#include "hal/hal_iwdg.h"
#include "hal/hal_timer.h"
#include "hal/hal_uart.h"
#include "state.h"

using namespace motor_driver::comms;

/* ── Wire format constants ─────────────────────────────────── */

static constexpr size_t kHeaderLen = 5;       /* sync + ver + flags + len_lo + len_hi */
static constexpr size_t kSubMsgHdrLen = 2;    /* sub-message length word */
static constexpr size_t kCrcLen = 2;
static constexpr size_t kMaxPayloadLen = 255;
static constexpr size_t kSubMsgParseHdrLen = 4; /* sub_len(2) + id(1) + fc(1) */

/* STM32F405 flash sector table (12 sectors) */
static constexpr uint32_t kFlashSectorCount = 12;
static const uint32_t flash_sector_starts[kFlashSectorCount] = {
    0x08000000, 0x08004000, 0x08008000, 0x0800C000,
    0x08010000, 0x08020000, 0x08040000, 0x08060000,
    0x08080000, 0x080A0000, 0x080C0000, 0x080E0000,
};
static const uint32_t flash_sector_sizes[kFlashSectorCount] = {
    0x4000,  0x4000,  0x4000,  0x4000,   /* 16K x4 */
    0x10000,                               /* 64K x1 */
    0x20000, 0x20000, 0x20000,             /* 128K x3 */
    0x20000, 0x20000, 0x20000, 0x20000,    /* 128K x4 */
};

/* ── Framing FSM states ────────────────────────────────────── */

enum class FrameState {
    IDLE,
    SYNC,       /* got 0xFF, expecting version */
    FLAGS,      /* got version, expecting flags */
    LENGTH_L,   /* expecting length low byte */
    LENGTH_H,   /* expecting length high byte */
    PAYLOAD,    /* accumulating payload + CRC bytes */
};

/* ── Protocol response states ──────────────────────────────── */

enum class ProtoState {
    IDLE,
    RESPONDING,
    RESPONDING_READ,
    RESPONDING_MEM,
    RESPONDING_U32,
    RESPONDING_U8,
};

/* ── Module state ──────────────────────────────────────────── */

static FrameState frame_state;
static uint8_t rx_packet[kHeaderLen + kMaxPayloadLen + kCrcLen];
static size_t rx_len;          /* payload length from header */
static size_t rx_index;        /* bytes received into payload+CRC area */
static comm_fg_t rx_flags;

static uint8_t tx_packet[kHeaderLen + kSubMsgHdrLen + kMaxPayloadLen + kCrcLen];

static uint8_t board_id;
static uint8_t resp_count;

/* Protocol handler state */
static ProtoState proto_state;
static comm_fc_t function_code;
static comm_addr_t start_addr;
static size_t reg_count;
static uint32_t u32_value;
static uint8_t u8_value;
static uint32_t src_addr;
static size_t src_len;
static bool broadcast;

/* Deferred action flags */
static bool should_reset;
static uint32_t jump_addr;
static bool wdg_timeout_flag;
static bool wdg_reset_flag;

/* Idle timeout tracking */
static uint32_t last_byte_time_us;

/* ── CRC helper ────────────────────────────────────────────── */

static uint16_t compute_crc(const uint8_t *buf, size_t len) {
    crc16_t crc = crc16_init();
    crc = crc16_update(crc, buf, len);
    return (uint16_t)crc16_finalize(crc);
}

/* ── Forward declarations ──────────────────────────────────── */

static void reset_frame(void);
static void process_packet(void);
static void handle_request(uint8_t *datagram, size_t datagram_len,
                           comm_fg_t flags, comm_errors_t &errors);
static void compose_and_send_response(comm_errors_t errors);

/* ── Public API ────────────────────────────────────────────── */

void comms_init(void) {
    frame_state = FrameState::IDLE;
    proto_state = ProtoState::IDLE;
    resp_count = 1;
    should_reset = false;
    jump_addr = 0;
    wdg_timeout_flag = false;
    wdg_reset_flag = false;

    /* Read board ID from flash */
    hal_flash_read(FLASH_BOARD_ID_ADDR, &board_id, sizeof(board_id));
    if (board_id == 0xFF) {
        board_id = 0;  /* Unprogrammed flash → broadcast ID */
    }
}

size_t comms_step(uint32_t max_bytes) {
    size_t bytes_consumed = 0;

    /* Check idle timeout: if we're mid-packet and no data for too long,
     * reset the framing FSM */
    if (frame_state != FrameState::IDLE) {
        uint32_t now = hal_timer_usec();
        uint32_t elapsed = now - last_byte_time_us;
        if (elapsed >= COMMS_IDLE_TIMEOUT_US) {
            reset_frame();
        }
    }

    while (bytes_consumed < max_bytes) {
        uint8_t byte;
        if (!hal_uart_rx_peek(&byte)) {
            break;  /* No more data */
        }

        last_byte_time_us = hal_timer_usec();
        hal_uart_rx_consume(1);
        bytes_consumed++;

        switch (frame_state) {
        case FrameState::IDLE:
            if (byte == 0xFF) {
                rx_packet[0] = byte;
                frame_state = FrameState::SYNC;
            }
            break;

        case FrameState::SYNC:
            rx_packet[1] = byte;
            if (byte == COMM_VERSION) {
                frame_state = FrameState::FLAGS;
            } else {
                reset_frame();
            }
            break;

        case FrameState::FLAGS:
            rx_packet[2] = byte;
            rx_flags = byte;
            frame_state = FrameState::LENGTH_L;
            break;

        case FrameState::LENGTH_L:
            rx_packet[3] = byte;
            frame_state = FrameState::LENGTH_H;
            break;

        case FrameState::LENGTH_H:
            rx_packet[4] = byte;
            rx_len = ((size_t)byte << 8) | rx_packet[3];
            if (rx_len <= kMaxPayloadLen) {
                rx_index = 0;
                frame_state = FrameState::PAYLOAD;
            } else {
                reset_frame();
            }
            break;

        case FrameState::PAYLOAD:
            rx_packet[kHeaderLen + rx_index] = byte;
            rx_index++;
            if (rx_index >= rx_len + kCrcLen) {
                /* Complete packet received */
                process_packet();
                reset_frame();
            }
            break;
        }
    }

    return bytes_consumed;
}

bool comms_should_reset(void) {
    return should_reset;
}

uint32_t comms_get_jump_addr(void) {
    return jump_addr;
}

void comms_set_watchdog_timeout(bool flag) {
    wdg_timeout_flag = flag;
}

void comms_set_wdg_reset_flag(bool flag) {
    wdg_reset_flag = flag;
}

/* ── Internal helpers ──────────────────────────────────────── */

static void reset_frame(void) {
    frame_state = FrameState::IDLE;
}

static void process_packet(void) {
    /* Validate CRC: computed over payload bytes only */
    uint16_t computed = compute_crc(rx_packet + kHeaderLen, rx_len);
    uint16_t received = (uint16_t)rx_packet[kHeaderLen + rx_len] |
                        ((uint16_t)rx_packet[kHeaderLen + rx_len + 1] << 8);

    if (computed != received) {
        return;  /* Bad CRC, silently drop */
    }

    comm_errors_t errors = COMM_ERRORS_NONE;

    handle_request(rx_packet + kHeaderLen, rx_len, rx_flags, errors);

    compose_and_send_response(errors);
}

/* ── Protocol request handler ──────────────────────────────── */

static void handle_request(uint8_t *datagram, size_t datagram_len,
                           comm_fg_t flags, comm_errors_t &errors) {
    /* If message from another board, decrement resp_count and exit */
    if (flags & COMM_FG_BOARD) {
        if (resp_count > 0) {
            resp_count--;
        }
        return;
    } else {
        resp_count = 0;
        proto_state = ProtoState::IDLE;
    }

    size_t index = 0, next_msg = 0;
    bool found_board = false;

    comm_id_t id = 0;

    /* Loop through sub-messages to find ours */
    while (((datagram_len - index) >= kSubMsgParseHdrLen) && !found_board) {
        uint16_t sub_len = (uint16_t)datagram[index] |
                           ((uint16_t)datagram[index + 1] << 8);
        if (sub_len == 0) {
            break;
        }
        index += 2;  /* skip sub_len field */
        next_msg += sub_len + 2;

        id = datagram[index++];

        if (id != COMM_ID_BROADCAST && id != board_id) {
            /* Not for us - increment resp_count */
            if (proto_state == ProtoState::IDLE) {
                resp_count++;
            }
            index = next_msg;
            continue;
        }
        found_board = true;
        datagram_len = next_msg;
    }

    if (!found_board) {
        resp_count = 0;
        return;
    }

    function_code = datagram[index++];
    broadcast = (id == COMM_ID_BROADCAST);

    /* Clear errors */
    errors = COMM_ERRORS_NONE;

    /* Temporaries for flash operations */
    uint32_t sector_num, dest_addr;
    size_t dest_len;
    bool success;
    uint8_t target_id;

    switch (function_code) {
    case COMM_FC_NOP:
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_REG_READ:
        if (datagram_len - index < 3) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        start_addr = (comm_addr_t)datagram[index++];
        start_addr |= (comm_addr_t)datagram[index++] << 8;
        reg_count = datagram[index++];
        proto_state = ProtoState::RESPONDING_READ;
        break;

    case COMM_FC_REG_WRITE:
        if (datagram_len - index < 3) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        start_addr = (comm_addr_t)datagram[index++];
        start_addr |= (comm_addr_t)datagram[index++] << 8;
        reg_count = datagram[index++];
        commsRegAccessHandler(start_addr, reg_count, &datagram[index],
                              datagram_len - index, RegAccessType::WRITE,
                              errors);
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_REG_READ_WRITE:
        if (datagram_len - index < 6) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        /* Write first (second set of 3 bytes) */
        index += 3;
        start_addr = (comm_addr_t)datagram[index++];
        start_addr |= (comm_addr_t)datagram[index++] << 8;
        reg_count = datagram[index++];
        commsRegAccessHandler(start_addr, reg_count, &datagram[index],
                              datagram_len - index, RegAccessType::WRITE,
                              errors);
        /* Then read (first set of 3 bytes) */
        index -= 6;
        start_addr = (comm_addr_t)datagram[index++];
        start_addr |= (comm_addr_t)datagram[index++] << 8;
        reg_count = datagram[index++];
        proto_state = ProtoState::RESPONDING_READ;
        break;

    case COMM_FC_CLEAR_IWDGRST:
        wdg_reset_flag = false;
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_SYSTEM_RESET:
        should_reset = true;
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_JUMP_TO_ADDR:
        if (datagram_len - index < 4) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        jump_addr = (uint32_t)datagram[index++];
        jump_addr |= (uint32_t)datagram[index++] << 8;
        jump_addr |= (uint32_t)datagram[index++] << 16;
        jump_addr |= (uint32_t)datagram[index++] << 24;
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_FLASH_SECTOR_COUNT:
        u32_value = kFlashSectorCount;
        proto_state = ProtoState::RESPONDING_U32;
        break;

    case COMM_FC_FLASH_SECTOR_START:
        if (datagram_len - index < 4) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        sector_num = (uint32_t)datagram[index++];
        sector_num |= (uint32_t)datagram[index++] << 8;
        sector_num |= (uint32_t)datagram[index++] << 16;
        sector_num |= (uint32_t)datagram[index++] << 24;
        u32_value = (sector_num < kFlashSectorCount) ?
                    flash_sector_starts[sector_num] : 0;
        proto_state = ProtoState::RESPONDING_U32;
        break;

    case COMM_FC_FLASH_SECTOR_SIZE:
        if (datagram_len - index < 4) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        sector_num = (uint32_t)datagram[index++];
        sector_num |= (uint32_t)datagram[index++] << 8;
        sector_num |= (uint32_t)datagram[index++] << 16;
        sector_num |= (uint32_t)datagram[index++] << 24;
        u32_value = (sector_num < kFlashSectorCount) ?
                    flash_sector_sizes[sector_num] : 0;
        proto_state = ProtoState::RESPONDING_U32;
        break;

    case COMM_FC_FLASH_SECTOR_ERASE:
        if (datagram_len - index < 4) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        sector_num = (uint32_t)datagram[index++];
        sector_num |= (uint32_t)datagram[index++] << 8;
        sector_num |= (uint32_t)datagram[index++] << 16;
        sector_num |= (uint32_t)datagram[index++] << 24;
        if (sector_num < kFlashSectorCount) {
            hal_iwdg_pause();
            success = hal_flash_erase(flash_sector_starts[sector_num],
                                      flash_sector_sizes[sector_num]);
            hal_iwdg_resume();
            if (!success) {
                errors |= COMM_ERRORS_OP_FAILED;
            }
        } else {
            errors |= COMM_ERRORS_INVALID_ARGS;
        }
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_FLASH_PROGRAM:
        if (datagram_len - index < 4) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        dest_addr = (uint32_t)datagram[index++];
        dest_addr |= (uint32_t)datagram[index++] << 8;
        dest_addr |= (uint32_t)datagram[index++] << 16;
        dest_addr |= (uint32_t)datagram[index++] << 24;
        dest_len = datagram_len - index;
        success = hal_flash_write(dest_addr, &datagram[index], dest_len);
        if (!success) {
            errors |= COMM_ERRORS_OP_FAILED;
        }
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_FLASH_READ:
        if (datagram_len - index < 8) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        src_addr = (uint32_t)datagram[index++];
        src_addr |= (uint32_t)datagram[index++] << 8;
        src_addr |= (uint32_t)datagram[index++] << 16;
        src_addr |= (uint32_t)datagram[index++] << 24;
        src_len = (size_t)datagram[index++];
        src_len |= (size_t)datagram[index++] << 8;
        src_len |= (size_t)datagram[index++] << 16;
        src_len |= (size_t)datagram[index++] << 24;
        proto_state = ProtoState::RESPONDING_MEM;
        break;

    case COMM_FC_FLASH_VERIFY:
        if (datagram_len - index < 4) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        dest_addr = (uint32_t)datagram[index++];
        dest_addr |= (uint32_t)datagram[index++] << 8;
        dest_addr |= (uint32_t)datagram[index++] << 16;
        dest_addr |= (uint32_t)datagram[index++] << 24;
        dest_len = datagram_len - index;
        if (!hal_flash_verify(dest_addr, &datagram[index], dest_len)) {
            errors |= COMM_ERRORS_OP_FAILED;
        }
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_FLASH_VERIFY_ERASED:
        if (datagram_len - index < 8) {
            errors |= COMM_ERRORS_MALFORMED;
            proto_state = ProtoState::RESPONDING;
            break;
        }
        dest_addr = (uint32_t)datagram[index++];
        dest_addr |= (uint32_t)datagram[index++] << 8;
        dest_addr |= (uint32_t)datagram[index++] << 16;
        dest_addr |= (uint32_t)datagram[index++] << 24;
        dest_len = (size_t)datagram[index++];
        dest_len |= (size_t)datagram[index++] << 8;
        dest_len |= (size_t)datagram[index++] << 16;
        dest_len |= (size_t)datagram[index++] << 24;
        if (!hal_flash_verify_erased(dest_addr, dest_len)) {
            errors |= COMM_ERRORS_OP_FAILED;
        }
        proto_state = ProtoState::RESPONDING;
        break;

    case COMM_FC_ENUMERATE:
        target_id = datagram[index++];
        if (board_id == target_id) {
            u8_value = board_id;
            proto_state = ProtoState::RESPONDING_U8;
        } else {
            proto_state = ProtoState::IDLE;
        }
        break;

    case COMM_FC_CONFIRM_ID:
        /* In firmware (not bootloader), just acknowledge */
        proto_state = ProtoState::RESPONDING;
        break;

    default:
        errors |= COMM_ERRORS_INVALID_FC;
        proto_state = ProtoState::RESPONDING;
        break;
    }
}

/* ── Response composer + transmitter ───────────────────────── */

static void compose_and_send_response(comm_errors_t errors) {
    if (proto_state == ProtoState::IDLE || resp_count != 0) {
        /* No response to send, or waiting for other boards */
        return;
    }

    /* Build sub-message into response area (after wire header + sub_len) */
    uint8_t *resp = tx_packet + kHeaderLen + kSubMsgHdrLen;
    size_t index = 0;
    size_t max_resp_len = kMaxPayloadLen - kSubMsgHdrLen;

    if (broadcast) {
        resp[index++] = 0;
    } else {
        resp[index++] = board_id;
    }
    resp[index++] = function_code;

    /* Reset resp_count for next cycle */
    resp_count = 1;

    size_t error_index;
    size_t read_len;

    switch (proto_state) {
    case ProtoState::RESPONDING:
        resp[index++] = (uint8_t)(errors & 0xFF);
        resp[index++] = (uint8_t)((errors >> 8) & 0xFF);
        break;

    case ProtoState::RESPONDING_READ:
        error_index = index;
        index += 2;  /* reserve space for error code */
        read_len = commsRegAccessHandler(start_addr, reg_count, &resp[index],
                                         max_resp_len - index,
                                         RegAccessType::READ, errors);
        index += read_len;
        resp[error_index] = (uint8_t)(errors & 0xFF);
        resp[error_index + 1] = (uint8_t)((errors >> 8) & 0xFF);
        break;

    case ProtoState::RESPONDING_MEM:
        error_index = index;
        index += 2;
        if (max_resp_len - index >= src_len) {
            hal_flash_read(src_addr, &resp[index], src_len);
            index += src_len;
        } else {
            errors |= COMM_ERRORS_INVALID_ARGS;
        }
        resp[error_index] = (uint8_t)(errors & 0xFF);
        resp[error_index + 1] = (uint8_t)((errors >> 8) & 0xFF);
        break;

    case ProtoState::RESPONDING_U32:
        resp[index++] = (uint8_t)(errors & 0xFF);
        resp[index++] = (uint8_t)((errors >> 8) & 0xFF);
        resp[index++] = (uint8_t)(u32_value & 0xFF);
        resp[index++] = (uint8_t)((u32_value >> 8) & 0xFF);
        resp[index++] = (uint8_t)((u32_value >> 16) & 0xFF);
        resp[index++] = (uint8_t)((u32_value >> 24) & 0xFF);
        break;

    case ProtoState::RESPONDING_U8:
        resp[index++] = (uint8_t)(errors & 0xFF);
        resp[index++] = (uint8_t)((errors >> 8) & 0xFF);
        resp[index++] = u8_value;
        break;

    default:
        proto_state = ProtoState::IDLE;
        return;
    }

    proto_state = ProtoState::IDLE;

    /* Check response fits */
    if (index > max_resp_len) {
        return;
    }

    /* Build wire packet */
    size_t resp_data_len = index;  /* sub-message content length */
    size_t payload_len = kSubMsgHdrLen + resp_data_len;

    /* Wire header */
    tx_packet[0] = 0xFF;
    tx_packet[1] = COMM_VERSION;
    tx_packet[2] = COMM_FG_BOARD;
    if (wdg_reset_flag) {
        tx_packet[2] |= COMM_FG_RESET;
    }
    if (wdg_timeout_flag) {
        tx_packet[2] |= COMM_FG_TIMEOUT;
    }
    tx_packet[3] = (uint8_t)(payload_len & 0xFF);
    tx_packet[4] = (uint8_t)((payload_len >> 8) & 0xFF);

    /* Sub-message length header (length of content, not including the 2-byte
     * length field itself) */
    tx_packet[kHeaderLen] = (uint8_t)(resp_data_len & 0xFF);
    tx_packet[kHeaderLen + 1] = (uint8_t)((resp_data_len >> 8) & 0xFF);

    /* CRC over payload (sub_len_header + sub_msg_content) */
    uint16_t crc = compute_crc(tx_packet + kHeaderLen, payload_len);
    tx_packet[kHeaderLen + payload_len] = (uint8_t)(crc & 0xFF);
    tx_packet[kHeaderLen + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    /* Transmit: set RS485 to TX, send via DMA, block until complete,
     * then switch back to RX.  At 1Mbit/s a typical response (~15 bytes)
     * takes ~150us — well within the IWDG budget. */
    size_t total_len = kHeaderLen + payload_len + kCrcLen;
    hal_uart_set_tx_mode(true);
    hal_uart_tx_send(tx_packet, total_len);
    hal_uart_tx_wait_complete();
    hal_uart_set_tx_mode(false);
}
