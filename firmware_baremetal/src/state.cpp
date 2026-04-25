/*
 * State management: global state variables and calibration persistence.
 * Platform-independent - uses only hal_flash.
 *
 * Supports loading calibration from two flash formats:
 *   0x5454 - Raw C struct (old format)
 *   0x8383 - Protobuf wire format (nanopb, written by ChibiOS firmware)
 */

#include "state.h"

#include <string.h>

#include "hal/hal_flash.h"
#include "hal/hal_iwdg.h"
#include "baremetal_config.h"

struct Results state_results;
struct Calibration state_calibration;
struct Parameters state_parameters;

/* ── Minimal protobuf wire format decoder ──────────────────────── */

struct PbStream {
    const uint8_t *buf;
    size_t len;
    size_t pos;
};

static bool pb_read_byte(struct PbStream *s, uint8_t *out) {
    if (s->pos >= s->len) return false;
    *out = s->buf[s->pos++];
    return true;
}

static bool pb_read_varint(struct PbStream *s, uint32_t *out) {
    uint32_t result = 0;
    uint8_t byte;
    for (int shift = 0; shift < 35; shift += 7) {
        if (!pb_read_byte(s, &byte)) return false;
        result |= (uint32_t)(byte & 0x7F) << shift;
        if (!(byte & 0x80)) {
            *out = result;
            return true;
        }
    }
    return false;  /* Varint too long */
}

static bool pb_read_fixed32(struct PbStream *s, uint32_t *out) {
    if (s->pos + 4 > s->len) return false;
    memcpy(out, s->buf + s->pos, 4);
    s->pos += 4;
    return true;
}

static bool pb_skip_field(struct PbStream *s, uint32_t wire_type) {
    uint32_t len;
    switch (wire_type) {
    case 0: /* varint */
        return pb_read_varint(s, &len);
    case 1: /* 64-bit */
        if (s->pos + 8 > s->len) return false;
        s->pos += 8;
        return true;
    case 2: /* length-delimited */
        if (!pb_read_varint(s, &len)) return false;
        if (s->pos + len > s->len) return false;
        s->pos += len;
        return true;
    case 5: /* 32-bit */
        if (s->pos + 4 > s->len) return false;
        s->pos += 4;
        return true;
    default:
        return false;
    }
}

/*
 * Decode a single float field: verify wire_type is 5 (fixed32),
 * read 4 bytes, copy into destination. On wire_type mismatch, skip.
 */
static bool pb_decode_float(struct PbStream *s, uint32_t wire_type, float *dst) {
    if (wire_type != 5) return pb_skip_field(s, wire_type);
    uint32_t uval;
    if (!pb_read_fixed32(s, &uval)) return false;
    memcpy(dst, &uval, 4);
    return true;
}

/*
 * Decode a single varint field: verify wire_type is 0,
 * read varint, return value. On wire_type mismatch, skip.
 */
static bool pb_decode_uvarint(struct PbStream *s, uint32_t wire_type, uint32_t *dst) {
    if (wire_type != 0) return pb_skip_field(s, wire_type);
    return pb_read_varint(s, dst);
}

static bool decode_calibration_pb(const uint8_t *data, size_t len,
                                  struct Calibration *cal) {
    struct PbStream s = {data, len, 0};
    uint32_t tag, field_num, wire_type, uval;

    while (s.pos < s.len) {
        if (!pb_read_varint(&s, &tag)) return false;
        field_num = tag >> 3;
        wire_type = tag & 0x07;

        switch (field_num) {
        case 1:  /* erev_start (uint32 → uint16_t) */
            if (!pb_decode_uvarint(&s, wire_type, &uval)) return false;
            cal->erev_start = (uint16_t)uval;
            break;
        case 2:  /* erevs_per_mrev (uint32 → uint8_t) */
            if (!pb_decode_uvarint(&s, wire_type, &uval)) return false;
            cal->erevs_per_mrev = (uint8_t)uval;
            break;
        case 3:  /* flip_phases (bool → uint8_t) */
            if (!pb_decode_uvarint(&s, wire_type, &uval)) return false;
            cal->flip_phases = (uint8_t)uval;
            break;
        case 4:  if (!pb_decode_float(&s, wire_type, &cal->foc_kp_d)) return false; break;
        case 5:  if (!pb_decode_float(&s, wire_type, &cal->foc_ki_d)) return false; break;
        case 6:  if (!pb_decode_float(&s, wire_type, &cal->foc_kp_q)) return false; break;
        case 7:  if (!pb_decode_float(&s, wire_type, &cal->foc_ki_q)) return false; break;
        case 8:  if (!pb_decode_float(&s, wire_type, &cal->velocity_kp)) return false; break;
        case 9:  if (!pb_decode_float(&s, wire_type, &cal->velocity_kd)) return false; break;
        case 10: if (!pb_decode_float(&s, wire_type, &cal->position_kp)) return false; break;
        case 11: if (!pb_decode_float(&s, wire_type, &cal->position_kd)) return false; break;
        case 12: if (!pb_decode_float(&s, wire_type, &cal->current_limit)) return false; break;
        case 13: if (!pb_decode_float(&s, wire_type, &cal->torque_limit)) return false; break;
        case 14: if (!pb_decode_float(&s, wire_type, &cal->velocity_limit)) return false; break;
        case 15: if (!pb_decode_float(&s, wire_type, &cal->position_lower_limit)) return false; break;
        case 16: if (!pb_decode_float(&s, wire_type, &cal->position_upper_limit)) return false; break;
        case 17: if (!pb_decode_float(&s, wire_type, &cal->motor_resistance)) return false; break;
        case 18: if (!pb_decode_float(&s, wire_type, &cal->motor_inductance)) return false; break;
        case 19: if (!pb_decode_float(&s, wire_type, &cal->motor_torque_const)) return false; break;
        case 20: /* control_timeout (uint32 → uint16_t) */
            if (!pb_decode_uvarint(&s, wire_type, &uval)) return false;
            cal->control_timeout = (uint16_t)uval;
            break;
        case 21: if (!pb_decode_float(&s, wire_type, &cal->hf_velocity_filter_param)) return false; break;
        case 22: if (!pb_decode_float(&s, wire_type, &cal->lf_velocity_filter_param)) return false; break;
        case 23: if (!pb_decode_float(&s, wire_type, &cal->position_offset)) return false; break;
        case 24: if (!pb_decode_float(&s, wire_type, &cal->ia_offset)) return false; break;
        case 25: if (!pb_decode_float(&s, wire_type, &cal->ib_offset)) return false; break;
        case 26: if (!pb_decode_float(&s, wire_type, &cal->ic_offset)) return false; break;
        case 27: if (!pb_decode_float(&s, wire_type, &cal->enc_ang_corr_scale)) return false; break;
        case 28: if (!pb_decode_float(&s, wire_type, &cal->enc_ang_corr_offset)) return false; break;
        case 29: /* enc_ang_corr_table_values (bytes) */
            if (wire_type != 2) { pb_skip_field(&s, wire_type); break; }
            if (!pb_read_varint(&s, &uval)) return false;
            if (s.pos + uval > s.len) return false;
            if (uval > 257) uval = 257;
            memcpy(cal->enc_ang_corr_table_values.bytes, s.buf + s.pos, uval);
            cal->enc_ang_corr_table_values.size = uval;
            s.pos += uval;
            break;
        default:
            if (!pb_skip_field(&s, wire_type)) return false;
            break;
        }
    }
    return true;
}

void state_init(void) {
    memset(&state_results, 0, sizeof(state_results));
    memset(&state_parameters, 0, sizeof(state_parameters));

    state_parameters.control_mode = 0;
    state_parameters.gate_active = false;
    state_parameters.gate_fault = false;
    state_parameters.timeout_flag = false;

    /* Default calibration */
    memset(&state_calibration, 0, sizeof(state_calibration));
    state_calibration.start_sequence = CALIB_START_SEQ;
    state_calibration.erevs_per_mrev = 1;
    state_calibration.foc_kp_d = 0.5f;
    state_calibration.foc_ki_d = 0.1f;
    state_calibration.foc_kp_q = 1.0f;
    state_calibration.foc_ki_q = 0.2f;
    state_calibration.velocity_kp = 0.1f;
    state_calibration.velocity_kd = 1e-3f;
    state_calibration.position_kp = 5.0f;
    state_calibration.current_limit = 2.0f;
    state_calibration.torque_limit = 3.0f;
    state_calibration.velocity_limit = 10.0f;
    state_calibration.motor_resistance = 17.8f;
    state_calibration.motor_torque_const = 0.5f;
    state_calibration.hf_velocity_filter_param = 0.01f;
    state_calibration.lf_velocity_filter_param = 0.0025f;
}

void state_store_calibration(void) {
    state_calibration.start_sequence = CALIB_START_SEQ;

    hal_iwdg_pause();
    hal_flash_erase(FLASH_CALIB_ADDR, sizeof(struct Calibration));
    hal_iwdg_resume();

    hal_flash_write(FLASH_CALIB_ADDR, &state_calibration,
                    sizeof(struct Calibration));
}

/* Flash layout for nanopb format: {uint16_t start_seq, uint16_t length, uint8_t data[length]} */
struct CalibHeader {
    uint16_t start_sequence;
    uint16_t length;
};

static uint8_t pb_decode_buf[1024];

void state_load_calibration(void) {
    struct CalibHeader header;
    hal_flash_read(FLASH_CALIB_ADDR, &header, sizeof(header));

    if (header.start_sequence == CALIB_START_SEQ) {
        /* Old format: raw C struct */
        hal_flash_read(FLASH_CALIB_ADDR, &state_calibration,
                       sizeof(struct Calibration));
        /* Sanitize table size in case flash data is corrupted */
        if (state_calibration.enc_ang_corr_table_values.size > 257)
            state_calibration.enc_ang_corr_table_values.size = 257;
    } else if (header.start_sequence == CALIB_START_SEQ_PB) {
        /* Nanopb format: decode protobuf wire format */
        size_t total = sizeof(struct CalibHeader) + header.length;
        if (total > sizeof(pb_decode_buf)) return;
        hal_flash_read(FLASH_CALIB_ADDR, pb_decode_buf, total);
        decode_calibration_pb(pb_decode_buf + sizeof(struct CalibHeader),
                              header.length, &state_calibration);
        state_calibration.start_sequence = CALIB_START_SEQ;
    }
    /* Otherwise keep defaults from state_init() */
}

void state_clear_calibration(void) {
    state_init();
}
