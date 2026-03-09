/*
 * Register access handler for the comms protocol.
 * Port of firmware/src/fw_comms.cpp, using:
 * - state.h structs instead of nanopb protobuf
 * - Function-pointer critical sections instead of ChibiOS mutex
 * - hal_timer for system time
 */

#include "fw_comms.h"

#include <cstring>

#include "control.h"
#include "hal/hal_timer.h"
#include "state.h"

using namespace motor_driver::comms;

/* ── Critical section function pointers ─────────────────────────── */

static void noop(void) {}

void (*critical_section_enter)(void) = noop;
void (*critical_section_exit)(void) = noop;

/* ── handleVarAccess template ───────────────────────────────────── */

template <typename T>
void handleVarAccess(T &var, uint8_t *buf, size_t &index, size_t buf_size,
                     RegAccessType access_type, comm_errors_t &errors) {
    constexpr size_t var_size = sizeof(var);

    if (buf_size - index < var_size) {
        errors |= COMM_ERRORS_BUF_LEN_MISMATCH;
        return;
    }

    uint8_t *u8_var = reinterpret_cast<uint8_t *>(&var);

    critical_section_enter();
    switch (access_type) {
    case RegAccessType::READ:
        std::memcpy(buf + index, u8_var, var_size);
        index += var_size;
        break;
    case RegAccessType::WRITE:
        std::memcpy(u8_var, buf + index, var_size);
        index += var_size;
        break;
    default:
        break;
    }
    critical_section_exit();
}

/* Explicit template instantiations */
template void handleVarAccess<uint8_t>(uint8_t &, uint8_t *, size_t &,
                                        size_t, RegAccessType, comm_errors_t &);
template void handleVarAccess<int8_t>(int8_t &, uint8_t *, size_t &,
                                       size_t, RegAccessType, comm_errors_t &);
template void handleVarAccess<uint16_t>(uint16_t &, uint8_t *, size_t &,
                                         size_t, RegAccessType, comm_errors_t &);
template void handleVarAccess<int16_t>(int16_t &, uint8_t *, size_t &,
                                        size_t, RegAccessType, comm_errors_t &);
template void handleVarAccess<uint32_t>(uint32_t &, uint8_t *, size_t &,
                                         size_t, RegAccessType, comm_errors_t &);
template void handleVarAccess<int32_t>(int32_t &, uint8_t *, size_t &,
                                        size_t, RegAccessType, comm_errors_t &);
template void handleVarAccess<float>(float &, uint8_t *, size_t &,
                                      size_t, RegAccessType, comm_errors_t &);

/* ── Register access handler ────────────────────────────────────── */

size_t commsRegAccessHandler(comm_addr_t start_addr, size_t reg_count,
                             uint8_t *buf, size_t buf_size,
                             RegAccessType access_type,
                             comm_errors_t &errors) {
    size_t index = 0;
    float cur_time;

    /* Reset control timeout on any comms activity (matches ChibiOS behavior) */
    control_reset_timeout();

    for (comm_addr_t addr = start_addr; addr < start_addr + reg_count; addr++) {
        if (addr >= 0x1200 &&
            addr < 0x1200 + state_calibration.enc_ang_corr_table_values.size) {
            /* Encoder angle correction table values */
            handleVarAccess(
                state_calibration.enc_ang_corr_table_values.bytes[addr - 0x1200],
                buf, index, buf_size, access_type, errors);
        } else {
            switch (addr) {
            case 0x0000: /* Register Map Version */
                break;
            case 0x0001: /* Board ID */
                break;
            case 0x0002: /* Firmware Version */
                break;
            case 0x0003: /* Bootloader Version */
                break;
            case 0x0004: /* Store Motor Calibration to Memory */
                state_store_calibration();
                break;
            case 0x0005: /* Clear Motor Calibration from Memory */
                state_clear_calibration();
                break;
            case 0x0006: /* System time */
                cur_time = (float)hal_timer_usec() / 1000000.0f;
                handleVarAccess(cur_time, buf, index, buf_size, access_type,
                                errors);
                break;

            /* ── Calibration registers (0x1000 - 0x11FF) ─────── */
            case 0x1000:
                handleVarAccess(state_calibration.erev_start, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1001:
                handleVarAccess(state_calibration.erevs_per_mrev, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1002:
                handleVarAccess(state_calibration.flip_phases, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1003:
                handleVarAccess(state_calibration.foc_kp_d, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1004:
                handleVarAccess(state_calibration.foc_ki_d, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1005:
                handleVarAccess(state_calibration.foc_kp_q, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1006:
                handleVarAccess(state_calibration.foc_ki_q, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1007:
                handleVarAccess(state_calibration.velocity_kp, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1008:
                handleVarAccess(state_calibration.velocity_kd, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1009:
                handleVarAccess(state_calibration.position_kp, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x100A:
                handleVarAccess(state_calibration.position_kd, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1010:
                handleVarAccess(state_calibration.current_limit, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1011:
                handleVarAccess(state_calibration.torque_limit, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1012:
                handleVarAccess(state_calibration.velocity_limit, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1013:
                handleVarAccess(state_calibration.position_lower_limit, buf,
                                index, buf_size, access_type, errors);
                break;
            case 0x1014:
                handleVarAccess(state_calibration.position_upper_limit, buf,
                                index, buf_size, access_type, errors);
                break;
            case 0x1015:
                handleVarAccess(state_calibration.position_offset, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1020:
                handleVarAccess(state_calibration.motor_resistance, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1021:
                handleVarAccess(state_calibration.motor_inductance, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1022:
                handleVarAccess(state_calibration.motor_torque_const, buf,
                                index, buf_size, access_type, errors);
                break;
            case 0x1030:
                handleVarAccess(state_calibration.control_timeout, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1040:
                handleVarAccess(state_calibration.hf_velocity_filter_param, buf,
                                index, buf_size, access_type, errors);
                break;
            case 0x1041:
                handleVarAccess(state_calibration.lf_velocity_filter_param, buf,
                                index, buf_size, access_type, errors);
                break;
            case 0x1050:
                handleVarAccess(state_calibration.ia_offset, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1051:
                handleVarAccess(state_calibration.ib_offset, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1052:
                handleVarAccess(state_calibration.ic_offset, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x1100:
                handleVarAccess(state_calibration.enc_ang_corr_scale, buf,
                                index, buf_size, access_type, errors);
                break;
            case 0x1101:
                handleVarAccess(state_calibration.enc_ang_corr_offset, buf,
                                index, buf_size, access_type, errors);
                break;

            /* ── Parameter registers (0x2000 - 0x200A) ───────── */
            case 0x2000:
                handleVarAccess(state_parameters.control_mode, buf, index,
                                buf_size, access_type, errors);
                state_parameters.timeout_flag = false;
                break;
            case 0x2001:
                handleVarAccess(state_parameters.foc_d_current_sp, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x2002:
                handleVarAccess(state_parameters.foc_q_current_sp, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x2003:
                handleVarAccess(state_parameters.phase0, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x2004:
                handleVarAccess(state_parameters.phase1, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x2005:
                handleVarAccess(state_parameters.phase2, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x2006:
                handleVarAccess(state_parameters.torque_sp, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x2007:
                handleVarAccess(state_parameters.velocity_sp, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x2008:
                handleVarAccess(state_parameters.position_sp, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x2009:
                handleVarAccess(state_parameters.feed_forward, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x200A:
                handleVarAccess(state_parameters.pwm_drive, buf, index,
                                buf_size, access_type, errors);
                break;

            /* ── Results registers (0x3000 - 0x3040) ─────────── */
            case 0x3000:
                handleVarAccess(state_results.rotor_pos, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3001:
                handleVarAccess(state_results.lf_rotor_vel, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x3002:
                handleVarAccess(state_results.foc_d_current, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x3003:
                handleVarAccess(state_results.foc_q_current, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x3004:
                handleVarAccess(state_results.vin, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3005:
                handleVarAccess(state_results.temperature, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3006:
                handleVarAccess(state_results.xl_x, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3007:
                handleVarAccess(state_results.xl_y, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3008:
                handleVarAccess(state_results.xl_z, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3010:
                handleVarAccess(state_results.raw_enc_value, buf, index,
                                buf_size, access_type, errors);
                break;
            case 0x3011:
                handleVarAccess(state_results.rotor_revs, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3020:
                handleVarAccess(state_results.iq_output, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3021:
                handleVarAccess(state_results.id_output, buf, index, buf_size,
                                access_type, errors);
                break;
            case 0x3040:
                handleVarAccess(state_results.estimation_loops, buf, index,
                                buf_size, access_type, errors);
                break;

            default:
                errors |= COMM_ERRORS_INVALID_ARGS;
                return 0;
            }
        }

        if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) {
            break;
        }
    }

    return index;
}
