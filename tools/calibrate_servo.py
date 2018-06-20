#!/usr/bin/env python

from __future__ import division
import argparse
from comms import *
import serial
import time
import sys
import numpy as np
from scipy import signal as sps, stats, interpolate
import matplotlib.pyplot as plt
import math
from math import pi
import json
from collections import OrderedDict

# 14-bit encoder
encoder_ticks_per_rev = 2 ** 14
steps_per_erev = 6
elec_ang_corr_table_size = 257

def midpoint_clamping_svm(angle, magnitude):
    dc = magnitude * np.cos(np.array([angle, angle - (2 / 3 * pi), angle - (4 / 3 * pi)])) / np.sqrt(3) + 0.5
    shift = 0.5 * (1.0 - np.min(dc) - np.max(dc))
    return dc + shift

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate calibration parameters for a single servo (motor and controller board).')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('duty_cycle', type=float, help='Duty cycle')
    parser.add_argument('--max_steps', type=int, help='Maximum number of steps')
    parser.add_argument('--delay', type=float, help='Delay between steps')
    parser.add_argument('--start_delay', type=float, help='Delay before first step')
    parser.add_argument('--plot', dest='plot', action='store_true', help='Plot the electrical angle compensation table')
    parser.add_argument('--no-plot', dest='plot', action='store_false', help='Don\'t plot the electrical angle compensation table')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, duty_cycle=0.6, max_steps=126, delay=0.05, start_delay=0.5, plot=False)
    args = parser.parse_args()

    #
    # Data collection
    #

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)

    client = BLDCControllerClient(ser, protocol_v2=True)

    client.leaveBootloader(args.board_id)
    time.sleep(0.2) # Wait for the controller to reset
    ser.reset_input_buffer()

    def set_phase_state(step):
        angle = (step % steps_per_erev) / steps_per_erev * 2 * np.pi
        a, b, c = midpoint_clamping_svm(angle, args.duty_cycle)
        client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', a, b, c))

    client.writeRegisters(args.board_id, 0x1030, 1, struct.pack('<H', 3000)) # Control watchdog timeout
    client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', 0, 0, 0))
    client.writeRegisters(args.board_id, 0x2000, 1, struct.pack('<B', 1))

    # Start one step before phase A to avoid boundary issues
    set_phase_state(-1)
    time.sleep(args.start_delay)

    # Forward
    forward_raw_angles = []
    for i in range(args.max_steps):
        set_phase_state(i)
        time.sleep(args.delay)

        raw_angle = struct.unpack('<H', client.readRegisters(args.board_id, 0x3010, 1))[0]

        if i >= steps_per_erev and (i % steps_per_erev == 0) and (abs(forward_raw_angles[0] - raw_angle) < abs(forward_raw_angles[steps_per_erev // 2] - forward_raw_angles[0])):
            break

        forward_raw_angles.append(raw_angle)

    step_count = len(forward_raw_angles)

    # Take one more step to avoid boundary issues
    set_phase_state(step_count)
    time.sleep(args.delay)

    # Backward
    backward_raw_angles = []
    for i in range(step_count - 1, -1, -1):
        set_phase_state(i)
        time.sleep(args.delay)

        raw_angle = struct.unpack('<H', client.readRegisters(args.board_id, 0x3010, 1))[0]

        backward_raw_angles.append(raw_angle)

    client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', 0, 0, 0))
    ser.close()

    #
    # Data analysis
    #

    # Flip backward raw angles to correspond with forward raw angles
    backward_raw_angles = backward_raw_angles[::-1]

    forward_raw_angles = np.array(forward_raw_angles)
    backward_raw_angles = np.array(backward_raw_angles)

    # Convert to radians
    forward_angles = forward_raw_angles / encoder_ticks_per_rev * 2 * np.pi
    backward_angles = backward_raw_angles / encoder_ticks_per_rev * 2 * np.pi

    # Phase unwrapping
    forward_angles = np.unwrap(forward_angles)
    backward_angles = np.unwrap(backward_angles)

    # Average forward and backward measurements
    enc_angles = (forward_angles + backward_angles) / 2

    # Find erevs/mrev, including its sign
    est_enc_angle_slope = np.mean(np.diff(np.unwrap(enc_angles)))
    steps_per_mrev = int(np.round(2 * np.pi / est_enc_angle_slope))
    assert abs(steps_per_mrev) == step_count
    erevs_per_mrev = steps_per_mrev // steps_per_erev # Could be negative

    # Shift smallest encoder angle corresponding to phase A to the front
    # This will be the start of a "mechanical revolution"
    enc_angles = enc_angles % (2 * np.pi) # Wrap phase
    smallest_phase_a_index = np.argmin(enc_angles[::steps_per_erev]) * steps_per_erev
    enc_angles = np.roll(enc_angles, -smallest_phase_a_index)

    # Find offset between mechanical angle and encoder angle
    enc_angle_slope = 2 * np.pi / steps_per_mrev
    enc_angle_offset = np.mean((enc_angles - np.arange(step_count) * enc_angle_slope) % (2 * np.pi))
    enc_tick_offset = enc_angle_offset / (2 * np.pi) * encoder_ticks_per_rev

    # Find mapping from mechanical angle to true electrical angle
    mech_angles = np.unwrap(enc_angles - enc_angle_offset)
    mech_angles_trend = np.arange(step_count) * enc_angle_slope
    smoothed_mech_angles = sps.savgol_filter(mech_angles - mech_angles_trend, 31, 3, mode='wrap') + mech_angles_trend
    elec_angle_slope = enc_angle_slope * erevs_per_mrev
    true_elec_angles = np.arange(step_count) * elec_angle_slope

    # if erevs_per_mrev < 0:
    #     smoothed_mech_angles = np.roll(smoothed_mech_angles, -1) % (2 * np.pi)
    #     true_elec_angles = np.roll(true_elec_angles, -1) % (2 * np.pi * abs(erevs_per_mrev))

    # Extend parameters to np.interp
    ext_smoothed_mech_angles = np.concatenate((
        smoothed_mech_angles[-steps_per_erev:] - step_count * enc_angle_slope,
        smoothed_mech_angles,
        smoothed_mech_angles[:steps_per_erev] + step_count * enc_angle_slope))
    ext_true_elec_angles = np.concatenate((
        true_elec_angles[-steps_per_erev:] - step_count * enc_angle_slope * erevs_per_mrev,
        true_elec_angles,
        true_elec_angles[:steps_per_erev] + step_count * enc_angle_slope * erevs_per_mrev))

    # Ensure ext_smoothed_mech_angles are strictly monotonically increasing
    if ext_smoothed_mech_angles[0] > ext_smoothed_mech_angles[-1]:
        ext_smoothed_mech_angles = ext_smoothed_mech_angles[::-1]
        ext_true_elec_angles = ext_true_elec_angles[::-1]
    assert all(np.diff(ext_smoothed_mech_angles) > 0), 'Mechanical angles are not strictly monotonically increasing'

    table_mech_angles = np.linspace(0, step_count * enc_angle_slope, elec_ang_corr_table_size, endpoint=True)
    table_elec_angle_residuals = np.unwrap(np.interp(table_mech_angles, ext_smoothed_mech_angles, ext_true_elec_angles) - table_mech_angles * erevs_per_mrev)

    # Generate electrical angle correction table
    elec_ang_corr_scale = np.ptp(table_elec_angle_residuals) / 254
    elec_ang_corr_offset = (np.min(table_elec_angle_residuals) + np.max(table_elec_angle_residuals)) / 2
    elec_ang_corr_values = np.round((table_elec_angle_residuals - elec_ang_corr_offset) / elec_ang_corr_scale).astype(np.int8)

    if args.plot:
        plt.plot(table_mech_angles, table_elec_angle_residuals, label='Floating point')
        plt.plot(table_mech_angles, elec_ang_corr_values * elec_ang_corr_scale + elec_ang_corr_offset, '.', label='int8 quantized')
        plt.xlabel('Mechanical angle (rad)')
        plt.ylabel('Electrical angle correction (rad)')
        plt.legend()
        plt.show()

    calibration_dict = OrderedDict()
    calibration_dict['angle'] = int(round(enc_tick_offset))
    calibration_dict['inv'] = int(erevs_per_mrev > 0)
    calibration_dict['epm'] = abs(erevs_per_mrev)
    calibration_dict['eac_type'] = 'int8'
    calibration_dict['eac_scale'] = elec_ang_corr_scale
    calibration_dict['eac_offset'] = elec_ang_corr_offset
    calibration_dict['eac_table'] = elec_ang_corr_values.tolist()

    print(json.dumps(calibration_dict))
