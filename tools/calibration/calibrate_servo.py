#!/usr/bin/env python
from __future__ import division

import sys
sys.path.append("..")

import argparse
from comms import *
import serial
import time
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
enc_ang_corr_table_size = 257

def midpoint_clamping_svm(angle, magnitude):
    dc = magnitude * np.cos(np.array([angle, angle - (2 / 3 * pi), angle - (4 / 3 * pi)])) / np.sqrt(3) + 0.5
    shift = 0.5 * (1.0 - np.min(dc) - np.max(dc))
    return dc + shift

def circular_mean(a, b):
    return np.arctan2(0.5 * (np.sin(a) + np.sin(b)), 0.5 * (np.cos(a) + np.cos(b)))

def wrap_pi(a):
    return (a + pi) % (2 * pi) - pi

def interp_periodic_periodic(x, xp, fp):
    '''
    Interpolates a periodic function of a periodic argument
    '''

    # Wrap x, xp into range [0, 2*pi)
    x = x % (2 * pi)
    xp = xp % (2 * pi)

    # Sort by xp
    sort_indices = np.argsort(xp)
    xp = xp[sort_indices]
    fp = fp[sort_indices]

    # Unwrap fp
    fp = np.unwrap(fp)

    # Extend to avoid boundary issues
    xp_ext = np.concatenate([xp[-1:] - 2 * pi, xp, xp[:1] + 2 * pi])
    fp_ext = np.unwrap(np.concatenate([fp[-1:], fp, fp[:1]]))

    f_interp = np.interp(x, xp_ext, fp_ext, left=np.nan, right=np.nan)

    assert not np.isnan(np.sum(f_interp))

    return f_interp

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate calibration parameters for a single servo (motor and controller board).')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('duty_cycle', type=float, help='Duty cycle')
    parser.add_argument('--max_steps', type=int, help='Maximum number of steps')
    parser.add_argument('--delay', type=float, help='Delay between steps')
    parser.add_argument('--start_delay', type=float, help='Delay before first step')
    parser.add_argument('--plot', dest='plot', action='store_true', help='Plot the encoder angle compensation table')
    parser.add_argument('--no-plot', dest='plot', action='store_false', help='Don\'t plot the encoder angle compensation table')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, duty_cycle=0.6, max_steps=126, delay=0.05, start_delay=0.5, plot=False)
    args = parser.parse_args()

    #
    # Data collection
    #

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)

    client = BLDCControllerClient(ser)

    client.leaveBootloader([args.board_id])
    time.sleep(0.2) # Wait for the controller to reset
    ser.reset_input_buffer()

    def set_phase_state(step):
        angle = (step % steps_per_erev) / steps_per_erev * 2 * np.pi
        a, b, c = midpoint_clamping_svm(angle, args.duty_cycle)
        client.writeRegisters([args.board_id], [0x2003], [3], [struct.pack('<fff', a, b, c)])

    client.writeRegisters([args.board_id], [0x1030], [1], [struct.pack('<H', 3000)]) # Control timeout
    client.writeRegisters([args.board_id], [0x2003], [3], [struct.pack('<fff', 0, 0, 0)])
    client.writeRegisters([args.board_id], [0x2000], [1], [struct.pack('<B', 1)])

    # Start one step before phase A to avoid boundary issues
    set_phase_state(-1)
    time.sleep(args.start_delay)

    # Forward
    forward_raw_angles = []
    for i in range(args.max_steps):
        set_phase_state(i)
        time.sleep(args.delay)

        raw_angle = struct.unpack('<H', client.readRegisters([args.board_id], [0x3010], [1])[0])[0]

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

        raw_angle = struct.unpack('<H', client.readRegisters([args.board_id], [0x3010], [1])[0])[0]

        backward_raw_angles.append(raw_angle)

    client.writeRegisters([args.board_id], [0x2003], [3], [struct.pack('<fff', 0, 0, 0)])
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

    # Average forward and backward angles, taking phase wrapping into account
    enc_angles = circular_mean(forward_angles, backward_angles)

    # Move smallest encoder angle aligned with phase A to the front, for consistency
    smallest_index = np.argmin(enc_angles[::steps_per_erev] % (2 * pi)) * steps_per_erev
    enc_angles = np.roll(enc_angles, -smallest_index)

    # Find the most likely true encoder angles
    enc_angles_slope = np.sign(np.diff(np.unwrap(enc_angles))[0])
    enc_angles_trend = np.linspace(0, 2 * pi, step_count, endpoint=False) * enc_angles_slope
    enc_angles_start = np.mean(np.unwrap(enc_angles) - enc_angles_trend)
    true_enc_angles = enc_angles_start + enc_angles_trend

    # Build encoder angle compensation table
    interp_enc_angles = np.linspace(0, 2 * pi, enc_ang_corr_table_size, endpoint=False)
    interp_true_enc_angles = interp_periodic_periodic(interp_enc_angles, enc_angles, true_enc_angles)
    interp_true_enc_angle_diff = wrap_pi(interp_true_enc_angles - interp_enc_angles)
    eac_scale = np.ptp(interp_true_enc_angle_diff) / 254
    eac_offset = (np.min(interp_true_enc_angle_diff) + np.max(interp_true_enc_angle_diff)) / 2
    eac_table = np.round((interp_true_enc_angle_diff - eac_offset) / eac_scale).astype(np.int8)

    if args.plot:
        plt.figure()
        plt.plot(interp_enc_angles, interp_true_enc_angle_diff, label='float')
        plt.plot(interp_enc_angles, eac_scale * eac_table + eac_offset, '.', label='int8')
        plt.title('Encoder angle compensation')
        plt.xlabel('Raw encoder angle (rad)')
        plt.ylabel('Encoder angle compensation (rad)')
        plt.legend()
        plt.show()

    # Use the first true encoder angle (which is aligned with phase A) as the start of a mechanical revolution
    mech_rev_start = true_enc_angles[0]

    erev_start = (mech_rev_start % (2 * pi)) / (2 * pi) * encoder_ticks_per_rev
    erevs_per_mrev = int(round(step_count / steps_per_erev))

    calibration_dict = OrderedDict()
    calibration_dict['angle'] = int(round(erev_start))
    calibration_dict['inv'] = int(enc_angles_slope > 0)
    calibration_dict['epm'] = erevs_per_mrev
    calibration_dict['eac_type'] = 'int8'
    calibration_dict['eac_scale'] = eac_scale
    calibration_dict['eac_offset'] = eac_offset
    calibration_dict['eac_table'] = eac_table.tolist()

    print(json.dumps(calibration_dict))
