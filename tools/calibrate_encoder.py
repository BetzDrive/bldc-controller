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

# 14-bit encoder
encoder_ticks_per_rev = 2 ** 14
phase_state_list = [(1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (1, 0, 1)]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate the encoder on a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('duty_cycle', type=float, help='Duty cycle')
    parser.add_argument('--max_steps', type=int, help='Maximum number of steps')
    parser.add_argument('--delay', type=float, help='Delay between steps')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, duty_cycle=0.6, max_steps=126, delay=0.1)
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

    def set_phase_state(phase_state):
        a, b, c = phase_state
        client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', a * args.duty_cycle, b * args.duty_cycle, c * args.duty_cycle))

    client.writeRegisters(args.board_id, 0x1030, 1, struct.pack('<H', 3000)) # Control watchdog timeout
    client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', 0, 0, 0))
    client.writeRegisters(args.board_id, 0x2000, 1, struct.pack('<B', 1))

    # Start one step before phase A to avoid boundary issues
    set_phase_state(phase_state_list[-1])
    time.sleep(args.delay)

    # Forward
    forward_raw_angles = []
    for i in range(args.max_steps):
        set_phase_state(phase_state_list[i % 6])
        time.sleep(args.delay)

        raw_angle = struct.unpack('<H', client.readRegisters(args.board_id, 0x3010, 1))[0]

        if i > 4 and abs(forward_raw_angles[0] - raw_angle) < abs(forward_raw_angles[1] - forward_raw_angles[0]) / 3.0:
            break

        forward_raw_angles.append(raw_angle)

    step_count = len(forward_raw_angles)

    # Take one more step to avoid boundary issues
    set_phase_state(phase_state_list[step_count % 6])
    time.sleep(args.delay)

    # Backward
    backward_raw_angles = []
    for i in range(step_count - 1, -1, -1):
        set_phase_state(phase_state_list[i % 6])
        time.sleep(args.delay)

        raw_angle = struct.unpack('<H', client.readRegisters(args.board_id, 0x3010, 1))[0]

        backward_raw_angles.append(raw_angle)

    set_phase_state((0, 0, 0))
    ser.close()

    #
    # Data analysis
    #

    # Flip backward raw angles to correspond with forward raw angles
    backward_raw_angles = backward_raw_angles[::-1]

    forward_raw_angles = np.array(forward_raw_angles)
    backward_raw_angles = np.array(backward_raw_angles)

    # raw_angles = np.array([14532, 14718, 14912, 15106, 15298, 15496, 15694, 15886, 16084, 16284, 92, 284, 488, 681, 873, 1074, 1261, 1453, 1655, 1850, 2040, 2238, 2430, 2622, 2820, 3017, 3209, 3412, 3602, 3799, 3998, 4190, 4384, 4585, 4779, 4968, 5170, 5359, 5551, 5748, 5945, 6136, 6329, 6525, 6719, 6917, 7109, 7305, 7504, 7697, 7893, 8092, 8285, 8479, 8682, 8874, 9063, 9268, 9461, 9650, 9852, 10047, 10238, 10435, 10631, 10819, 11017, 11210, 11406, 11603, 11797, 11992, 12192, 12382, 12575, 12781, 12966, 13159, 13364, 13552, 13743, 13941, 14135, 14324])

    # Convert to radians
    forward_angles = forward_raw_angles / encoder_ticks_per_rev * 2 * np.pi
    backward_angles = backward_raw_angles / encoder_ticks_per_rev * 2 * np.pi

    # Phase unwrapping
    forward_angles = np.unwrap(forward_angles)
    backward_angles = np.unwrap(backward_angles)

    # Average forward and backward measurements
    angles = (forward_angles + backward_angles) / 2

    # Guess erevs/mrev
    angle_slope = np.mean(np.diff(angles))
    steps_per_mrev = int(np.round(2 * np.pi / angle_slope))
    erevs_per_mrev = steps_per_mrev // 6 # Could be negative

    # Convert to electrical angle
    elec_angles = angles * erevs_per_mrev

    # Subtract expected trend
    elec_angle_residuals = elec_angles - np.r_[:len(elec_angles)] * (2 * np.pi / 6)

    # Find smallest raw angle aligned with phase A
    elec_angle_offset = np.mean(elec_angle_residuals)
    wrapped_elec_angle_offset = elec_angle_offset % (2 * np.pi)
    if erevs_per_mrev < 0:
        wrapped_elec_angle_offset -= 2 * np.pi
    angle_offset = wrapped_elec_angle_offset / erevs_per_mrev
    erev_start = angle_offset / (2 * np.pi) * encoder_ticks_per_rev

    print('    erev_start: {:5d}'.format(int(round(erev_start))))
    print('erevs_per_mrev: {:5d}'.format(abs(erevs_per_mrev)))
    print('   flip_phases: {:5d}'.format(int(erevs_per_mrev > 0)))

    # plt.plot((elec_angle_residuals - elec_angle_offset) / erevs_per_mrev / (2 * np.pi) * encoder_ticks_per_rev)
    # plt.show()

    # # Apply smoothing
    # angle_residuals = sps.savgol_filter(angle_residuals, 31, 3, mode='wrap')
    # smoothed_angles = angle_residuals + indices * angle_slope
