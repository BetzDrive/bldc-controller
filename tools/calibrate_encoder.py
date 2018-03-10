#!/usr/bin/env python

from __future__ import division
import argparse
from comms import *
import serial
import time
import sys
import numpy as np
from scipy import signal as sps, stats, interpolate
# import matplotlib.pyplot as plt

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate the encoder on a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('duty_cycle', type=float, help='Duty cycle')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, duty_cycle=0.6)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    client = BLDCControllerClient(ser, protocol_v2=True)

    client.leaveBootloader(args.board_id)
    time.sleep(0.1) # Wait for the controller to reset

    count = 0
    duty_cycle = args.duty_cycle
    phase_state_list = [(1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (1, 0, 1)]

    angles = []

    client.writeRegisters(args.board_id, 0x1030, 1, struct.pack('<H', 3000)) # Control watchdog timeout
    client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', 0, 0, 0))
    client.writeRegisters(args.board_id, 0x2000, 1, struct.pack('<B', 1))

    while True:
        a, b, c = phase_state_list[count % 6]

        client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', a * duty_cycle, b * duty_cycle, c * duty_cycle))

        time.sleep(2 if count == 0 else 0.1)

        angle = struct.unpack('<H', client.readRegisters(args.board_id, 0x3010, 1))[0]

        if count > 4 and abs(angles[0] - angle) < abs(angles[1] - angles[0]) / 3.0:
            break

        angles.append(angle)

        count += 1

    client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', 0, 0, 0))

    ser.close()

    print angles

    # angles = [14532, 14718, 14912, 15106, 15298, 15496, 15694, 15886, 16084, 16284, 92, 284, 488, 681, 873, 1074, 1261, 1453, 1655, 1850, 2040, 2238, 2430, 2622, 2820, 3017, 3209, 3412, 3602, 3799, 3998, 4190, 4384, 4585, 4779, 4968, 5170, 5359, 5551, 5748, 5945, 6136, 6329, 6525, 6719, 6917, 7109, 7305, 7504, 7697, 7893, 8092, 8285, 8479, 8682, 8874, 9063, 9268, 9461, 9650, 9852, 10047, 10238, 10435, 10631, 10819, 11017, 11210, 11406, 11603, 11797, 11992, 12192, 12382, 12575, 12781, 12966, 13159, 13364, 13552, 13743, 13941, 14135, 14324]
    # step_count = len(angles)
    # angle_period = 2**14
    # angle_slope = angle_period / step_count

    # # Move wraparound to edge of data
    # wrap_index = np.argmin(angles)
    # angles = np.roll(angles, -wrap_index)
    # indices = np.r_[:step_count]

    # # Subtract expected trend
    # angle_residuals = angles - indices * angle_slope

    # # Apply smoothing
    # angle_residuals = sps.savgol_filter(angle_residuals, 31, 3, mode='wrap')
    # smoothed_angles = angle_residuals + indices * angle_slope

    # # Invert function
    # ext_smoothed_angles = np.concatenate(([smoothed_angles[-1] - angle_period], smoothed_angles, [smoothed_angles[0] + angle_period]))
    # ext_indices = np.concatenate(([indices[-1] - step_count], indices, [indices[0] + step_count]))
    # table_angles = np.r_[:angle_period]
    # step_residuals = np.interp(table_angles, ext_smoothed_angles, ext_indices) - table_angles / angle_slope

    # # Build table
    # offset = 0.5 * (np.min(step_residuals) + np.max(step_residuals))
    # step_residuals -= offset
    # scale = np.max(step_residuals) / 127
    # table = np.round(step_residuals / scale).astype(np.int8)

    # # for i in range(len(table)):
    # #     sys.stdout.write('{:d}, '.format(table[i]))

    # print ''
    # print 'offset:', offset
    # print 'scale:', scale

    # plt.subplot(2, 1, 1)
    # plt.plot(angle_residuals)
    # plt.title("Angle residuals")
    # plt.xlabel("Step")
    # plt.ylabel("Angle (encoder counts)")
    # plt.subplot(2, 1, 2)
    # plt.plot(step_residuals)
    # plt.title("Step residuals")
    # plt.xlabel("Angle (encoder counts)")
    # plt.ylabel("Step")
    # plt.tight_layout()
    # plt.show()

    # print 'step residuals:', step_residuals

    # erpm_per_revolution = count / 6
    # phase_aligned_angle = angles[0]
    # print("ERPM per rev:\t" + str(erpm_per_revolution))
    # print("Phase aligned angle:\t" + str(phase_aligned_angle))
    # print(angles)
    # print(count)
