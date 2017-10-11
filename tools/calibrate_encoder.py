#!/usr/bin/env python

import argparse
from comms import *
import serial
import time

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate the encoder on a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('duty_cycle', type=float, help='Duty cycle')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_DEFAULT_FIRMWARE_OFFSET, duty_cycle=0.6)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    client = BLDCControllerClient(ser)

    client.leaveBootloader(args.board_id)
    time.sleep(0.1) # Wait for the controller to reset

    count = 0
    duty_cycle = args.duty_cycle
    phase_state_list = [(1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (1, 0, 1)]

    angles = []

    while True:
        a, b, c = phase_state_list[count % 6]

        client.writeRegisters(args.board_id, 0x0102, 4, struct.pack('<Bfff', 1, a * duty_cycle, b * duty_cycle, c * duty_cycle))

        time.sleep(2 if count == 0 else 0.1)

        angle = struct.unpack('<H', client.readRegisters(args.board_id, 0x100, 1))[0]
        angles.append(angle)

        if count > 4 and abs(angles[0] - angle) < abs(angles[1] - angles[0]) / 3.0:
            break

        count += 1

    client.writeRegisters(args.board_id, 0x0102, 4, struct.pack('<Bfff', 1, 0, 0, 0))

    erpm_per_revolution = count / 6
    phase_aligned_angle = angles[0]
    print("ERPM per rev:\t" + str(erpm_per_revolution))
    print("Phase aligned angle:\t" + str(phase_aligned_angle))
    print(angles)
    print(count)

    ser.close()
