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

# 14-bit encoder
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

    client = BLDCControllerClient(ser)

    client.enterBootloader([args.board_id])
    time.sleep(0.2)
    try:
        print (client.enumerateBoards([args.board_id]))
    except:
        print("Failed to receive enumerate response")
    time.sleep(0.2)

    client.leaveBootloader([args.board_id])
    time.sleep(0.2) # Wait for the controller to reset
    ser.reset_input_buffer()

    def set_phase_state(phase_state):
        a, b, c = phase_state
        client.writeRegisters([args.board_id], [0x2003], [3], [struct.pack('<fff', a * args.duty_cycle, b * args.duty_cycle, c * args.duty_cycle)])


    client.writeRegisters([args.board_id], [0x1030], [1], [struct.pack('<H', 1000)]) # Control watchdog timeout
    client.writeRegisters([args.board_id], [0x2003], [3], [struct.pack('<fff', 0, 0, 0)])
    client.writeRegisters([args.board_id], [0x2000], [1], [struct.pack('<B', 1)])

    time.sleep(args.delay)

    for i in range(args.max_steps):
        set_phase_state(phase_state_list[i % 6])
        time.sleep(args.delay)

        #raw_angle = struct.unpack('<H', client.readRegisters([args.board_id], [0x3010], [1])[0])[0]
        #print (raw_angle)



