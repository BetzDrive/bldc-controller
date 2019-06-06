#!/usr/bin/env python
from __future__ import print_function

import sys
import serial
import time
from math import sin, cos, pi
import argparse

from comms import *
from boards import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Drive motor module(s) with a given control mode.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board ID (separate with comma)')
    parser.add_argument('mode', type=str, help='Control mode: foc (duty cycle [dc]), raw_pwm(dc,dc,dc), torque (N*m), velocity (rad/s), position (rad), pos_vel (rad,rad/s), pwm (dc)')
    parser.add_argument('actuation', type=str, help='Actuation amount in the units of the selected mode (if requires multiple args, separate by comma)')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    board_ids = [int(bid) for bid in args.board_ids.split(',')]
    duty_cycles = [float(dc) for dc in args.actuation.split(',')]

    mode = args.mode

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)

    client = BLDCControllerClient(ser)
    initialized = initBoards(client, board_ids)
    
    client.resetInputBuffer()

    loadMotorCalibration(client, board_ids, duty_cycles, mode)
    
    start_time = time.time()
    count = 0
    rollover = 1000
    while True:
        for board_id in board_ids:
            try:
                client.writeRegisters([board_id], [0x2000], [1], [struct.pack('<B', 2)]) # Torque control
            except (ProtocolError, struct.error):
                print("Failed to communicate with board: ", board_id)
                pass
    
        count += 1
        if count % rollover == 0:
            now = time.time()
            diff = (now - start_time)
            print(diff)
            freq = rollover / diff
            print("Comm Frequency:{}".format(freq))
            start_time = now
            sys.stdout.flush()
