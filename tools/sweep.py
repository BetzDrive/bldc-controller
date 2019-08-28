#!/usr/bin/env python
from __future__ import print_function

import sys
import serial
import time
from math import sin, cos, pi
import argparse
import ast

from comms import *
from boards import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Drive motor between a set of positions with a delta and given frequency.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('pos_targets', type=str, help='Position (rad) targets')
    parser.add_argument('steps', type=int, help='Number of steps to split movement into')
    parser.add_argument('frequency', type=int, help='Frequency (Hz) at which next pos is sent')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    make_list = lambda x: list(x) if (type(x) == list or type(x) == tuple) else [x]
    pos_targets = make_list(ast.literal_eval(args.pos_targets))

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.04)

    board_id = [args.board_id]

    client = BLDCControllerClient(ser)
    initialized = initBoards(client, board_id)
    client.leaveBootloader(board_id)

    client.resetInputBuffer()

    initMotor(client, board_id)

    interval = 1.0/args.frequency
    last_time = time.time()

    while True:
        last_target = 0
        for target in pos_targets:
            diff = target - last_target
            curr_target = [last_target*1.0]

            for _ in range(args.steps):
                curr_target[0] += diff * 1.0 / args.steps
                try:
                    driveMotor(client, board_id, curr_target, 'position')
                except (ProtocolError, struct.error):
                    print("Failed to communicate with board: ", board_id)
                    pass

                curr_time = time.time()
                t_diff = curr_time - last_time
                time_to_sleep = interval - t_diff
                #time.sleep(time_to_sleep)
                time.sleep(0.5)
                last_time = curr_time
            last_target = target
