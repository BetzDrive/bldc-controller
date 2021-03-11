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
    parser = argparse.ArgumentParser(description='Drive motor module(s) with a given control mode.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board ID (separate with comma)')
    parser.add_argument('mode', type=str, help='Control mode: \
                                                current (Id[A], Iq[A]), \
                                                phase (dc,dc,dc), \
                                                torque (N*m), \
                                                velocity (rad/s), \
                                                position (rad), \
                                                pos_vel (rad,rad/s), \
                                                pos_ff (rad,ff[A]), \
                                                pwm (dc)')
    parser.add_argument('actuations', type=str, help='Actuation amount in the units of the selected mode (if requires multiple args, separate by comma)')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    make_list = lambda x: list(x) if (type(x) == list or type(x) == tuple) else [x]
    make_type = lambda x, to_type: [to_type(y) for y in x]
    board_ids  = make_type(make_list(ast.literal_eval(args.board_ids)), int)
    actuations = make_list(ast.literal_eval(args.actuations))

    mode = args.mode

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)

    client = BLDCControllerClient(ser)
    initialized = initBoards(client, board_ids)

    client.leaveBootloader(board_ids)

    client.resetInputBuffer()

    initMotor(client, board_ids)

    start_time = time.time()
    count = 0
    rollover = 1000
    while True:
        # If there's a watchdog reset, clear the reset and perform any configuration again
        crashed = client.checkWDGRST()
        if crashed != []:
            try:
                client.clearWDGRST(crashed)
            except (ProtocolError, MalformedPacketError):
                pass

        try:
            driveMotor(client, board_ids, actuations, mode)
        except (ProtocolError, MalformedPacketError) as e:
            print(e)

        count += 1
        if count % rollover == 0:
            now = time.time()
            diff = (now - start_time)
            print(diff)
            freq = rollover / diff
            print("Comm Frequency:{}".format(freq))
            start_time = now
            sys.stdout.flush()
