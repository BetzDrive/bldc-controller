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
from livegraph import livegraph

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Drive motor module(s) with a given control mode and plot current measurements.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board ID (separate with comma)')
    parser.add_argument('mode', type=str, help='Control mode: \
                                                current (Id[A], Iq[A]), \
                                                raw_pwm(dc,dc,dc), \
                                                torque (N*m), \
                                                velocity (rad/s), \
                                                position (rad), \
                                                pos_vel (rad,rad/s), \
                                                pwm (dc)')
    parser.add_argument('actuations', type=str, help='Actuation amount in the units of the selected mode (if requires multiple args, separate by comma)')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    make_list = lambda x: x if (type(x) == list) else [x]
    board_ids  = make_list(ast.literal_eval(args.board_ids))
    actuations = make_list(ast.literal_eval(args.actuations))

    mode = args.mode

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)

    client = BLDCControllerClient(ser)
    initialized = initBoards(client, board_ids)
    
    client.resetInputBuffer()

    loadMotorCalibration(client, board_ids, mode)

    def updateCurrent(i): 
        data = []
        for board_id in board_ids:
            try:
                driveMotor(client, board_ids, actuations, mode)
                # Read the iq calulated
                read = struct.unpack('<f', client.readRegisters([board_id], [0x3003], [1])[0])
                data.append(read)
                # Read the iq command
                read = struct.unpack('<f', client.readRegisters([board_id], [0x3011], [1])[0])
                data.append(read)
            except (ProtocolError, struct.error):
                #print("Failed to communicate with board: ", board_id)
                data.append([0.0])
                data.append([0.0])
        return i, data

    flatten = lambda l: [item for sublist in l for item in sublist]

    labels = []
    labels.extend([[str(bid) + '\'s iq Reading', str(bid) + '\'s iq PID output'] for bid in board_ids])
    labels = flatten(labels)
    graph = livegraph(updateCurrent, labels, sample_interval=1, window_size = 2000)

    graph.start()
