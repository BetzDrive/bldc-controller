#!/usr/bin/env python
from __future__ import print_function

import sys
import serial
import time
from math import sin, cos, pi
import argparse

from comms import *
from boards import *
from livegraph import livegraph

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Drive motor module(s) with a given control mode and plot current measurements.')
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

    def updateCurrent(i): 
        data = []
        for board_id in board_ids:
            try:
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
    labels.extend([['board ' + str(bid) + ' iq', 'board ' + str(bid) + ' iq command'] for bid in board_ids])
    labels = flatten(labels)
    graph = livegraph(updateCurrent, labels, sample_interval=1, window_size = 2000)

    graph.start()
