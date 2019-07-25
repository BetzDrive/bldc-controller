#!/usr/bin/env python
from __future__ import print_function

import sys
sys.path.append("..")

import serial
import time
import numpy as np
import argparse
import ast
import pickle

from comms import *
from boards import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Collection Algorithm 1 from UPenn Anti-cogging.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('file_name', type=str, help='File name to record data')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    board_id = args.board_id
    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.04)

    client = BLDCControllerClient(ser)
    initialized = initBoards(client, [board_id])
    
    client.leaveBootloader([board_id])

    client.resetInputBuffer()

    initMotor(client, [board_id])
    
    data = []
    driveMotor(client, [board_id], [0], "pwm")
    time.sleep(0.1)

    d_min_static = 0     # Minimum duty cycle to move motor from stop
    d_min_kinetic = 0    # Minimum duty cycle to move motor while spinning
    d_del_min = 0.0001   # Minimum change in duty cycle per sample
    d_min = 0

    print("Attempting to break static friction (stiction) with minimum delta PWM of", d_del_min)
    pos = client.getRotorPosition([board_id])[0]
    last_pos = pos
    while abs(pos - last_pos) < 0.1:
        try:
            driveMotor(client, [board_id], [d_min], "pwm")
            time.sleep(0.1)
            last_pos = pos
            pos = client.getRotorPosition([board_id])[0]
            d_min += d_del_min
        except (ProtocolError, struct.error):
            pass

    d_min_static = d_min

    print("Attempting to detect kinetic friction with minimum delta PWM of", d_del_min)
    start_pos = last_pos
    while (pos - last_pos) is not 0:
        while abs(start_pos - start_pos) / (2*np.pi) < 1:
            try:
                driveMotor(client, [board_id], [d_min], "pwm")
                time.sleep(0.1)
                last_pos = pos
                pos = client.getRotorPosition([board_id])[0]
            except (ProtocolError, struct.error):
                pass
        d_min -= d_del_min

    d_min_kinetic = d_min

    time.sleep(0.5)
    print("Testing to break static friction:", d_min_static)
    for _ in range(5):
        try:
            driveMotor(client, [board_id], [d_min_static], "pwm")
            time.sleep(0.2)
        except (ProtocolError, struct.error):
            pass

    print("Testing for continued rotation at min kinetic:", d_min_kinetic)
    for _ in range(5):
        try:
            driveMotor(client, [board_id], [d_min_kinetic], "pwm")
            time.sleep(0.2)
        except (ProtocolError, struct.error):
            pass

    print("Logging data at min kinetic")
    pos = client.getRotorPosition([board_id])[0]
    start_pos = pos
    revs = (2) * 2*np.pi
    j = 0
    data = []
    while pos - start_pos < revs:
        try:
            driveMotor(client, [board_id], [d_min_kinetic], "pwm")
            j = j + 1
            pos = client.getRotorPosition([board_id])[0]
            vel = client.getRotorVelocity([board_id])[0]
            t = time.time()
            data.append([pos, vel, t, j])
        except (ProtocolError, struct.error):
            pass

    if args.file_name:
        with open(args.file_name, 'wb') as file:
            pickle.dump(data, file)
        print("dumped data to file " + args.file_name)
    else:
        pp = pprint.PrettyPrinter()
        pp.pprint(arr[0])
