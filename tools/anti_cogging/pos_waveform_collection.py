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
    
    counts_per_rev = 2 << 14
    steps = np.arange(0, 2*np.pi, 2*np.pi / counts_per_rev)

    data = []
    driveMotor(client, [board_id], [0], "position")
    time.sleep(1)
    for i, cmd_pos in enumerate(steps):
        success = False
        while not success:
            try:
                driveMotor(client, [board_id], [cmd_pos], "position")
                act_pos = client.getRotorPosition([board_id])[0]
                voltage = client.getVoltage([board_id])[0]
                quad_voltage = client.getTargetQuadratureVoltage([board_id])[0]
                duty = quad_voltage / voltage
                current = client.getTargetQuadratureCurrent([board_id])[0]
                print(cmd_pos, act_pos, duty, voltage, current)
                data.append([cmd_pos, act_pos, duty, voltage, current])
                success = True
            except (ProtocolError, struct.error):
                print("Failed to communicate with board: ", board_id)

    if args.file_name:
        with open(args.file_name, 'wb') as file:
            pickle.dump(data, file)
        print("dumped data to file " + args.file_name)
    else:
        pp = pprint.PrettyPrinter()
        pp.pprint(arr[0])


