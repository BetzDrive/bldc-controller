#!/usr/bin/env python
from __future__ import print_function

from comms import *
from boards import *

import argparse
import serial
import time
import json
import struct

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload calibration values to a motor driver board')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=str, help='Board id to flash or all')
    parser.add_argument('--calibration_file', type=str, help='The file which the calibrations were put in')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, calibration_file='calibrations.json')
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.2)
    ser.reset_input_buffer()
    board_id = int(args.board_id)

    client = BLDCControllerClient(ser)

    initialized = initBoards(client, [board_id])
        
    client.resetInputBuffer()

    if initialized:
        # Reset Calibration on Board
        client.clearCalibration([board_id])

        # Load in Custom Values
        with open(args.calibration_file) as json_file:
            data = json.load(json_file)
        loadCalibrationFromJSON(board_id, data)

        # Store Calibration struct to Parameters
        client.storeCalibration([board_id])

    ser.close()
