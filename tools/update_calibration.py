#!/usr/bin/env python
from __future__ import print_function

from comms import *
from boards import *

import argparse
import serial
import time
import json
import struct
import ast

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload calibration values to motor driver board(s)')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board id(s) to flash')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.2)
    ser.reset_input_buffer()

    make_list = lambda x: list(x) if (type(x) == list or type(x) == tuple) else [x]
    make_int = lambda x: [int(y) for y in x]
    board_ids = make_int(make_list(ast.literal_eval(args.board_ids)))

    client = BLDCControllerClient(ser)

    initialized = initBoards(client, board_ids)
        
    client.resetInputBuffer()

    if initialized:
        for board_id in board_ids:
            client.leaveBootloader([board_id])

            client.setWatchdogTimeout([board_id], [1000])
    
            # Setting gains for motor
            client.setDirectCurrentKp([board_id], [0.5])
            client.setDirectCurrentKi([board_id], [0.1])
            client.setQuadratureCurrentKp([board_id], [1.0])
            client.setQuadratureCurrentKi([board_id], [0.2])
            client.setVelocityKp([board_id], [0.1])
            client.setVelocityKi([board_id], [1e-3])
            client.setPositionKp([board_id], [5.0])
            client.setPositionKi([board_id], [0.0])

            # Modifying Limits
            client.setCurrentLimit([board_id], [2.0])
            client.setVelocityLimit([board_id], [10.0])

            # Store Calibration struct to Parameters
            client.storeCalibration([board_id])

            print("Updated:", board_id)

    print("Finished Updating Calibrations")

    ser.close()
