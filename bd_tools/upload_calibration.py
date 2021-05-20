#!/usr/bin/env python3

import argparse
import serial
import time
import json
import struct
import ast

from bd_tools import boards, comms


def parser_args():
    parser = argparse.ArgumentParser(
        description="Upload calibration values to motor driver board(s)"
    )
    parser.add_argument("serial", type=str, help="Serial port")
    parser.add_argument("--baud_rate", type=int, help="Serial baud rate")
    parser.add_argument("board_ids", type=str, help="Board id(s) to flash")
    parser.add_argument(
        "--calibration_file",
        type=str,
        help="The file which the calibration(s) is/are in",
    )
    parser.set_defaults(
        baud_rate=comms.COMM_DEFAULT_BAUD_RATE, calibration_file="calibrations.json"
    )
    return parser.parse_args()


def action(args):
    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.2)
    ser.reset_input_buffer()

    make_list = lambda x: list(x) if (type(x) == list or type(x) == tuple) else [x]
    make_int = lambda x: [int(y) for y in x]
    board_ids = make_int(make_list(ast.literal_eval(args.board_ids)))

    # Load in Custom Values
    with open(args.calibration_file) as json_file:
        calibration = json.load(json_file)

    client = comms.BLDCControllerClient(ser)

    initialized = boards.initBoards(client, board_ids)

    client.resetInputBuffer()

    if initialized:
        for board_id, calib in zip(board_ids, calibration):
            print(board_id, "-", calib)
            client.leaveBootloader([board_id])

            # Reset Calibration on Board
            client.clearCalibration([board_id])

            boards.loadCalibrationFromJSON(client, board_id, calib)

            client.setWatchdogTimeout([board_id], [1000])

            # Setting gains for motor
            client.setDirectCurrentKp([board_id], [0.5])
            client.setDirectCurrentKi([board_id], [0.1])
            client.setQuadratureCurrentKp([board_id], [1.0])
            client.setQuadratureCurrentKi([board_id], [0.2])

            # Store Calibration struct to Parameters
            client.storeCalibration([board_id])

    ser.close()


if __name__ == "__main__":
    action(parser_args())
