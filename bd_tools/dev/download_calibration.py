#!/usr/bin/env python
from __future__ import print_function

from comms import *
from boards import *

import argparse
import serial
import time
import json
import struct

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Download json calibrations off a set of boards!"
    )
    parser.add_argument("serial", type=str, help="Serial port")
    parser.add_argument("--baud_rate", type=int, help="Serial baud rate")
    parser.add_argument("board_ids", type=str, help="Board ids to read from")
    parser.add_argument(
        "--calibration_file",
        type=str,
        help="The file which the calibrations will be put in",
    )
    parser.set_defaults(
        baud_rate=COMM_DEFAULT_BAUD_RATE,
        calibration_file="downloaded_calibrations.json",
    )
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.2)
    ser.reset_input_buffer()

    board_ids = [int(bid) for bid in args.board_ids.split(",")]

    client = BLDCControllerClient(ser)

    initialized = initBoards(client, board_ids)

    client.resetInputBuffer()

    data = []
    if initialized:
        for board_id in board_ids:
            calibration = client.readCalibration([board_id])
            data.append(calibration)
            print(calibration)

    ser.close()

    with open(args.calibration_file, "w") as outfile:
        json.dump(data, outfile)
