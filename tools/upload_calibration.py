#!/usr/bin/env python
from comms import *
from boards import *

import argparse
import serial
import time
import json
import struct

def is_int(i):
    try:
        int(i)
        return True
    except ValueError:
        return False

def flash_board(client, board_id, data):
    flash_sector_map = client.getFlashSectorMap([board_id])

    success = client.eraseFlash([board_id], COMM_NVPARAMS_OFFSET, 1, sector_map=flash_sector_map)

    buf = struct.pack('<H', len(data)) + data

    success = success and client.programFlash([board_id], COMM_NVPARAMS_OFFSET, buf)

    l = struct.unpack('<H', client.readFlash([board_id], COMM_NVPARAMS_OFFSET, 2))[0]
    d = client.readFlash([board_id], COMM_NVPARAMS_OFFSET+2, l)

    if success and d == data:
        print("Success", board_id)
        print("Wrote:")
        time.sleep(0.2)
        ser.reset_input_buffer()
        print(json.loads(d))
    else:
        print("Failed ", board_id)

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

    client = BLDCControllerClient(ser)

    initialized = initBoards(client, [int(args.board_id)])

    if initialized:
        with open(args.calibration_file) as json_file:
            data = json.load(json_file)
        flash_board(client, int(args.board_id), json.dumps(data, separators=(',', ':')))

    ser.close()
