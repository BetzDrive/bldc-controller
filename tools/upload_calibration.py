#!/usr/bin/env python

import argparse
from comms import *
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
    client.resetSystem(board_id)
    time.sleep(0.1) # Wait for the controller to reset

    old_board_id = client.readFlash(board_id, COMM_NVPARAMS_OFFSET, 1)

    flash_sector_map = client.getFlashSectorMap(board_id)

    success = client.eraseFlash(board_id, COMM_NVPARAMS_OFFSET, 1, sector_map=flash_sector_map)

    buf = old_board_id + struct.pack('<H', len(data)) + data
    print(len(data))

    success = success and client.programFlash(board_id, COMM_NVPARAMS_OFFSET, buf)

    l = struct.unpack('<H', client.readFlash(board_id, COMM_NVPARAMS_OFFSET+1, 2))[0]
    d = client.readFlash(board_id, COMM_NVPARAMS_OFFSET+3, l)

    if success and d == data:
        client.resetSystem(board_id)
        print("Success", board_id)
    else:
        print("Failed ", board_id)

calibrations = {}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Change the board ID of a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=str, help='Board id to flash or all')
    parser.add_argument('file', type=str, help='path to calibration file')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE)
    args = parser.parse_args()

    with open(args.file, 'r') as file:
        data = file.read()

    calibrations = json.loads(data)

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    client = BLDCControllerClient(ser, protocol_v2=True)
    if args.board_id == 'all':
        for id in calibrations:
            try:
                flash_board(client, int(id), json.dumps(calibrations[id], separators=(',', ':')))
            except TypeError:
                print("Failed %s" % id)
    elif is_int(args.board_id):
        flash_board(client, int(args.board_id), json.dumps(calibrations[args.board_id], separators=(',', ':')))

    ser.close()
