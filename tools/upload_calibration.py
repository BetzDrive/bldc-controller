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
    old_board_id = client.readFlash([board_id], COMM_NVPARAMS_OFFSET, 1)

    flash_sector_map = client.getFlashSectorMap([board_id])

    success = client.eraseFlash([board_id], COMM_NVPARAMS_OFFSET, 1, sector_map=flash_sector_map)

    buf = old_board_id + struct.pack('<H', len(data)) + data

    success = success and client.programFlash([board_id], COMM_NVPARAMS_OFFSET, buf)

    l = struct.unpack('<H', client.readFlash([board_id], COMM_NVPARAMS_OFFSET+1, 2))[0]
    d = client.readFlash([board_id], COMM_NVPARAMS_OFFSET+3, l)

    if success and d == data:
        client.resetSystem([board_id])
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
    parser.add_argument('erev_start', type=int, help='Starting erev')
    parser.add_argument('epmrev', type=int, help='Electronic revolutions per magnetic revolution')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.2)
    ser.reset_input_buffer()

    client = BLDCControllerClient(ser)

    time.sleep(0.2)
    try:
        print (client.enumerateBoards([args.board_id]))
    except:
        print("Failed to receive enumerate response")
    time.sleep(0.2)

    ser.reset_input_buffer()

    size = str(raw_input("What size is the motor? (S/L)\n"))
    calibration = {
        "inv":int(args.epmrev>0),
        "epm":args.epmrev,
        "angle":args.erev_start,
        "torque": (1.45, 0.6)[size.lower() == 's'],
        "zero":0.0
        }
    flash_board(client, int(args.board_id), json.dumps(calibration, separators=(',', ':')))

    ser.close()
