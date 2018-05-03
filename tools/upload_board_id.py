#!/usr/bin/env python

import argparse
from comms import *
import serial
import time

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Change the board ID of a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('new_board_id', type=int, help='New board ID')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.1)

    client = BLDCControllerClient(ser, protocol_v2=True)

    client.enterBootloader(args.board_id)
    time.sleep(0.2) # Wait for the controller to reset
    ser.reset_input_buffer()

    flash_sector_map = client.getFlashSectorMap(args.board_id)

    success = client.eraseFlash(args.board_id, COMM_NVPARAMS_OFFSET, 1, sector_map=flash_sector_map)

    success = success and client.programFlash(args.board_id, COMM_NVPARAMS_OFFSET, chr(args.new_board_id))

    if success:
        client.resetSystem(args.new_board_id)
        print "Success"
    else:
        print "Failed"

    ser.close()
