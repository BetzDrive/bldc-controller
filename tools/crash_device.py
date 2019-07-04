#!/usr/bin/env python2
from __future__ import print_function

import serial
import time
import argparse

from comms import *
from boards import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Read temperature sensor from boards.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board ID (separate with comma)')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.004)
    
    board_ids = [int(bid) for bid in args.board_ids.split(',')]

    client = BLDCControllerClient(ser)

    while(True):
        initialized = initBoards(client, board_ids)
        client.leaveBootloader(board_ids)
