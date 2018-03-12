#!/usr/bin/env python

import argparse
from comms import *
import serial
import time

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload firmware to a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('bin_file', type=str, help='.bin file containing firmware image')
    parser.add_argument('--offset', type=int, help='Offset address for firmware image')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_FIRMWARE_OFFSET)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)
    # ser.reset_input_buffer()

    client = BLDCControllerClient(ser, protocol_v2=True)

    client.enterBootloader(args.board_id)
    time.sleep(0.2) # Wait for the controller to reset
    ser.reset_input_buffer()

    flash_sector_map = client.getFlashSectorMap(args.board_id)

    with open(args.bin_file, 'rb') as bin_file:
        firmware_image = bin_file.read()

    success = client.writeFlash(args.board_id, args.offset, firmware_image, sector_map=flash_sector_map, print_progress=True)

    if success:
        # client.leaveBootloader(args.board_id)
        print "Success"
    else:
        print "Failed"

    ser.close()
