#!/usr/bin/env python

import argparse
from comms import *
import serial
import time

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload firmware to a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=str, help='Board ID')
    parser.add_argument('bin_file', type=str, help='.bin file containing firmware image')
    parser.add_argument('--offset', type=int, help='Offset address for firmware image')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_FIRMWARE_OFFSET)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.1)
    # ser.reset_input_buffer()

    client = BLDCControllerClient(ser)

    try:
        board_ids = [int(args.board_id)]
    except ValueError:
        board_ids = [int(board_id_str) for board_id_str in args.board_id.split(',')]
    
    for board_id in board_ids:
        client.enterBootloader([board_id])
        time.sleep(0.2) # Wait for the controller to reset
        ser.reset_input_buffer()

        flash_sector_maps = client.getFlashSectorMap([board_id])

        with open(args.bin_file, 'rb') as bin_file:
            firmware_image = bin_file.read()

        success = client.writeFlash([board_id], args.offset, firmware_image, sector_map=flash_sector_maps, print_progress=True)

        print 'Board {}'.format(board_id)

        if success:
            # client.leaveBootloader(board_id)
            print "Success"
        else:
            print "Failed"

    ser.close()
