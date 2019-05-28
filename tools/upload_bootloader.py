#!/usr/bin/env python
from comms import *
from boards import *

import argparse
import serial
import time

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload bootloader to a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board IDs')
    parser.add_argument('bin_file', type=str, help='.bin file containing bootloader image')
    parser.add_argument('--offset', type=int, help='Offset address for bootloader image')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    board_ids = [int(bid) for bid in args.board_ids.split(',')]

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.1)

    client = BLDCControllerClient(ser)

    initialized = initBoards(client, board_ids)

    ser.reset_input_buffer()

    if initialized:
        for board_id in board_ids:
            client.leaveBootloader([board_id])
            time.sleep(0.2) # Wait for the controller to reset
            ser.reset_input_buffer()

            flash_sector_maps = client.getFlashSectorMap([board_id])

            with open(args.bin_file, 'rb') as bin_file:
                firmware_image = bin_file.read()

            success = client.writeFlash([board_id], args.offset, firmware_image, sector_map=flash_sector_maps, print_progress=True)

            print 'Board {}'.format(board_id)
            if success:
                print "Success"
            else:
                print "Failed"

    ser.close()
