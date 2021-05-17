#!/usr/bin/env python
import argparse
import serial
import time

from tools import boards, comms


def parser_args():
    parser = argparse.ArgumentParser(
        description='Upload firmware to a motor controller board.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board IDs')
    parser.add_argument('bin_file',
                        type=str,
                        help='.bin file containing firmware image')
    parser.add_argument('--offset',
                        type=int,
                        help='Offset address for firmware image')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE,
                        offset=COMM_FIRMWARE_OFFSET)
    return parser.parse_args()


def action(args):
    board_ids = [int(bid) for bid in args.board_ids.split(',')]

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.1)

    client = comms.BLDCControllerClient(ser)

    initialized = boards.initBoards(client, board_ids)

    ser.reset_input_buffer()

    if initialized:
        crashed = client.checkWDGRST()
        if crashed:
            print(
                "Some boards have crashed, please power cycle before upload:",
                crashed)

        if not crashed:
            for board_id in board_ids:
                flash_sector_maps = client.getFlashSectorMap([board_id])

                with open(args.bin_file, 'rb') as bin_file:
                    firmware_image = bin_file.read()

                success = False
                while not success:
                    try:
                        success = client.writeFlash(
                            [board_id],
                            args.offset,
                            firmware_image,
                            sector_map=flash_sector_maps,
                            print_progress=True)
                    except (comms.MalformedPacketError,
                            comms.ProtocolError) as e:
                        print(f'Upload to board {board_id} failed with error:')
                        print(e)
                        print('Retrying...')
                        continue

                if success:
                    print("Upload to Board", board_id, "Succeeded")
                else:
                    print("Upload to Board", board_id, "Failed")
                    break

    ser.close()


if __name__ == '__main__':
    action(parser_args())
