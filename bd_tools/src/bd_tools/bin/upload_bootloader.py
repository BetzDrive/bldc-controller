#!/usr/bin/env python3
import argparse
import time

import serial

from bd_tools import boards, comms


def parser_args():
    parser = argparse.ArgumentParser(
        description="Upload bootloader to a motor controller board."
    )
    boards.addBoardArgs(parser)
    parser.add_argument(
        "bin_file", type=str, help=".bin file containing bootloader image"
    )
    parser.add_argument(
        "--offset", type=int, help="Offset address for bootloader image"
    )
    parser.set_defaults(
        baud_rate=comms.COMM_DEFAULT_BAUD_RATE,
        offset=comms.COMM_BOOTLOADER_OFFSET,
    )
    return parser.parse_args()


def action(args):
    board_ids = [int(bid) for bid in args.board_ids.split(",")]

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
                crashed,
            )

        if not crashed:
            for board_id in board_ids:
                client.leaveBootloader([board_id])
                time.sleep(0.2)  # Wait for the controller to reset
                ser.reset_input_buffer()

                flash_sector_maps = client.getFlashSectorMap([board_id])

                with open(args.bin_file, "rb") as bin_file:
                    firmware_image = bin_file.read()

                success = False
                while not success:
                    try:
                        success = client.writeFlash(
                            [board_id],
                            args.offset,
                            firmware_image,
                            sector_map=flash_sector_maps,
                            print_progress=True,
                        )
                    except (
                        comms.MalformedPacketError,
                        comms.ProtocolError,
                    ) as e:
                        print(f"Upload to board {board_id} failed with error:")
                        print(e)
                        print("Retrying...")
                        continue

                if success:
                    print("Upload to Board", board_id, "Succeeded")
                else:
                    print("Upload to Board", board_id, "Failed")
                    break

    ser.close()


if __name__ == "__main__":
    action(parser_args())
