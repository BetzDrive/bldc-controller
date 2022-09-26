#!/usr/bin/env python3
import argparse
import struct

import serial

from bd_tools import boards, comms, utils

ReadOnlyRegs = {}
ReadOnlyRegs["encoder"] = boards.COMM_ROR_ROTOR_POS
ReadOnlyRegs["encoder_raw"] = boards.COMM_ROR_ROTOR_POS_RAW
ReadOnlyRegs["velocity"] = boards.COMM_ROR_ROTOR_VEL
ReadOnlyRegs["id"] = boards.COMM_ROR_CURRENT_DIRECT
ReadOnlyRegs["iq"] = boards.COMM_ROR_CURRENT_QUADRATURE
ReadOnlyRegs["supply"] = boards.COMM_ROR_SUPPLY_V
ReadOnlyRegs["temp"] = boards.COMM_ROR_TEMPERATURE
ReadOnlyRegs["imu"] = boards.COMM_ROR_ACC_X


def parser_args():
    parser = argparse.ArgumentParser(description="Read a sensor from boards.")
    boards.addBoardArgs(parser)
    parser.add_argument(
        "--num_iters",
        type=int,
        help="Number of iterations to loop (default to infinity)",
    )
    parser.add_argument(
        "sensor",
        type=str,
        help="Choose sensor (encoder, encoder_raw, velocity, id, iq, supply, "
        "temp, imu)",
    )
    parser.set_defaults(
        baud_rate=comms.COMM_DEFAULT_BAUD_RATE,
        num_iters=0,
        offset=comms.COMM_BOOTLOADER_OFFSET,
    )
    return parser.parse_args()


def action(args):
    ser = serial.Serial(
        port=args.serial, baudrate=args.baud_rate, timeout=0.004
    )

    board_ids = [int(bid) for bid in args.board_ids.split(",")]

    client = comms.BLDCControllerClient(ser)

    boards.initBoards(client, board_ids)

    for bid in board_ids:
        client.leaveBootloader([bid])

    sen = args.sensor
    address = ReadOnlyRegs[sen]
    decode = "<f"
    num_regs = 1
    message = "{0}: {1[0]}"

    if sen == "encoder_raw":
        decode = "<H"
        message = "{0}: {1[0]} ticks"
    elif sen == "encoder":
        message = "{0}: {1[0]} radians"
    elif sen == "velocity":
        message = "{0}: {1[0]} rad/s"
    elif sen == "id":
        message = "{0}: {1[0]} amps"
    elif sen == "iq":
        message = "{0}: {1[0]} amps"
    elif sen == "supply":
        message = "{0}: {1[0]} volts"
    elif sen == "temp":
        message = "{0}: {1[0]} degC"
    if sen == "imu":
        decode = "<hhh"
        num_regs = 3
        message = "{0} -> x:{1[0]}, y:{1[1]}, z:{1[2]}"

    num_boards = len(board_ids)

    def callback() -> int:
        boards.clearWDGRST(client)

        try:
            responses = client.readRegisters(
                board_ids, [address] * num_boards, [num_regs] * num_boards
            )
            for i in range(len(responses)):
                if not responses[i]:
                    print("Board {} could not be read.".format(i))
                    continue

                val = struct.unpack(decode, responses[i])
                bid = board_ids[i]
                print("Board:", bid, message.format(args.sensor, val))

        except (comms.MalformedPacketError, comms.ProtocolError) as e:
            if "id: " in str(e):
                return int(str(e).split("id: ")[1][0])
            else:
                return -1

        return 0

    loop = utils.DebugLoop(
        callback=callback, num_iters=args.num_iters, iters_per_print=1000
    )

    loop.loop()


if __name__ == "__main__":
    action(parser_args())
