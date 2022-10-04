#!/usr/bin/env python3
from __future__ import print_function

import argparse
import ast
import dataclasses

import serial

from bd_tools import boards, comms, utils


@dataclasses.dataclass
class Control:
    """Drive motor module(s) with a given control mode.

    Args:
        num_iters: Number of iterations to loop (default to infinity).
    """

    serial: boards.Serial
    motor: boards.Motor
    num_iters: int = 0

    def main(self):
        action(self)


def parser_args():
    parser = argparse.ArgumentParser(
        description="Drive motor module(s) with a given control mode."
    )
    parser.add_argument(
        "--num_iters",
        type=int,
        help="Number of iterations to loop (default to infinity)",
    )
    boards.addBoardArgs(parser)
    boards.addMotorControlArgs(parser)
    parser.set_defaults(
        baud_rate=comms.COMM_DEFAULT_BAUD_RATE,
        num_iters=0,
        offset=comms.COMM_BOOTLOADER_OFFSET,
    )
    return parser.parse_args()


def action(args):
    def make_list(x):
        return list(x) if (type(x) == list or type(x) == tuple) else [x]

    def make_type(x, to_type):
        return [to_type(y) for y in x]

    board_ids = make_type(
        make_list(ast.literal_eval(args.motor.board_ids)), int
    )
    actuations = make_list(ast.literal_eval(args.actuations))

    mode = args.mode

    ser = serial.Serial(
        port=args.serial, baudrate=args.baud_rate, timeout=0.001
    )

    client = comms.BLDCControllerClient(ser)
    boards.initBoards(client, board_ids)

    client.leaveBootloader(board_ids)

    client.resetInputBuffer()

    boards.initMotor(client, board_ids)

    def callback() -> bool:
        boards.clearWDGRST(client)

        try:
            boards.driveMotor(client, board_ids, actuations, mode)
        except (comms.ProtocolError, comms.MalformedPacketError):
            return False

        return True

    loop = utils.DebugLoop(callback, args.num_iters, iters_per_print=1000)

    loop.loop()


if __name__ == "__main__":
    action(parser_args())
