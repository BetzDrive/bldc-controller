#!/usr/bin/env python3

import argparse
import ast
import struct
import time

import serial

from bd_tools import boards, comms, livegraph


def parser_args():
    parser = argparse.ArgumentParser(description="Display system time.")
    boards.addBoardArgs(parser)
    parser.set_defaults(
        baud_rate=comms.COMM_DEFAULT_BAUD_RATE,
        offset=comms.COMM_BOOTLOADER_OFFSET,
    )
    return parser.parse_args()


def action(args):
    make_list = (
        lambda x: list(x) if (type(x) == list or type(x) == tuple) else [x]
    )

    def make_int(x):
        return [int(y) for y in x]

    board_ids = make_int(make_list(ast.literal_eval(args.board_ids)))

    ser = serial.Serial(
        port=args.serial, baudrate=args.baud_rate, timeout=0.05
    )

    client = comms.BLDCControllerClient(ser)
    boards.initBoards(client, board_ids)

    client.leaveBootloader(board_ids)
    client.resetInputBuffer()

    boards.initMotor(client, board_ids)

    def updateCurrent(i):
        data = []
        for board_id in board_ids:
            try:
                # Read the local time
                data.append(
                    struct.unpack(
                        "<f",
                        client.readRegisters([board_id], [0x0006], [1])[0],
                    )
                )
                print(data[-1])
            except (comms.MalformedPacketError, comms.ProtocolError):
                print("Failed to communicate with board: ", board_id)
        return time.time(), None if len(data) != (1 * len(board_ids)) else data

    def flatten(item_list):
        return [item for sublist in item_list for item in sublist]

    labels = []
    labels.extend(
        [
            [
                str(bid) + "'s Time Reading",
            ]
            for bid in board_ids
        ]
    )
    labels = flatten(labels)
    graph = livegraph.LiveGraph(
        updateCurrent, labels, sample_interval=1, window_size=2000
    )

    graph.start()


if __name__ == "__main__":
    action(parser_args())
