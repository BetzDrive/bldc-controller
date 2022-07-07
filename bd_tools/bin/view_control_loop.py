#!/usr/bin/env python3

import argparse
import ast
import struct
import time

import serial

from bd_tools import boards, comms, livegraph


def parser_args():
    parser = argparse.ArgumentParser(
        description="Drive motor module(s) with a given control mode and plot "
        "current measurements."
    )
    parser.add_argument("serial", type=str, help="Serial port")
    parser.add_argument("--baud_rate", type=int, help="Serial baud rate")
    parser.add_argument(
        "board_ids", type=str, help="Board ID (separate with comma)"
    )
    parser.add_argument(
        "mode",
        type=str,
        help="Control mode: current (Id[A], Iq[A]), "
        "phase (dc,dc,dc), "
        "torque (N*m), "
        "velocity (rad/s), "
        "position (rad), "
        "pos_vel (rad,rad/s), "
        "pos_ff (rad,ff[A]), "
        "pwm (dc)",
    )
    parser.add_argument(
        "actuations",
        type=str,
        help="Actuation amount in the units of the selected mode (if requires "
        "multiple args, separate by comma)",
    )
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
    actuations = make_list(ast.literal_eval(args.actuations))

    mode = args.mode

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
                boards.driveMotor(client, board_ids, actuations, mode)
                # Read the iq calulated
                data.append(
                    struct.unpack(
                        "<f",
                        client.readRegisters([board_id], [0x3003], [1])[0],
                    )
                )
                # Read the iq command
                data.append(
                    struct.unpack(
                        "<f",
                        client.readRegisters([board_id], [0x3020], [1])[0],
                    )
                )
                # Read the low frequency velocity estimate
                data.append(
                    struct.unpack(
                        "<f",
                        client.readRegisters([board_id], [0x3001], [1])[0],
                    )
                )
            except (comms.MalformedPacketError, comms.ProtocolError):
                print("Failed to communicate with board: ", board_id)
        return time.time(), None if len(data) != (3 * len(board_ids)) else data

    def flatten(item_list):
        return [item for sublist in item_list for item in sublist]

    labels = []
    labels.extend(
        [
            [
                str(bid) + "'s iq Reading",
                str(bid) + "'s iq PID output",
                str(bid) + "'s lf velocity",
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
