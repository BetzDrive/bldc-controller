#!/usr/bin/env python3
import argparse
import ast
import struct
import time

import numpy as np
import serial

from bd_tools import boards, comms

GRAVITY = 9.81

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Drive motor module(s) with a given control mode."
    )
    parser.add_argument("serial", type=str, help="Serial port")
    parser.add_argument("--baud_rate", type=int, help="Serial baud rate")
    parser.add_argument(
        "board_ids", type=str, help="Board ID (separate with comma)"
    )
    parser.add_argument(
        "mode",
        type=str,
        help="Control mode: \
                                                current (Id[A], Iq[A]), \
                                                phase (dc,dc,dc), \
                                                torque (N*m), \
                                                velocity (rad/s), \
                                                position (rad), \
                                                pos_vel (rad,rad/s), \
                                                pos_ff (rad,ff[A]), \
                                                pwm (dc)",
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
    args = parser.parse_args()

    def make_list(x):
        return list(x) if (type(x) == list or type(x) == tuple) else [x]

    def make_type(x, to_type):
        return [to_type(y) for y in x]

    board_ids = make_type(make_list(ast.literal_eval(args.board_ids)), int)
    actuations = make_list(ast.literal_eval(args.actuations))

    mode = args.mode

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)

    client = comms.BLDCControllerClient(ser)
    initialized = boards.initBoards(client, board_ids)

    client.leaveBootloader(board_ids)

    client.resetInputBuffer()

    boards.initMotor(client, board_ids)

    start_time = time.time()
    count = 0
    rollover = 1000

    def getIMU(bids):
        return client.readRegisters(
            bids, [comms.COMM_ROR_ACC_X] * len(bids), [3] * len(bids)
        )

    while True:
        # If there's a watchdog reset, clear the reset and perform any
        # configuration again
        crashed = client.checkWDGRST()
        if crashed != []:
            try:
                client.clearWDGRST(crashed)
            except (comms.MalformedPacketError, comms.ProtocolError):
                pass

        try:
            boards.driveMotor(client, board_ids, actuations, mode)
            responses = getIMU(board_ids)
            for i in range(len(responses)):
                val = struct.unpack("<hhh", responses[i])
                np_val = np.array(val, dtype=float)
                np_val = np_val / (1 << 13) * (GRAVITY)
                bid = board_ids[i]
                message = "{0} -> x:{1[0]}, y:{1[1]}, z:{1[2]}"
                print(
                    "Board:",
                    bid,
                    message.format("imu", np_val),
                    np.linalg.norm(np_val),
                )
        except (comms.MalformedPacketError, comms.ProtocolError):
            time.sleep(0.1)
