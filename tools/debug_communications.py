#!/usr/bin/env python2
from __future__ import print_function

import argparse
import time

import serial
from boards import *
from comms import *

ReadOnlyRegs = {}
ReadOnlyRegs["encoder"] = COMM_ROR_ROTOR_POS
ReadOnlyRegs["encoder_raw"] = COMM_ROR_ROTOR_POS_RAW
ReadOnlyRegs["velocity"] = COMM_ROR_ROTOR_VEL
ReadOnlyRegs["id"] = COMM_ROR_CURRENT_DIRECT
ReadOnlyRegs["iq"] = COMM_ROR_CURRENT_QUADRATURE
ReadOnlyRegs["supply"] = COMM_ROR_SUPPLY_V
ReadOnlyRegs["temp"] = COMM_ROR_TEMPERATURE
ReadOnlyRegs["imu"] = COMM_ROR_ACC_X


def count_errors(board_ids):
    num_boards = len(board_ids)

    total_messages = 1000
    protocal_error = 0
    struct_error = 0

    for _ in range(total_messages):
        total_messages += 1
        crashed = client.checkWDGRST()
        if crashed:
            print("boards:", crashed, "have crashed")
        try:
            address = ReadOnlyRegs[args.sensor]
            responses = client.readRegisters(
                board_ids, [address] * num_boards, [num_regs] * num_boards
            )
            for i in range(len(responses)):
                val = struct.unpack(decode, responses[i])
                bid = board_ids[i]
                if not args.debug_only:
                    print("Board:", bid, message.format(args.sensor, val))
        except ProtocolError as err:
            protocal_error += 1
        except struct.error as err:
            struct_error += 1

        time.sleep(0.05)  # at most 20 hz

    print("board ids: ", board_ids)
    print("total_messages: ", total_messages)
    print("protocal_errors: ", protocal_error)
    print("struct_errors: ", struct_error)
    print(
        "total_error_percent: ",
        float(protocal_error + struct_error) / total_messages * 100.0,
    )
    print()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read a sensor from boards.")
    parser.add_argument("serial", type=str, help="Serial port")
    parser.add_argument("--baud_rate", type=int, help="Serial baud rate")
    parser.add_argument("num_boards", type=int, help="total number of boards")
    parser.add_argument(
        "sensor",
        type=str,
        help="Choose sensor (encoder, encoder_raw, velocity, id, iq, supply, temp, imu)",
    )
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.004)
    board_ids = [i + 1 for i in range(args.num_boards)]
    client = BLDCControllerClient(ser)
    initialized = initBoards(client, board_ids)
    if not initialized:
        raise RuntimeError("Boards not initalized")

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

    count_errors(board_ids)
    for i in range(8):
        count_errors(i + 1)
