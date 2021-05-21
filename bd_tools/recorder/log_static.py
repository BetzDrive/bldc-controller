#!/usr/bin/env python
import sys

sys.path.append("..")

import argparse
import pickle
import pprint
import time

import numpy as np
import serial
from boards import *
from comms import *

# 14-bit encoder
phase_state_list = [(1, 0, 0)]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calibrate the encoder on a motor controller board."
    )
    parser.add_argument("serial", type=str, help="Serial port")
    parser.add_argument("--baud_rate", type=int, help="Serial baud rate")
    parser.add_argument("board_id", type=int, help="Board ID")
    parser.add_argument("file_name", type=str, help="File name to record data")
    parser.add_argument("--steps", type=int, help="Number of steps")
    parser.add_argument("--delay", type=float, help="Delay between steps")
    parser.set_defaults(
        baud_rate=COMM_DEFAULT_BAUD_RATE,
        steps=len(phase_state_list),
        delay=0.015,
        file_name="",
    )
    args = parser.parse_args()

    #
    # Data collection
    #

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)

    client = BLDCControllerClient(ser)

    initialized = initBoards(client, [args.board_id])

    client.leaveBootloader([args.board_id])
    time.sleep(0.2)  # Wait for the controller to reset
    client.resetInputBuffer()

    def set_phase_state(phase_state):
        a, b, c = phase_state
        client.writeRegisters(
            [args.board_id], [0x2003], [3], [struct.pack("<fff", a, b, c)]
        )

    client.writeRegisters(
        [args.board_id], [0x1030], [1], [struct.pack("<H", 1000)]
    )  # Control watchdog timeout
    client.writeRegisters(
        [args.board_id], [0x2003], [3], [struct.pack("<fff", 0, 0, 0)]
    )
    client.writeRegisters(
        [args.board_id], [0x2000], [1], [struct.pack("<B", 1)]
    )

    time.sleep(args.delay)

    # Reset Recorder
    reset = struct.unpack(
        "<B", client.readRegisters([args.board_id], [0x300B], [1])[0]
    )[0]
    print("reset: %u" % reset)
    success = struct.unpack(
        "<B", client.readRegisters([args.board_id], [0x3009], [1])[0]
    )[0]
    print("success: %u" % success)

    for i in range(args.steps):
        set_phase_state(phase_state_list[i % len(phase_state_list)])
        time.sleep(args.delay)

    # The number of values returned by the recorder (all floats)
    num_recorder_elements = 8

    if success:
        time.sleep(0.2)
        l = struct.unpack(
            "<H", client.readRegisters([args.board_id], [0x300A], [1])[0]
        )[0]
        while l == 0:
            l = struct.unpack(
                "<H", client.readRegisters([args.board_id], [0x300A], [1])[0]
            )[0]
            time.sleep(0.1)
        arr = []
        for i in range(0, l, num_recorder_elements):
            # Grab the recorder data
            while True:
                try:
                    a = struct.unpack(
                        "<" + str(num_recorder_elements) + "f",
                        client.readRegisters(
                            [args.board_id],
                            [0x8000 + i],
                            [num_recorder_elements],
                        )[0],
                    )
                    arr += [a]
                    break
                except:
                    continue
    if args.file_name:
        with open(args.file_name, "wb") as file:
            pickle.dump(arr, file)
        print("dumped data to file " + args.file_name)
    else:
        pp = pprint.PrettyPrinter()
        pp.pprint(arr[0])
