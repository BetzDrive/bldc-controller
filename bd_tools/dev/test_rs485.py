#!/usr/bin/env python
from __future__ import print_function

import sys
import serial
import time
import argparse
import random
import numpy as np

DEFAULT_BAUD_RATE = 3000000  # 1 Mbps

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Drive motor module(s) with a given control mode."
    )
    parser.add_argument(
        "transmit", type=str, help="Transmitter serial port (i.e. /dev/ttyUSB0)"
    )
    parser.add_argument("receive", type=str, help="Receiver serial port")
    parser.add_argument("--baud_rate", type=int, help="Serial baud rate")
    parser.set_defaults(baud_rate=DEFAULT_BAUD_RATE)
    args = parser.parse_args()

    trn = serial.Serial(port=args.transmit, baudrate=args.baud_rate, timeout=2.0)
    rec = serial.Serial(port=args.receive, baudrate=args.baud_rate, timeout=2.0)

    trn.reset_input_buffer()
    rec.reset_input_buffer()

    # Generate 1k random bits
    s = random.randint(0, 2 ** 1000 - 1)
    msg = hex(s)
    print(msg)

    trn.write(msg)

    time.sleep(0.01)

    return_msg = rec.read_all()
    print(return_msg)

    print("Equal:", return_msg == msg)
