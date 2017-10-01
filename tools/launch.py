#!/usr/bin/env python

from comms import *
import serial
import sys

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE)

print s.BAUDRATES

client = BLDCControllerClient(s)

client.leaveBootloader(0x01)
s.flush()
