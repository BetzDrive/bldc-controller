#!/usr/bin/env python

from comms import *
import serial
import sys

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=3000000)

print s.BAUDRATES

client = BLDCControllerClient(s)

client.leaveBootloader(0x01)
s.flush()
