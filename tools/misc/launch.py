#!/usr/bin/env python
import sys
sys.path.append("..")

from comms import *
import serial

port = sys.argv[1]
address = int(sys.argv[2])
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE)

print s.BAUDRATES

client = BLDCControllerClient(s)

client.leaveBootloader([address])
s.flush()
