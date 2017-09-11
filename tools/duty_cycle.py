#!/usr/bin/env python
import numpy as np
from comms import *
import serial
import sys
import time

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=3000000)

print s.BAUDRATES

client = BLDCControllerClient(s)

client.leaveBootloader(0x01)
s.flush()
time.sleep(0.1)

client.writeRegisters(0x01, 0x0101, 1, struct.pack('<L', 9346) )
client.writeRegisters(0x01, 0x0106, 1, struct.pack('<f', 0.5) )
client.writeRegisters(0x01, 0x0102, 1, struct.pack('<B', 0) )

