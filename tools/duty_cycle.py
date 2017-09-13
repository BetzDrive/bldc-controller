#!/usr/bin/env python
import numpy as np
from comms import *
import serial
import sys
import time

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=3000000, timeout=0.2)

print s.BAUDRATES

client = BLDCControllerClient(s)

client.leaveBootloader(0x01)
print("hello")
s.flush()
time.sleep(0.1)
print("hello")
client.writeRegisters(0x01, 0x0101, 1, struct.pack('<H', 9346) )
print("hello")
client.writeRegisters(0x01, 0x0106, 1, struct.pack('<f', -1) )
print("hello")
client.writeRegisters(0x01, 0x0102, 1, struct.pack('<B', 0) )
print("hello")

while True:
	angle = struct.unpack('<f', client.readRegisters(0x01, 0x0107, 1))[0]
	print(angle)
	time.sleep(0.1)
