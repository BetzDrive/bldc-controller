#!/usr/bin/env python
import numpy as np
from comms import *
import serial
import sys
import time

if len(sys.argv) < 3:
        print("give me a serial port and duty cycle")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=3000000, timeout=0.2)

address = int(sys.argv[2])
duty_cycle = float(sys.argv[3])

client = BLDCControllerClient(s)

client.leaveBootloader(address)
s.flush()
time.sleep(0.1)

client.writeRegisters(address, 0x0101, 1, struct.pack('<H', 9346) )
client.writeRegisters(address, 0x0106, 1, struct.pack('<f', duty_cycle) )
client.writeRegisters(address, 0x0102, 1, struct.pack('<B', 0) )

# while True:
#         angle = struct.unpack('<f', client.readRegisters(address, 0x0107, 1))[0]
#         print(angle)
#         time.sleep(0.1)
