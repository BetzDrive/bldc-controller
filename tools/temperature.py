#!/usr/bin/env python
import numpy as np
from comms import *
import serial
import sys
import time

if len(sys.argv) != 3:
        print("give me a serial port and address")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

address = int(sys.argv[2])

client = BLDCControllerClient(s, True)

client.leaveBootloader(address)
s.reset_input_buffer()
time.sleep(0.1)

while True:
    try:
        temperature = struct.unpack('<f', client.readRegisters(address, 0x3005, 1))
        print(temperature[0])
        # print struct.unpack('<f', client.readRegisters(address, 0x8001, 1))[0]
    except IOError:
        print "ioerror"
        pass
    # angle = struct.unpack('<f', client.readRegisters(address, 0x8001, 1))[0]
    # print angle
    time.sleep(0.1)

# try:
#     temperature = struct.unpack('<f', client.readRegisters(address, 0x010c, 1))
#     print(temperature[0])
# except IOError:
#     print "ioerror"
