#!/usr/bin/env python
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
time.sleep(0.2)
s.reset_input_buffer()

while True:
    try:
        x_x, x_y, x_z = struct.unpack('<iii', client.readRegisters(address, 0x3006, 3))
        print("x accel: ", x_x)
        print("y accel: ", x_y)
        print("z accel: ", x_z)
        #debug = struct.unpack('<H', client.readRegisters(address, 0x8000, 1))
        #print(debug[0])
    except IOError:
        print "ioerror"
        pass
    time.sleep(1)

