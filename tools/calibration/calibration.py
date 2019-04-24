#!/usr/bin/env python
import sys
sys.path.append("..")

from comms import *
import serial
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

print(client.readCalibration(address))

