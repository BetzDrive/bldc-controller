#!/usr/bin/env python
from comms import *
import serial
import sys
import time

if len(sys.argv) < 3:
        print("give me a serial port and address")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

addresses = list(int(addr) for addr in sys.argv[2:])

client = BLDCControllerClient(s, True)

for addr in addresses:
    client.leaveBootloader(addr)
    time.sleep(0.2)
    s.reset_input_buffer()

while True:
    for addr in addresses:
        try:
            state = struct.unpack('<f', client.readRegisters(addr, 0x3000, 1))
            #print state 
        except IOError:
            print "ioerror, address {}".format(addr)
            pass
    #time.sleep(0.001)
    time.sleep(0.01)
