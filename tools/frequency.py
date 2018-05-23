#!/usr/bin/env python
from comms import *
import serial
import sys
import time

if len(sys.argv) < 3:
        print("give me a serial port and address(es)")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=1000000, timeout=0.1)

boards = sys.argv[2:]
address = [0]*len(boards)
for i in range(len(boards)):
    address[i] = int(boards[i])

client = BLDCControllerClient(s, 3)

client.leaveMultiBootloader(address)
s.reset_input_buffer()

last_time = time.time()

while True:
    for _ in range(1000):
        try:
            client.readMultiRegisters(address, 0x3005, 1)
        except IOError:
            print "ioerror"
            pass
    
    print (time.time()-last_time)
    last_time = time.time()
