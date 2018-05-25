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
address = [int(b) for b in boards]

client = BLDCControllerClient(s)

client.leaveBootloader(address)
s.reset_input_buffer()

last_time = time.time()

while True:
    client.sum_time = 0
    errors = 0

    for _ in range(1000):
        try:
            client.readRegisters(address, [0x3005 for b in boards], [1 for b in boards])
        except IOError:
            errors += 1
            pass
    
    diff = time.time() - last_time
    print ("Frequency: " + str(1000.0 / diff) + 
            " Millis: " + str(diff/1000.0) +
            " Errors: " + str(errors))
    last_time = time.time()
