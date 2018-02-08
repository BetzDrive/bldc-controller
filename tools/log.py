#!/usr/bin/env python
from comms import *
import serial
import sys
import time
import pickle
import pprint

if len(sys.argv) < 3:
        print("give me a serial port and address")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

address = int(sys.argv[2])

client = BLDCControllerClient(s, True)

client.leaveBootloader(address)
s.reset_input_buffer()
time.sleep(1)

reset = struct.unpack('<B', client.readRegisters(address, 0x300b, 1))[0]
print("reset: %u" % reset)
success = struct.unpack('<B', client.readRegisters(address, 0x3009, 1))[0]
print("success: %u" % success)
if success:
    time.sleep(0.1)
    l = struct.unpack('<H', client.readRegisters(address, 0x300a, 1))[0]
    while l == 0:
        l = struct.unpack('<H', client.readRegisters(address, 0x300a, 1))[0]
        time.sleep(0.1)
    arr = []
    for i in range(0, l, 16):
        a = (struct.unpack("<16f", client.readRegisters(address, 0x8000 + i, 16)))
        arr += a


if len(sys.argv) == 4:
    with open(sys.argv[3], 'wb') as file:
        pickle.dump(arr, file)
    print("dumped data to file " + sys.argv[3])
else:
    pp = pprint.PrettyPrinter()
    pp.pprint(arr)
