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

address = [int(addr) for addr in sys.argv[2].split(',')]

client = BLDCControllerClient(s)

for addr in address:
    client.enterBootloader([addr])
    time.sleep(0.2)
    print(struct.unpack('<B', client.enumerateBoards([addr])[0])[0])
    time.sleep(0.2)

# try:
#     temperature = struct.unpack('<f', client.readRegisters(address, 0x010c, 1))
#     print(temperature[0])
# except IOError:
#     print "ioerror"
