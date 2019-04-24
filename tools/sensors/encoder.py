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

client = BLDCControllerClient(s)

client.enterBootloader([address])
time.sleep(0.5)
try:
    client.enumerateBoards([address])
except:
    print("Failed to respond")
time.sleep(0.5)
client.leaveBootloader([address])
time.sleep(1)
s.reset_input_buffer()

while True:
    try:
        encoder = struct.unpack('<H', client.readRegisters([address], [0x3010], [1])[0])
        print(encoder[0])
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
