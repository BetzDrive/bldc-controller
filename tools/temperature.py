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

client = BLDCControllerClient(s)

client.enterBootloader([address])
time.sleep(0.2)
try:
    print (client.enumerateBoards([address]))
except:
    print("no response")
time.sleep(1)
client.leaveBootloader([address])
time.sleep(0.2)
s.reset_input_buffer()

while True:
    try:
        temperature = client.getTemperature([address])
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
