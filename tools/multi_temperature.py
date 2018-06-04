#!/usr/bin/env python
from comms import *
import serial
import sys
import time

if len(sys.argv) < 3:
        print("give me a serial port and address(es)")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

boards = sys.argv[2:]
address = [0]*len(boards)
for i in range(len(boards)):
    address[i] = int(boards[i])

client = BLDCControllerClient(s)

client.leaveBootloader(address)
time.sleep(0.2)
s.reset_input_buffer()

while True:
    try:
        temperature = client.readRegisters(address, [0x3005 for b in boards], [1 for b in boards])
        for i in range(len(temperature)):
            print(str(boards[i]) + ": " + str(struct.unpack('<f', temperature[i])[0]))
        # print struct.unpack('<f', client.readRegisters(address, 0x8001, 1))[0]
    except IOError:
        print "ioerror"
    except ProtocolError:
        print "Protocol Error"
        #pass
    # angle = struct.unpack('<f', client.readRegisters(address, 0x8001, 1))[0]
    # print angle
    #time.sleep(0.1)

# try:
#     temperature = struct.unpack('<f', client.readRegisters(address, 0x010c, 1))
#     print(temperature[0])
# except IOError:
#     print "ioerror"
