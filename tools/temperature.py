#!/usr/bin/env python3
from comms import *
import serial
import sys
import time

if len(sys.argv) != 3:
        print("give me a serial port and address")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

address = [int(bid) for bid in sys.argv[2].split(',')]

client = BLDCControllerClient(s)

client.leaveBootloader(address)
time.sleep(0.2)
while (True):
    try:
        print "Received Board ID: " + str(client.enumerateBoards(address))
        break
    except IOError as e:
        print("Comms Error:", e)
        time.sleep(0.2)

time.sleep(1)
client.leaveBootloader(address)
time.sleep(0.2)
s.reset_input_buffer()

while True:
    try:
        temperature = client.getTemperature(address)
        print(temperature)
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
