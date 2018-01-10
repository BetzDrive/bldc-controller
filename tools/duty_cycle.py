#!/usr/bin/env python
import numpy as np
from comms import *
import serial
import sys
import time

if len(sys.argv) != 4:
        print("give me a serial port, address, and duty cycle")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.01)

address = int(sys.argv[2])
duty_cycle = float(sys.argv[3])

client = BLDCControllerClient(s)

client.leaveBootloader(address)
s.flush()
time.sleep(0.1)

angle_mapping = {1: 10356, 2: 13430, 3: 2827, 4: 8132, 5: 7568, 10: 11067, 11: 2164, 12: 1200, 13: 11839, 14: 4484, 15: 13002, 16: 2373, 17: 10720, 18: 284, 19: 2668, 20: 3839, 21: 5899, 22: 6985, 23: 6262} # mapping of id to joints

needs_flip_phase = [2, 3, 13, 17, 22, 23]

has_21_erevs_per_mrev = [18, 19, 20, 21]

client.writeRegisters(address, 0x0101, 1, struct.pack('<H', angle_mapping[address]) )
client.writeRegisters(address, 0x0106, 1, struct.pack('<f', duty_cycle) )
client.writeRegisters(address, 0x0102, 1, struct.pack('<B', 0) )
client.writeRegisters(address, 0x0109, 1, struct.pack('<B', int(address in needs_flip_phase)) )
try:
    client.writeRegisters(address, 0x010a, 1, struct.pack('<B', 21 if (address in has_21_erevs_per_mrev) else 14))
except:
    print "WARNING: Motor driver board does not support erevs_per_mrev, try updating the firmware."

# while True:
#     try:
#         # adc_averages = struct.unpack('<7f', client.readRegisters(address, 0x0200, 7))
#         # print "ia:{: > 7.3f} ib:{: > 7.3f} ic:{: > 7.3f} va:{: > 7.3f} vb:{: > 7.3f} vc:{: > 7.3f} vin:{: > 7.3f}".format(*adc_averages)
#         data = struct.unpack('<2f', client.readRegisters(address, 0x010c, 2))
#         print "id:{: > 7.3f} iq:{: > 7.3f}".format(*data)
#         # print struct.unpack('<f', client.readRegisters(address, 0x8001, 1))[0]
#     except IOError as e:
#         print e
#     # angle = struct.unpack('<f', client.readRegisters(address, 0x8001, 1))[0]
#     # print angle
#     time.sleep(0.5)
