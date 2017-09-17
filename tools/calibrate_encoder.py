#!/usr/bin/env python
import numpy as np
from comms import *
import serial
import sys
import time

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=3000000, timeout=0.1)

print s.BAUDRATES

client = BLDCControllerClient(s)

client.leaveBootloader(0x01)
s.flush()
time.sleep(0.1)
count = 0
duty_cycle = 0.6
phase_state_list = [(1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (1, 0, 1)]

angles = []

while True:
    a, b, c = phase_state_list[count % 6]

    client.writeRegisters(0x01, 0x0102, 4, struct.pack('<Bfff', 1, a * duty_cycle, b * duty_cycle, c * duty_cycle))

    time.sleep(2 if count == 0 else 0.1)

    angle = struct.unpack('<H', client.readRegisters(0x01, 0x100, 1))[0]
    angles.append(angle)

    if count > 4 and abs(angles[0] - angle) < abs(angles[1] - angles[0]) / 3.0:
        break

    count += 1

erpm_per_revolution = count / 6
phase_aligned_angle = angles[0]
print("ERPM per rev:\t" + str(erpm_per_revolution))
print("Phase aligned angle:\t" + str(phase_aligned_angle))
print(angles)
print(count)
# client.writeRegisters(0x01, 0x0102, 1, struct.pack('<B', 1))
# try:
#     client.writeRegisters(0x01, 0x100, 1, struct.pack('<H', 100))
# except Exception as e:
#     print(e.errors)
# print(struct.unpack('<H', client.readRegisters(0x01, 0x101, 1)))

