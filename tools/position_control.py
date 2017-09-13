#!/usr/bin/env python
import numpy as np
from comms import *
import serial
import sys
import time

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=3000000, timeout=0.001)

print s.BAUDRATES

client = BLDCControllerClient(s)

client.leaveBootloader(0x01)
print("hello")
s.flush()
time.sleep(0.1)
print("hello")
client.writeRegisters(0x01, 0x0101, 1, struct.pack('<H', 9346) )
print("hello")
client.writeRegisters(0x01, 0x0106, 1, struct.pack('<f', 0) )
print("hello")
client.writeRegisters(0x01, 0x0102, 1, struct.pack('<B', 0) )
print("hello")

position_setpoint = 5000
next_step = time.time() + 1
while True:
    duty_cycle = 0.0
    angle = struct.unpack('<H', client.readRegisters(0x01, 0x100, 1))[0]
    duty_cycle = min(max((angle - position_setpoint) * 0.001, -1), 1)
    client.writeRegisters(0x01, 0x0106, 1, struct.pack('<f', duty_cycle) )

    if time.time() > next_step:
        print("hellO")
        position_setpoint += 1000
        position_setpoint %= 2 ** 14
        next_step += 1
