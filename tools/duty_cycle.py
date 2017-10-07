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

angle_mapping = {1: 10356, 2: 13430, 3: 12164, 4: 8132} # mapping of id to joints
flip_phase_mapping = {1: False , 2: True , 3: False , 4: False}

client.writeRegisters(address, 0x0101, 1, struct.pack('<H', angle_mapping[address]) )
client.writeRegisters(address, 0x0106, 1, struct.pack('<f', duty_cycle) )
client.writeRegisters(address, 0x0102, 1, struct.pack('<B', 0) )
client.writeRegisters(address, 0x0109, 1, struct.pack('<B', 1) )

while True:
#    try:
#        adc_averages = struct.unpack('<7f', client.readRegisters(address, 0x0200, 7))
#        print "ia:{: > 7.3f} ib:{: > 7.3f} ic:{: > 7.3f} va:{: > 7.3f} vb:{: > 7.3f} vc:{: > 7.3f} vin:{: > 7.3f}".format(*adc_averages)
#    except IOError:
#        pass
    angle = struct.unpack('<H', client.readRegisters(address, 0x100, 1))[0]
    print angle
    time.sleep(0.5)
