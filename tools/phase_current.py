#!/usr/bin/env python
from comms import *
import serial
import sys
import time
import numpy as np

if len(sys.argv) != 4:
        print("give me a serial port, address, and file name")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

address = int(sys.argv[2])

filename = sys.argv[3]

client = BLDCControllerClient(s)

client.leaveBootloader([address])
time.sleep(0.2)
s.reset_input_buffer()

sample = 0
phase_currents = []
num_samples = 8000

while sample < num_samples:
    try:
        phase_currents.append(struct.unpack('<f', client.readRegisters([address], [0x3020], [1])[0])[0])
        print(phase_currents[sample])
        sample += 1
        # print struct.unpack('<f', client.readRegisters(address, 0x8001, 1))[0]
    except:
        print "ioerror"
        pass
    # angle = struct.unpack('<f', client.readRegisters(address, 0x8001, 1))[0]
    # print angle
    #time.sleep(0.1)

num_read = struct.unpack('<L', client.readRegisters([address], [0x3021], [1])[0])[0]
print("Number of samples sent from board: " + str(num_read))

print("Number of current samples: " + str(num_samples))
print("Average current: " + str(np.average(phase_currents)))
print("Standard deviation: " + str(np.std(phase_currents)))

with open(filename, "w") as myfile:
    for i in range(len(phase_currents)):
        myfile.write(str(phase_currents[i]) + "\n")
    myfile.close()


# try:
#     temperature = struct.unpack('<f', client.readRegisters(address, 0x010c, 1))
#     print(temperature[0])
# except IOError:
#     print "ioerror"
