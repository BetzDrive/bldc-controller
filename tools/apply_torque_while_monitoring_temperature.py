#!/usr/bin/env python
from comms import *
import serial
import sys
import time
from math import sin, cos, pi

## Applies a torque and continuously prints out temperature information
## Output is meant to be piped to a CSV file

PROTOCOL_V2 = 2

if len(sys.argv) != 4:
        print("give me a serial port, address, and torque")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

address = int(sys.argv[2])
torque = float(sys.argv[3])

client = BLDCControllerClient(s, protocol=PROTOCOL_V2)

client.leaveBootloader(address)
time.sleep(0.2)
s.reset_input_buffer()

angle_mapping = {1: 654, 2: 243, 3: 2827, 4: 1125, 5: 7568, 10: 547, 11: 16015, 12: 8710, 13: 10054, 14: 1008, 15: 775, 16: 22, 17: 204, 18: 247, 19: 601, 20: 721, 21: 621, 22: 6985, 23: 6262, 24: 518} # mapping of id to joints

needs_flip_phase = [3, 4, 11, 17, 18, 22, 23, 24]

has_21_erevs_per_mrev = [2, 13, 18, 19, 20, 21]

if PROTOCOL_V2:
    calibration = client.readCalibration(address)
    client.writeRegisters(address, 0x1000, 1, struct.pack('<H', calibration['angle']) )
    client.writeRegisters(address, 0x1002, 1, struct.pack('<B', calibration['inv']) )
    try:
        client.writeRegisters(address, 0x1001, 1, struct.pack('<B', calibration['epm']))
    except:
        print "WARNING: Motor driver board does not support erevs_per_mrev, try updating the firmware."

    # start_angle = struct.unpack('<f', client.readRegisters(address, 0x010b, 1))[0]
    # client.writeRegisters(address, 0x0110, 1, struct.pack('<f', start_angle - 0.5))
    # client.writeRegisters(address, 0x0111, 1, struct.pack('<f', start_angle))
    # client.writeRegisters(address, 0x0112, 1, struct.pack('<f', 20.0))

    client.writeRegisters(address, 0x1022, 1, struct.pack('<f', calibration['torque'])) # Motor torque constant
    client.writeRegisters(address, 0x1003, 1, struct.pack('<f', 1.0)) # FOC direct current Kp
    client.writeRegisters(address, 0x1005, 1, struct.pack('<f', 1.0)) # FOC quadrature current Kp
    client.writeRegisters(address, 0x1040, 1, struct.pack('<f', 1e-2)) # Velocity filter parameter
    client.writeRegisters(address, 0x1030, 1, struct.pack('<H', 1000)) # Control watchdog timeout
    # client.writeRegisters(address, 0x1030, 1, struct.pack('<H', 0))

    client.writeRegisters(address, 0x2006, 1, struct.pack('<f', torque))
    client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 2) ) # Torque control
else:
    client.writeRegisters(address, 0x1010, 1, struct.pack('<H', angle_mapping[address]) )
    client.writeRegisters(address, 0x1011, 1, struct.pack('<B', 0) )
    client.writeRegisters(address, 0x1012, 1, struct.pack('<B', int(address in needs_flip_phase)) )
    try:
        client.writeRegisters(address, 0x010a, 1, struct.pack('<B', 21 if (address in has_21_erevs_per_mrev) else 14))
    except:
        print "WARNING: Motor driver board does not support erevs_per_mrev, try updating the firmware."

    client.writeRegisters(address, 0x0106, 1, struct.pack('<f', torque))

print("Time,Temperature")
next_print = time.time()
while True:
    data = struct.unpack('<f', client.readRegisters(address, 0x3005, 1))
    print(str("{},{}".format(time.time(), data))

    next_print += 1.0
    sleep_time = next_print - time.time()
    if sleep_time > 0:
        time.sleep(sleep_time)

