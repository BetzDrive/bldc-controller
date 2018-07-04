#!/usr/bin/env python
from comms import *
import serial
import sys
import time
import pickle
import pprint

if len(sys.argv) < 3:
        print("give me a serial port and address")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

address = int(sys.argv[2])

client = BLDCControllerClient(s, protocol=2)

angle_mapping = {1: 726, 2: 243, 3: 2827, 4: 1125, 5: 7568, 10: 800, 11: 823, 12: 501, 13: 10054, 14: 1008, 15: 775, 16: 22, 17: 1087, 18: 245, 19: 601, 20: 721, 21: 621, 22: 269, 23: 678, 24: 518} # mapping of id to joints

needs_flip_phase = [3, 4, 11, 17, 18, 22, 23, 24]

has_21_erevs_per_mrev = [2, 13, 18, 19, 20, 21]

client.leaveBootloader(address)
time.sleep(0.2)
s.reset_input_buffer()

client.writeRegisters(address, 0x1000, 1, struct.pack('<H', angle_mapping[address]) )
client.writeRegisters(address, 0x1002, 1, struct.pack('<B', int(address in needs_flip_phase)) )
try:
    client.writeRegisters(address, 0x1001, 1, struct.pack('<B', 21 if (address in has_21_erevs_per_mrev) else 14))
except:
    print "WARNING: Motor driver board does not support erevs_per_mrev, try updating the firmware."

# start_angle = struct.unpack('<f', client.readRegisters(address, 0x010b, 1))[0]
# client.writeRegisters(address, 0x0110, 1, struct.pack('<f', start_angle - 0.5))
# client.writeRegisters(address, 0x0111, 1, struct.pack('<f', start_angle))
# client.writeRegisters(address, 0x0112, 1, struct.pack('<f', 20.0))

client.writeRegisters(address, 0x1022, 1, struct.pack('<f', 0.55 if (address in has_21_erevs_per_mrev) else 1.45)) # Motor torque constant
# client.writeRegisters(address, 0x1022, 1, struct.pack('<f', 0))
client.writeRegisters(address, 0x1003, 1, struct.pack('<f', 0.1)) # FOC direct current Kp
client.writeRegisters(address, 0x1004, 1, struct.pack('<f', 0.0)) # FOC direct current Ki
client.writeRegisters(address, 0x1005, 1, struct.pack('<f', 0.1)) # FOC quadrature current Kp
client.writeRegisters(address, 0x1006, 1, struct.pack('<f', 0.0)) # FOC quadrature current Ki
client.writeRegisters(address, 0x1040, 1, struct.pack('<f', 1e-3)) # Velocity filter parameter
client.writeRegisters(address, 0x1030, 1, struct.pack('<H', 1000)) # Control watchdog timeout
# client.writeRegisters(address, 0x1030, 1, struct.pack('<H', 0))

client.writeRegisters(address, 0x2006, 1, struct.pack('<f', 0))
client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 2) ) # Torque control

# client.writeRegisters(address, 0x2003, 3, struct.pack('<fff', 0, 0, 0))
# client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 1) ) # Raw duty cycle control

reset = struct.unpack('<B', client.readRegisters(address, 0x300b, 1))[0]
print("reset: %u" % reset)
success = struct.unpack('<B', client.readRegisters(address, 0x3009, 1))[0]
print("success: %u" % success)
if success:
    time.sleep(0.2)
    l = struct.unpack('<H', client.readRegisters(address, 0x300a, 1))[0]
    while l == 0:
        l = struct.unpack('<H', client.readRegisters(address, 0x300a, 1))[0]
        time.sleep(0.1)
    arr = []
    for i in range(0, l, 16):
        a = (struct.unpack("<16f", client.readRegisters(address, 0x8000 + i, 16)))
        arr += a


if len(sys.argv) == 4:
    with open(sys.argv[3], 'wb') as file:
        pickle.dump(arr, file)
    print("dumped data to file " + sys.argv[3])
else:
    pp = pprint.PrettyPrinter()
    pp.pprint(arr)
