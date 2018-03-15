#!/usr/bin/env python
from comms import *
import serial
import sys
import time
from math import sin, cos, pi

PROTOCOL_V2 = True

if len(sys.argv) != 4:
        print("give me a serial port, address, and duty cycle")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

address = int(sys.argv[2])
duty_cycle = float(sys.argv[3])

client = BLDCControllerClient(s, protocol_v2=PROTOCOL_V2)

client.leaveBootloader(address)
time.sleep(0.2)
s.reset_input_buffer()

angle_mapping = {1: 654, 2: 243, 3: 2827, 4: 1125, 5: 7568, 10: 547, 11: 16015, 12: 8710, 13: 10054, 14: 1008, 15: 775, 16: 22, 17: 204, 18: 247, 19: 601, 20: 721, 21: 621, 22: 6985, 23: 6262, 24: 518} # mapping of id to joints

needs_flip_phase = [3, 4, 11, 17, 18, 22, 23, 24]

has_21_erevs_per_mrev = [2, 13, 18, 19, 20, 21]

if PROTOCOL_V2:
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
    client.writeRegisters(address, 0x1003, 1, struct.pack('<f', 1.0)) # FOC direct current Kp
    client.writeRegisters(address, 0x1005, 1, struct.pack('<f', 1.0)) # FOC quadrature current Kp
    client.writeRegisters(address, 0x1040, 1, struct.pack('<f', 1e-2)) # Velocity filter parameter
    client.writeRegisters(address, 0x1030, 1, struct.pack('<H', 1000)) # Control watchdog timeout
    # client.writeRegisters(address, 0x1030, 1, struct.pack('<H', 0))

    client.writeRegisters(address, 0x2006, 1, struct.pack('<f', duty_cycle))
    client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 2) ) # Torque control

    # client.writeRegisters(address, 0x1007, 1, struct.pack('<f', 1.0))
    # client.writeRegisters(address, 0x1008, 1, struct.pack('<f', 0.01))
    # client.writeRegisters(address, 0x1011, 1, struct.pack('<f', 0.5))
    # client.writeRegisters(address, 0x2007, 1, struct.pack('<f', duty_cycle))
    # client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 3) ) # Velocity control

    # client.writeRegisters(address, 0x1009, 1, struct.pack('<f', 20.0))
    # client.writeRegisters(address, 0x100a, 1, struct.pack('<f', 0.05))
    # client.writeRegisters(address, 0x1012, 1, struct.pack('<f', 20.0))
    # client.writeRegisters(address, 0x2008, 1, struct.pack('<f', duty_cycle))
    # client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 4) ) # Position control

    # print struct.unpack('<f', client.readRegisters(address, 0x1003, 1))
else:
    client.writeRegisters(address, 0x1010, 1, struct.pack('<H', angle_mapping[address]) )
    client.writeRegisters(address, 0x1011, 1, struct.pack('<B', 0) )
    client.writeRegisters(address, 0x1012, 1, struct.pack('<B', int(address in needs_flip_phase)) )
    try:
        client.writeRegisters(address, 0x010a, 1, struct.pack('<B', 21 if (address in has_21_erevs_per_mrev) else 14))
    except:
        print "WARNING: Motor driver board does not support erevs_per_mrev, try updating the firmware."

    # start_angle = struct.unpack('<f', client.readRegisters(address, 0x010b, 1))[0]
    # client.writeRegisters(address, 0x0110, 1, struct.pack('<f', start_angle - 0.5))
    # client.writeRegisters(address, 0x0111, 1, struct.pack('<f', start_angle))
    # client.writeRegisters(address, 0x0112, 1, struct.pack('<f', 20.0))

    client.writeRegisters(address, 0x0106, 1, struct.pack('<f', duty_cycle))

while True:
    data = struct.unpack('<ff', client.readRegisters(address, 0x3002, 2))
    print(data)
    time.sleep(0.01)

# t = 0
# ts = 0.01
# amplitude = 0.5
# frequency = 0.5
# offset, = struct.unpack('<f', client.readRegisters(address, 0x3000, 1))
# while True:
#     position_sp = offset + amplitude * (1 - cos(2 * pi * frequency * t))
#     velocity_sp = amplitude * sin(2 * pi * frequency * t) * 2 * pi * frequency
#     # client.writeRegisters(address, 0x1012, 1, struct.pack('<f', abs(velocity_sp)))
#     # client.writeRegisters(address, 0x2008, 1, struct.pack('<f', position_sp))
#     # position, = struct.unpack('<f', client.readRegisters(address, 0x3000, 1))
#     client.writeRegisters(address, 0x2007, 1, struct.pack('<f', velocity_sp))
#     time.sleep(ts)
#     t += ts

# while True:
#     client.writeRegisters(address, 0x2008, 1, struct.pack('<f', duty_cycle))
#     time.sleep(2)
#     client.writeRegisters(address, 0x2008, 1, struct.pack('<f', -duty_cycle))
#     time.sleep(2)

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
