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

try:
    addresses = [int(sys.argv[2])]
    duty_cycles = [float(sys.argv[3])]
except ValueError:
    addresses = [int(address_str) for address_str in sys.argv[2].split(',')]
    duty_cycles = [float(duty_cycle_str) for duty_cycle_str in sys.argv[3].split(',')]

client = BLDCControllerClient(s, protocol_v2=PROTOCOL_V2)

for address, duty_cycle in zip(addresses, duty_cycles):
    client.leaveBootloader(address)
    time.sleep(0.2)
    s.reset_input_buffer()

    if PROTOCOL_V2:
        calibration_obj = client.readCalibration(address)

        client.setZeroAngle(address, calibration_obj['angle'])
        client.setInvertPhases(address, calibration_obj['inv'])
        client.setERevsPerMRev(address, calibration_obj['epm'])
        client.setTorqueConstant(address, calibration_obj['torque'])
        client.setPositionOffset(address, calibration_obj['zero'])
        if calibration_obj['eac_type'] == 'int8':
            print('EAC calibration available')
            try:
                client.writeRegisters(address, 0x1100, 1, struct.pack('<f', calibration_obj['eac_scale']))
                client.writeRegisters(address, 0x1101, 1, struct.pack('<f', calibration_obj['eac_offset']))
                eac_table_len = len(calibration_obj['eac_table'])
                slice_len = 64
                for i in range(0, eac_table_len, slice_len):
                    table_slice = calibration_obj['eac_table'][i:i+slice_len]
                    client.writeRegisters(address, 0x1200+i, len(table_slice), struct.pack('<{}b'.format(len(table_slice)), *table_slice))
            except ProtocolError:
                print('WARNING: Motor driver board does not support electrical angle compensation, try updating the firmware.')
        client.setCurrentControlMode(address)
        client.writeRegisters(address, 0x1030, 1, struct.pack('<H', 1000))
        # print("Motor %d ready: supply voltage=%fV", address, client.getVoltage(address))

        client.writeRegisters(address, 0x2006, 1, struct.pack('<f', duty_cycle))
        client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 2) ) # Torque control

        # client.writeRegisters(address, 0x1007, 1, struct.pack('<f', 10.0))
        # client.writeRegisters(address, 0x1008, 1, struct.pack('<f', 0.1))
        # if address in has_21_erevs_per_mrev:
        #     client.writeRegisters(address, 0x1011, 1, struct.pack('<f', 0.55))
        # else:
        #     client.writeRegisters(address, 0x1011, 1, struct.pack('<f', 1.45))
        # client.writeRegisters(address, 0x2007, 1, struct.pack('<f', duty_cycle))
        # client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 3) ) # Velocity control

        # client.writeRegisters(address, 0x1007, 1, struct.pack('<f', 2.0))
        # client.writeRegisters(address, 0x1008, 1, struct.pack('<f', 0.1))
        # client.writeRegisters(address, 0x1009, 1, struct.pack('<f', 25.0))
        # client.writeRegisters(address, 0x100a, 1, struct.pack('<f', 0.01))
        # if address in has_21_erevs_per_mrev:
        #     client.writeRegisters(address, 0x1011, 1, struct.pack('<f', 0.25))
        # else:
        #     client.writeRegisters(address, 0x1011, 1, struct.pack('<f', 1.00))
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

start_time = time.time()
count = 0
while True:
    for address in addresses:
        try:
            data = struct.unpack('<ff', client.readRegisters(address, 0x3000, 2))
            # print(address, data)
        except IOError:
            pass

        count += 1
        if count % 100 == 0:
            freq = count / (time.time() - start_time)
            print("{} \t {}".format(address, freq))
            print("hello!")
            sys.stdout.flush()
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
