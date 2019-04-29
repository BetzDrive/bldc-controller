#!/usr/bin/env python
import sys
from comms import *
import serial
import time
from math import sin, cos, pi
import argparse

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

client = BLDCControllerClient(s)

time.sleep(0.2)
try:
    print (client.enumerateBoards(addresses))
except:
    print("Failed to receive enumerate response")
time.sleep(0.2)


for address, duty_cycle in zip(addresses, duty_cycles):
    client.leaveBootloader([address])
    s.reset_input_buffer()
    time.sleep(0.2)

    calibration_obj = client.readCalibration([address])
    print(calibration_obj)

    client.setZeroAngle([address], [calibration_obj['angle']])
    client.setInvertPhases([address], [calibration_obj['inv']])
    client.setERevsPerMRev([address], [calibration_obj['epm']])
    client.setTorqueConstant([address], [calibration_obj['torque']])
    client.setPositionOffset([address], [calibration_obj['zero']])
    if 'eac_type' in calibration_obj and calibration_obj['eac_type'] == 'int8':
        print('EAC calibration available')
        try:
            client.writeRegisters([address], [0x1100], [1], [struct.pack('<f', calibration_obj['eac_scale'])])
            client.writeRegisters([address], [0x1101], [1], [struct.pack('<f', calibration_obj['eac_offset'])])
            eac_table_len = len(calibration_obj['eac_table'])
            slice_len = 64
            for i in range(0, eac_table_len, slice_len):
                table_slice = calibration_obj['eac_table'][i:i+slice_len]
                client.writeRegisters([address], [0x1200+i], [len(table_slice)], [struct.pack('<{}b'.format(len(table_slice)), *table_slice)])
        except ProtocolError:
            print('WARNING: Motor driver board does not support encoder angle compensation, try updating the firmware.')
    client.setCurrentControlMode([address])
    client.writeRegisters([address], [0x1030], [1], [struct.pack('<H', 1000)])
    # print("Motor %d ready: supply voltage=%fV", address, client.getVoltage(address))

    client.writeRegisters([address], [0x2006], [1], [struct.pack('<f', duty_cycle)])
    client.writeRegisters([address], [0x2000], [1], [struct.pack('<B', 2)]) # Torque control

    # Setting gains for motor
    client.writeRegisters([address], [0x1003], [1], [struct.pack('<f', 1)])  # DI Kp
    client.writeRegisters([address], [0x1004], [1], [struct.pack('<f', 0)]) # DI Ki
    client.writeRegisters([address], [0x1005], [1], [struct.pack('<f', 1)])  # QI Kp
    client.writeRegisters([address], [0x1006], [1], [struct.pack('<f', 0)]) # QI Ki

start_time = time.time()
count = 0
while True:
    for address in addresses:
        try:
            data = struct.unpack('<ff', client.readRegisters([address], [0x3000], [2])[0])
            # print(address, data)
        except IOError:
            pass

        count += 1
        if count % 100 == 0:
            freq = count / (time.time() - start_time)
            print("{} \t {}".format(address, freq))
            sys.stdout.flush()
    time.sleep(0.01)

