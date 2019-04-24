#!/usr/bin/env python
import sys
sys.path.append("..")

from comms import *
import serial
import time
from math import sin, cos, pi

MAX_CYCLES = 1000

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

for address, duty_cycle in zip(addresses, duty_cycles):
    client.leaveBootloader([address])
    time.sleep(0.2)
    s.reset_input_buffer()

    calibration_obj = client.readCalibration([address])

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
    client.writeRegisters([address], [0x2000], [1], [struct.pack('<B', 2)]) # Torque control
    # print("Motor %d ready: supply voltage=%fV", address, client.getVoltage(address))

num_grips = 0
start_pos = struct.unpack('<f', client.readRegisters([address], [0x3000], [1])[0])[0]
overheated = False

di_list = []
qi_list = []
time_list = []

while num_grips < 1: # MAX_CYCLES:
    address = addresses[0]
    if not overheated:
        duty_cycle = duty_cycles[0]
    else:
        duty_cycle = 0.0
    # Close the gripper for 1 second
    last_time = time.time()
    while (time.time() - last_time < 1):
        try:
            client.writeRegisters([address], [0x2006], [1], [struct.pack('<f', duty_cycle)])
            state = struct.unpack('<ffffff', client.readRegisters([address], [0x3000], [6])[0])
            di_list.append(state[2]) 
            qi_list.append(state[3])
            time_list.append(time.time())
        except Exception as e:
            print(str(e))
            pass

    #max_current = max(qi_list[-10:])
    closed_pos = state[0]

    # Open the gripper until position is the starting position 
    while (state[0] > start_pos):
        try:
            client.writeRegisters([address], [0x2006], [1], [struct.pack('<f', -duty_cycle)])
            state = struct.unpack('<ffffff', client.readRegisters([address], [0x3000], [6])[0])
            di_list.append(state[2]) 
            qi_list.append(state[3])
            time_list.append(time.time())
        except Exception as e:
            print(str(e))
            pass

    opened_pos = state[0]

    temperature = state[5]
    if temperature > 70:
        overheated = True
    elif overheated and temperature < 50:
        overheated = False

    with open("gripper_log.txt", "a") as myfile:
        myfile.write(str(time.time()) + ", " + str(closed_pos) + ", " + str(opened_pos) + ", " + str(temperature) + ", " + str(max_current) + "\n")
        myfile.close()

    print (num_grips)
    print (str(time.time()) + ", " + str(closed_pos) + ", " + str(opened_pos) + ", " + str(temperature) + ", " + str(max_current) + "\n")
    num_grips += 1

with open("gripper_current_log.txt", "a") as myfile:
    for i in range(len(time_list)):
        myfile.write(str(time_list[i]) + ", " + str(di_list[i]) + ", " + str(qi_list[i]) + "\n")
    myfile.close()

    
