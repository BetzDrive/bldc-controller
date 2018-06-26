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
        if 'eac_type' in calibration_obj and calibration_obj['eac_type'] == 'int8':
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
                print('WARNING: Motor driver board does not support encoder angle compensation, try updating the firmware.')
        client.setCurrentControlMode(address)
        client.writeRegisters(address, 0x1030, 1, struct.pack('<H', 1000))
        client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 2) ) # Torque control
        # print("Motor %d ready: supply voltage=%fV", address, client.getVoltage(address))
    else:
        client.writeRegisters(address, 0x1010, 1, struct.pack('<H', angle_mapping[address]) )
        client.writeRegisters(address, 0x1011, 1, struct.pack('<B', 0) )
        client.writeRegisters(address, 0x1012, 1, struct.pack('<B', int(address in needs_flip_phase)) )
        try:
            client.writeRegisters(address, 0x010a, 1, struct.pack('<B', 21 if (address in has_21_erevs_per_mrev) else 14))
        except:
            print "WARNING: Motor driver board does not support erevs_per_mrev, try updating the firmware."

num_grips = 0
start_state = struct.unpack('<f', client.readRegisters(address, 0x3000, 1))[0]

while True:
    address = addresses[0]
    duty_cycle = duty_cycles[0]
    # Close the gripper for 1 second
    last_time = time.time()
    while (time.time() - last_time < 1):
        try:
            client.writeRegisters(address, 0x2006, 1, struct.pack('<f', duty_cycle))
            last_state = struct.unpack('<f', client.readRegisters(address, 0x3000, 1))[0]
        except Exception as e:
            print(str(e))
            pass

    closed_pos = last_state

    # Open the gripper until position is 0
    while (last_state > start_state):
        try:
            client.writeRegisters(address, 0x2006, 1, struct.pack('<f', -duty_cycle))
            last_state = struct.unpack('<f', client.readRegisters(address, 0x3000, 1))[0]
        except Exception as e:
            print(str(e))
            pass

    opened_pos = last_state

    with open("gripper_log.txt", "a") as myfile:
        myfile.write(str(time.time()) + ", " + str(closed_pos) + ", " + str(opened_pos) + "\n")
        myfile.close()

    print (str(time.time()) + ", " + str(closed_pos) + ", " + str(opened_pos) + "\n")

    
