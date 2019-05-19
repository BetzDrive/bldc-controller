#!/usr/bin/env python
from __future__ import print_function

import sys
import serial
import time
from math import sin, cos, pi
import argparse

from comms import *
from boards import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Read temperature sensor from boards.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board ID (separate with comma)')
    parser.add_argument('duty_cycles', type=str, help='Duty Cycles per Board (separate with comma)')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    board_ids = [int(bid) for bid in args.board_ids.split(',')]
    duty_cycles = [float(dc) for dc in args.duty_cycles.split(',')]

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)

    client = BLDCControllerClient(ser)
    initialized = initBoards(client, board_ids)
    
    ser.reset_input_buffer()
    
    for board_id, duty_cycle in zip(board_ids, duty_cycles):
        success = False
        while not success:
            try:
                print("Calibrating board:", board_id)
                client.leaveBootloader([board_id])
                ser.reset_input_buffer()
                time.sleep(0.2)
    
                calibration_obj = client.readCalibration([board_id])
                print(calibration_obj)
    
                client.setZeroAngle([board_id], [calibration_obj['angle']])
                client.setInvertPhases([board_id], [calibration_obj['inv']])
                client.setERevsPerMRev([board_id], [calibration_obj['epm']])
                client.setTorqueConstant([board_id], [calibration_obj['torque']])
                client.setPositionOffset([board_id], [calibration_obj['zero']])
                if 'eac_type' in calibration_obj and calibration_obj['eac_type'] == 'int8':
                    print('EAC calibration available')
                    try:
                        client.writeRegisters([board_id], [0x1100], [1], [struct.pack('<f', calibration_obj['eac_scale'])])
                        client.writeRegisters([board_id], [0x1101], [1], [struct.pack('<f', calibration_obj['eac_offset'])])
                        eac_table_len = len(calibration_obj['eac_table'])
                        slice_len = 64
                        for i in range(0, eac_table_len, slice_len):
                            table_slice = calibration_obj['eac_table'][i:i+slice_len]
                            client.writeRegisters([board_id], [0x1200+i], [len(table_slice)], [struct.pack('<{}b'.format(len(table_slice)), *table_slice)])
                    except ProtocolError:
                        print('WARNING: Motor driver board does not support encoder angle compensation, try updating the firmware.')
                client.setCurrentControlMode([board_id])
                client.writeRegisters([board_id], [0x1030], [1], [struct.pack('<H', 1000)])
                # print("Motor %d ready: supply voltage=%fV", board_id, client.getVoltage(board_id))
    
                #client.writeRegisters([board_id], [0x1040], [1], [struct.pack('<f', 0.01)])
    
                client.writeRegisters([board_id], [0x2006], [1], [struct.pack('<f', duty_cycle)])
                client.writeRegisters([board_id], [0x2000], [1], [struct.pack('<B', 2)]) # Torque control
    
                # Setting gains for motor
                client.writeRegisters([board_id], [0x1003], [1], [struct.pack('<f', 1)])  # DI Kp
                client.writeRegisters([board_id], [0x1004], [1], [struct.pack('<f', 0)]) # DI Ki
                client.writeRegisters([board_id], [0x1005], [1], [struct.pack('<f', 1)])  # QI Kp
                client.writeRegisters([board_id], [0x1006], [1], [struct.pack('<f', 0)]) # QI Ki
                success = True
            except (ProtocolError, struct.error, TypeError):
                print("Failed to calibrate board, retrying...")
    
    start_time = time.time()
    count = 0
    while True:
        for board_id in board_ids:
            try:
                #data = struct.unpack('<ff', client.readRegisters([board_id], [0x3000], [2])[0])
                client.writeRegisters([board_id], [0x2000], [1], [struct.pack('<B', 2)]) # Torque control
                # print(board_id, data)
            except (ProtocolError, struct.error):
                print("Failed to communicate with board: ", board_id)
                pass
    
        count += 1
        if count % 100 == 0:
            now = time.time()
            freq = count / (now - start_time)
            print("Comm Frequency:{}".format(freq))
            start_time = now
            sys.stdout.flush()
            count = 0
        time.sleep(0.01)

