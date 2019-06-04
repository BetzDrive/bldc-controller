from __future__ import print_function
from comms import *

# Read Only Registers
COMM_ROR_ROTOR_POS = 0x3000
COMM_ROR_ROTOR_VEL = 0x3001
COMM_ROR_CURRENT_DIRECT = 0x3002
COMM_ROR_CURRENT_QUADRATURE = 0x3003
COMM_ROR_SUPPLY_V = 0x3004
COMM_ROR_TEMPERATURE = 0x3005
COMM_ROR_ACC_X = 0x3006
COMM_ROR_ACC_Y = 0x3007
COMM_ROR_ACC_Z = 0x3008
COMM_ROR_ROTOR_POS_RAW = 0x3010

def initBoards(client, board_ids):
    if type(board_ids) == int:
        board_ids = [board_ids]

    client.resetSystem([0])
    time.sleep(0.2)
    client.enterBootloader([0])
    time.sleep(0.2)

    client.resetInputBuffer()

    success = []
    for bid in board_ids:
        print("Enumerating Board ID:", bid)
        found_id = False
        # Retry until hear back from board 
        while not found_id:
            try:
                response = client.enumerateBoards(bid)
                print("received id:", response)
                if response == bid:
                    found_id = True
                time.sleep(0.2)
            except ProtocolError as e:
                print("Comms Error:", e)
                client.resetInputBuffer()

        confirmed = False
        # Retry until confirmed
        while not confirmed:
            try:
                print("Requesting Confirm")
                confirmed = client.confirmBoards(bid)
                time.sleep(0.2)
            except ProtocolError as e:
                print("Comms Error:", e)
                client.resetInputBuffer()

        success.append(bid)

    print("Enumerated boards:", success)
    return True

def loadMotorCalibration(client, board_ids, duty_cycles, mode):
    for board_id, duty_cycle in zip(board_ids, duty_cycles):
        success = False
        while not success:
            try:
                print("Calibrating board:", board_id)
                client.leaveBootloader([board_id])
                client.resetInputBuffer()
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
    
                # Velocity IIR Alpha Term
                client.writeRegisters([board_id], [0x1040], [1], [struct.pack('<f', 0.01)])

                # Upload current offsets
                offset_data = struct.pack('<fff', calibration_obj['ia_off'], calibration_obj['ib_off'], calibration_obj['ic_off'])
                client.writeRegisters([board_id], [0x1050], [3], [offset_data])
    
                if mode == 'torque':
                    client.writeRegisters([board_id], [0x2006], [1], [struct.pack('<f', duty_cycle)])
                    client.writeRegisters([board_id], [0x2000], [1], [struct.pack('<B', 2)]) # Torque control
                elif mode == 'raw_pwm':
                    client.writeRegisters([board_id], [0x2003], [1], [struct.pack('<f', duty_cycle)])
                    client.writeRegisters([board_id], [0x2004], [1], [struct.pack('<f', duty_cycle)])
                    client.writeRegisters([board_id], [0x2005], [1], [struct.pack('<f', duty_cycle)])
                    client.writeRegisters([board_id], [0x2000], [1], [struct.pack('<B', 1)]) # PWM control
    
                # Setting gains for motor
                client.writeRegisters([board_id], [0x1003], [1], [struct.pack('<f', 5)])  # DI Kp
                client.writeRegisters([board_id], [0x1004], [1], [struct.pack('<f', 0)]) # DI Ki
                client.writeRegisters([board_id], [0x1005], [1], [struct.pack('<f', 10)])  # QI Kp
                client.writeRegisters([board_id], [0x1006], [1], [struct.pack('<f', 0)]) # QI Ki
                success = True
            except (ProtocolError, struct.error, TypeError):
                print("Failed to calibrate board, retrying...")
 
