from __future__ import print_function

import struct
import time

from bd_tools import comms

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
    time.sleep(0.1)
    client.enterBootloader([0])
    time.sleep(0.1)

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
                time.sleep(0.1)
            except comms.ProtocolError as e:
                print("Comms Error:", e)
                client.resetInputBuffer()

        confirmed = False
        # Retry until confirmed
        while not confirmed:
            try:
                print("Requesting Confirm")
                confirmed = client.confirmBoards(bid)
                time.sleep(0.1)
            except comms.ProtocolError as e:
                print("Comms Error:", e)
                client.resetInputBuffer()

        success.append(bid)

    print("Enumerated boards:", success)
    return True


def loadCalibrationFromJSON(client, board_id, calibration_obj):
    client.setZeroAngle([board_id], [calibration_obj['angle']])
    client.setInvertPhases([board_id], [calibration_obj['inv']])
    client.setERevsPerMRev([board_id], [calibration_obj['epm']])
    client.setTorqueConstant([board_id], [calibration_obj['torque']])
    client.setPositionOffset([board_id], [calibration_obj['zero']])

    if 'eac_type' in calibration_obj and calibration_obj['eac_type'] == 'int8':
        print('EAC calibration available')
        try:
            client.writeRegisters(
                [board_id], [0x1100], [1],
                [struct.pack('<f', calibration_obj['eac_scale'])])
            client.writeRegisters(
                [board_id], [0x1101], [1],
                [struct.pack('<f', calibration_obj['eac_offset'])])
            eac_table_len = len(calibration_obj['eac_table'])
            slice_len = 64
            for i in range(0, eac_table_len, slice_len):
                table_slice = calibration_obj['eac_table'][i:i + slice_len]
                client.writeRegisters(
                    [board_id], [0x1200 + i], [len(table_slice)], [
                        struct.pack('<{}b'.format(len(table_slice)), *
                                    table_slice)
                    ])
        except ProtocolError:
            print(
                'WARNING: Motor driver board does not support encoder angle compensation, try updating the firmware.'
            )
    client.setCurrentControlMode([board_id])

    # Upload current offsets
    offset_data = struct.pack('<fff', calibration_obj['ia_off'],
                              calibration_obj['ib_off'],
                              calibration_obj['ic_off'])
    client.writeRegisters([board_id], [0x1050], [3], [offset_data])


def initMotor(client, board_ids):
    success = False
    while not success:
        try:
            client.setWatchdogTimeout(board_ids, [1000] * len(board_ids))

            # Setting gains for motor
            client.setDirectCurrentKp(board_ids, [0.5] * len(board_ids))
            client.setDirectCurrentKi(board_ids, [0.1] * len(board_ids))
            client.setQuadratureCurrentKp(board_ids, [1.0] * len(board_ids))
            client.setQuadratureCurrentKi(board_ids, [0.2] * len(board_ids))

            success = True
        except (comms.MalformedPacketError, comms.ProtocolError):
            print("Failed to calibrate board, retrying...")
    print("Finished calibration of boards:", board_ids)


# Defining Control Mode ID Lookup
control_modes = {
    'current': 0,
    'phase': 1,
    'torque': 2,
    'velocity': 3,
    'position': 4,
    'pos_vel': 5,
    'pos_ff': 6,
    'pwm': 7
}


# This should be placed in a try/catch to handle comms errors
def driveMotor(client, board_ids, actuations, mode):
    if (len(board_ids) == 1):
        actuations = [actuations]

    for board_id, actuation in zip(board_ids, actuations):
        control_mode = control_modes[mode]
        client.writeRegisters(
            [board_id], [0x2000], [1],
            [struct.pack('<B', control_mode)])  # Set Control Mode

        if mode == 'current':
            client.writeRegisters(
                [board_id], [0x2001], [2],
                [struct.pack('<ff', actuation[0], actuation[1])])
        elif mode == 'phase':
            client.writeRegisters([board_id], [0x2003], [3], [
                struct.pack('<fff', actuation[0], actuation[1], actuation[2])
            ])
        elif mode == 'torque':
            client.writeRegisters([board_id], [0x2006], [1],
                                  [struct.pack('<f', actuation[0])])
        elif mode == 'velocity':
            client.writeRegisters([board_id], [0x2007], [1],
                                  [struct.pack('<f', actuation[0])])
        elif mode == 'position':
            client.writeRegisters([board_id], [0x2008], [1],
                                  [struct.pack('<f', actuation[0])])
        elif mode == 'pos_ff':
            client.writeRegisters(
                [board_id], [0x2008], [2],
                [struct.pack('<ff', actuation[0], actuation[1])])
        elif mode == 'pwm':
            client.writeRegisters([board_id], [0x200A], [1],
                                  [struct.pack('<f', actuation[0])])


def clearWDGRST(client):
    # If there's a watchdog reset, clear the reset and perform any
    # configuration again
    crashed = client.checkWDGRST()
    if crashed != []:
        try:
            client.clearWDGRST(crashed)
        except (comms.ProtocolError, comms.MalformedPacketError):
            return False

    return True
