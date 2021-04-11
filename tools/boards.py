"""Class for managing standard board operations."""
import argparse
import json
import serial
import struct
import time
from typing import Any, List

import comms

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


def argparse_template(
        parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """For convenience, uses argparse to declare some standard arguments."""
    parser.add_argument('serial_port', type=str, help='Serial port')
    parser.add_argument('board_ids',
                        help='Board IDs',
                        type=lambda s: [int(item) for item in s.split(',')])
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE)

    return parser


class BoardManager:
    """Used to manage the init/standard procedures for the BLDC."""

    # Defining Control Mode ID Lookup
    CONTROL_MODES = {
        'current': 0,
        'phase': 1,
        'torque': 2,
        'velocity': 3,
        'position': 4,
        'pos_vel': 5,
        'pos_ff': 6,
        'pwm': 7
    }

    def __init__(self,
                 serial_port: str,
                 baud_rate: int,
                 board_ids: List[int],
                 timeout: float = 0.004,
                 init_boards: bool = False,
                 **kwargs: Any):
        """
        Inits serial interface and stores board information.

        Arguments:
            port: serial device port which the RS485 bus is connected.
            baud_rate: baud rate of the RS485 connection.
            board_ids: list of boards connected to this serial connection.
            timeout: time for system to wait on serial line for comms response.
                This should generally be below 1ms but there can be edge cases
                where it takes longer.
            init_boards: whether to fully reset boards. Only needs to be run
                the first time boards are booted.
        """
        self._client = comms.BLDCControllerClient(
            serial.Serial(port=port, baudrate=baud_rate, timeout=timeout))
        self._boards = board_ids
        self._kwargs = kwargs

        if init_boards:
            self.initBoards()

    def initBoards(self):
        """
        Returns all boards to bootloader, enumerates, and jumps to firmware.
        """

        # Return to bootloader
        self._client.resetSystem([comms.COMM_BROADCAST_ADDRESS])
        self._client.enterBootloader([comms.COMM_BROADCAST_ADDRESS])

        self._client.resetInputBuffer()

        for bid in board_ids:
            print(f"Enumerating board ID: {bid}")
            found_id = False
            # Retry until hear back from board
            while not found_id:
                try:
                    response = self._client.enumerateBoards(bid)
                    print(
                        f"Received enumerate op success from board: {response}"
                    )
                    if response == bid:
                        found_id = True
                except comms.ProtocolError as e:
                    print("Comms Error:", e)
                    self._client.resetInputBuffer()

            confirmed = False
            # Retry until confirmed
            while not confirmed:
                try:
                    print(f"Requested confirmation from board: {bid}")
                    confirmed = self._client.confirmBoards(bid)
                except comms.ProtocolError as e:
                    print("Comms Error:", e)
                    self._client.resetInputBuffer()

            print(f"Enumerated board: {bid}")

    def loadCalibrationFromJSON(self, board_id, calibration_obj):
        self._client.setZeroAngle([board_id], [calibration_obj['angle']])
        self._client.setInvertPhases([board_id], [calibration_obj['inv']])
        self._client.setERevsPerMRev([board_id], [calibration_obj['epm']])
        self._client.setTorqueConstant([board_id], [calibration_obj['torque']])
        self._client.setPositionOffset([board_id], [calibration_obj['zero']])

        if 'eac_type' in calibration_obj and calibration_obj[
                'eac_type'] == 'int8':
            print('EAC calibration available')
            try:
                self._client.writeRegisters(
                    [board_id], [0x1100], [1],
                    [struct.pack('<f', calibration_obj['eac_scale'])])
                self._client.writeRegisters(
                    [board_id], [0x1101], [1],
                    [struct.pack('<f', calibration_obj['eac_offset'])])
                eac_table_len = len(calibration_obj['eac_table'])
                slice_len = 64
                for i in range(0, eac_table_len, slice_len):
                    table_slice = calibration_obj['eac_table'][i:i + slice_len]
                    self._client.writeRegisters(
                        [board_id], [0x1200 + i], [len(table_slice)], [
                            struct.pack('<{}b'.format(len(table_slice)), *
                                        table_slice)
                        ])
            except comms.ProtocolError:
                print(
                    'WARNING: Motor driver board does not support encoder angle'
                    'compensation, try updating the firmware.')
        self._client.setCurrentControlMode([board_id])

        # Upload current offsets
        offset_data = struct.pack('<fff', calibration_obj['ia_off'],
                                  calibration_obj['ib_off'],
                                  calibration_obj['ic_off'])
        self._client.writeRegisters([board_id], [0x1050], [3], [offset_data])

    def initMotor(self, board_ids):
        success = False
        while not success:
            try:
                self._client.setWatchdogTimeout(board_ids,
                                                [1000] * len(board_ids))

                # Setting gains for motor
                self._client.setDirectCurrentKp(board_ids,
                                                [0.5] * len(board_ids))
                self._client.setDirectCurrentKi(board_ids,
                                                [0.1] * len(board_ids))
                self._client.setQuadratureCurrentKp(board_ids,
                                                    [1.0] * len(board_ids))
                self._client.setQuadratureCurrentKi(board_ids,
                                                    [0.2] * len(board_ids))

                success = True
            except (comms.MalformedPacketError, comms.ProtocolError):
                print("Failed to calibrate board, retrying...")
        print("Finished calibration of boards:", board_ids)

    def driveMotor(self, board_ids, actuations, mode):
        """This should be placed in a try/catch to handle comms errors."""
        if (len(board_ids) == 1):
            actuations = [actuations]

        for board_id, actuation in zip(board_ids, actuations):
            control_mode = self.CONTROL_MODES[mode]
            self._client.writeRegisters(
                [board_id], [0x2000], [1],
                [struct.pack('<B', control_mode)])  # Set Control Mode

            if mode == 'current':
                self._client.writeRegisters(
                    [board_id], [0x2001], [2],
                    [struct.pack('<ff', actuation[0], actuation[1])])
            elif mode == 'phase':
                self._client.writeRegisters([board_id], [0x2003], [3], [
                    struct.pack('<fff', actuation[0], actuation[1],
                                actuation[2])
                ])
            elif mode == 'torque':
                self._client.writeRegisters([board_id], [0x2006], [1],
                                            [struct.pack('<f', actuation[0])])
            elif mode == 'velocity':
                self._client.writeRegisters([board_id], [0x2007], [1],
                                            [struct.pack('<f', actuation[0])])
            elif mode == 'position':
                self._client.writeRegisters([board_id], [0x2008], [1],
                                            [struct.pack('<f', actuation[0])])
            elif mode == 'pos_ff':
                self._client.writeRegisters(
                    [board_id], [0x2008], [2],
                    [struct.pack('<ff', actuation[0], actuation[1])])
            elif mode == 'pwm':
                self._client.writeRegisters([board_id], [0x200A], [1],
                                            [struct.pack('<f', actuation[0])])
