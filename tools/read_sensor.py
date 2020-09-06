#!/usr/bin/env python2
from __future__ import print_function

import serial
import time
import argparse

from comms import *
from boards import *

ReadOnlyRegs = {}
ReadOnlyRegs['encoder'] = COMM_ROR_ROTOR_POS
ReadOnlyRegs['encoder_raw'] = COMM_ROR_ROTOR_POS_RAW
ReadOnlyRegs['velocity'] = COMM_ROR_ROTOR_VEL
ReadOnlyRegs['id'] = COMM_ROR_CURRENT_DIRECT
ReadOnlyRegs['iq'] = COMM_ROR_CURRENT_QUADRATURE
ReadOnlyRegs['supply'] = COMM_ROR_SUPPLY_V
ReadOnlyRegs['temp'] = COMM_ROR_TEMPERATURE
ReadOnlyRegs['imu'] = COMM_ROR_ACC_X

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Read a sensor from boards.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board ID (separate with comma)')
    parser.add_argument('sensor', type=str, help='Choose sensor (encoder, encoder_raw, velocity, id, iq, supply, temp, imu)')
    parser.add_argument('--init_boards', type=bool, store_true=True, help='Serial baud rate')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.004)

    board_ids = [int(bid) for bid in args.board_ids.split(',')]

    client = BLDCControllerClient(ser)

    if args.init_boards:
        initialized = initBoards(client, board_ids)

    for bid in board_ids:
        client.leaveBootloader([bid])

    sen = args.sensor
    address = ReadOnlyRegs[sen]
    decode = '<f'
    num_regs = 1
    message = '{0}: {1[0]}'

    if sen == 'encoder_raw':
        decode = '<H'
        message = '{0}: {1[0]} ticks'
    elif sen == 'encoder':
        message = '{0}: {1[0]} radians'
    elif sen == 'velocity':
        message = '{0}: {1[0]} rad/s'
    elif sen == 'id':
        message = '{0}: {1[0]} amps'
    elif sen == 'iq':
        message = '{0}: {1[0]} amps'
    elif sen == 'supply':
        message = '{0}: {1[0]} volts'
    elif sen == 'temp':
        message = '{0}: {1[0]} degC'
    if sen == 'imu':
        decode = '<hhh'
        num_regs = 3
        message = '{0} -> x:{1[0]}, y:{1[1]}, z:{1[2]}'

    num_boards = len(board_ids)

    total_messages = 0
    protocal_error = 0
    struct_error = 0
    while initialized:
        total_messages += 1
        crashed = client.checkWDGRST()
        if crashed:
            print('boards:', crashed, 'have crashed')
        try:
            address = ReadOnlyRegs[args.sensor]
            responses = client.readRegisters(board_ids, [address]*num_boards, [num_regs]*num_boards)
            for i in range(len(responses)):
                val = struct.unpack(decode, responses[i])
                bid = board_ids[i]
                print("Board:", bid, message.format(args.sensor , val))
        except ProtocolError as err:
            print(err)
            pass
        except struct.error as err:
            print(err)
            pass
        time.sleep(0.1)
        if total_messages % 100 == 0:
            print("total_messages: ", total_messages)
            print("protocal_errors: ", protocal_error)
            print("struct_errors: ", struct_error)
            print("total_error_percent: ", (protocal_error + struct_error) / total_messages)
    print("Exiting.")
