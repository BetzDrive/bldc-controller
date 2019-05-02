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
    parser = argparse.ArgumentParser(description='Read temperature sensor from boards.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids', type=str, help='Board ID (separate with comma)')
    parser.add_argument('sensor', type=str, help='Choose sensor (encoder, encoder_raw, velocity, id, iq, supply, temp, accel)')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, offset=COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=2.0)
    time.sleep(0.1)
    
    board_ids = [int(bid) for bid in args.board_ids.split(',')]

    client = BLDCControllerClient(ser)

    initialized = initBoards(client, board_ids)

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
        decode = '<iii'
        num_regs = 3
        message = '{0} -> x:{1[0]}, y:{1[1]}, z:{1[2]}'

    num_boards = len(board_ids)
    while initialized:
        try:
            address = ReadOnlyRegs[args.sensor]
            responses = client.readRegisters(board_ids, [address]*num_boards, [num_regs]*num_boards)
            for i in range(len(responses)):
                val = struct.unpack(decode, responses[i])
                bid = board_ids[i]
                print("Board:", bid, message.format(args.sensor , val))
        except IOError:
            print("ioerror")
            pass
        time.sleep(0.1)
    
    print("Exiting.")
