#!/usr/bin/env python3
import serial
import time
import argparse
import struct

import comms
import boards
import utils

ReadOnlyRegs = {}
ReadOnlyRegs['encoder'] = boards.COMM_ROR_ROTOR_POS
ReadOnlyRegs['encoder_raw'] = boards.COMM_ROR_ROTOR_POS_RAW
ReadOnlyRegs['velocity'] = boards.COMM_ROR_ROTOR_VEL
ReadOnlyRegs['id'] = boards.COMM_ROR_CURRENT_DIRECT
ReadOnlyRegs['iq'] = boards.COMM_ROR_CURRENT_QUADRATURE
ReadOnlyRegs['supply'] = boards.COMM_ROR_SUPPLY_V
ReadOnlyRegs['temp'] = boards.COMM_ROR_TEMPERATURE
ReadOnlyRegs['imu'] = boards.COMM_ROR_ACC_X

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Read a sensor from boards.')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_ids',
                        type=str,
                        help='Board ID (separate with comma)')
    parser.add_argument(
        'sensor',
        type=str,
        help=
        'Choose sensor (encoder, encoder_raw, velocity, id, iq, supply, temp, imu)'
    )
    parser.set_defaults(baud_rate=comms.COMM_DEFAULT_BAUD_RATE,
                        offset=comms.COMM_BOOTLOADER_OFFSET)
    args = parser.parse_args()

    ser = serial.Serial(port=args.serial,
                        baudrate=args.baud_rate,
                        timeout=0.004)

    board_ids = [int(bid) for bid in args.board_ids.split(',')]

    client = comms.BLDCControllerClient(ser)

    initialized = boards.initBoards(client, board_ids)

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

    def callback() -> bool:
        boards.clearWDGRST(client)

        try:
            address = ReadOnlyRegs[args.sensor]
            responses = client.readRegisters(board_ids, [address] * num_boards,
                                             [num_regs] * num_boards)
            for i in range(len(responses)):
                if not responses[i]:
                    print('Board {} could not be read.'.format(i))
                    continue

                val = struct.unpack(decode, responses[i])
                bid = board_ids[i]
                print("Board:", bid, message.format(args.sensor, val))

        except (comms.MalformedPacketError, comms.ProtocolError) as e:
            print(e)
            return False

    loop = utils.DebugLoop(callback, iters_per_print=1000)

    loop.loop()
