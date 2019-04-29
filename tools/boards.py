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
    client.leaveBootloader(board_ids)
    time.sleep(0.2)
    try:
        print("Received Board ID:", client.enumerateBoards(board_ids))
    except IOError as e:
        print("Comms Error:", e)
        return False
    
    time.sleep(0.2)
    client.leaveBootloader(board_ids)
    time.sleep(0.2)

    return True;
