#!/usr/bin/env bash
# Read sensor FSM debug registers from a board over RS485.
# Usage: bazelisk run //firmware_baremetal:sensor_debug -- [--serial /dev/ttyUSB0] [--id 1]

SERIAL="/dev/ttyUSB0"
BOARD_ID=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --serial) SERIAL="$2"; shift 2 ;;
        --id)     BOARD_ID="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

python3 - "$SERIAL" "$BOARD_ID" << 'EOF'
import sys
import struct
import time
import serial
from bd_tools import boards, comms

FSM_STATE_NAMES = ['UNINIT', 'INIT_WAIT_1', 'INIT_WAIT_2', 'IDLE', 'WAIT_ACCEL', 'WAIT_TEMP']
I2C_ERR_NAMES   = ['OK', 'ERR_NACK', 'ERR_BUS', 'ERR_TIMEOUT', 'ERR_DMA', 'ERR_BUSY']
I2C_DRV_NAMES   = ['IDLE', 'START_SENT', 'ADDR_W_SENT', 'TX_DATA',
                   'RESTART_SENT', 'ADDR_R_SENT', 'RX_DMA', 'COMPLETE', 'ERROR']

serial_port = sys.argv[1]
board_id    = int(sys.argv[2])

ser = serial.Serial(port=serial_port, baudrate=1000000, timeout=0.01)
client = comms.BLDCControllerClient(ser)
boards.initBoards(client, [board_id])
client.leaveBootloader([board_id])
boards.clearWDGRST(client)
time.sleep(0.3)  # wait for at least one sensor poll cycle (100ms)

data = client.readRegisters([board_id], [0x3005], [1])
temp = struct.unpack('<f', data[0])[0]

data = client.readRegisters([board_id], [0x3050], [4])
fsm, i2c_err, init_errs, drv_state = struct.unpack('<BBBB', data[0])

fsm_name = FSM_STATE_NAMES[fsm]         if fsm       < len(FSM_STATE_NAMES) else str(fsm)
err_name = I2C_ERR_NAMES[i2c_err]       if i2c_err   < len(I2C_ERR_NAMES)  else str(i2c_err)
drv_name = I2C_DRV_NAMES[drv_state]     if drv_state < len(I2C_DRV_NAMES)  else str(drv_state)

print(f"temperature:       {temp:.2f} C")
print(f"sensor FSM:        {fsm_name} ({fsm})")
print(f"I2C error:         {err_name} ({i2c_err})")
print(f"init errors:       {init_errs}")
print(f"I2C driver state:  {drv_name} ({drv_state})")
print()
print("I2C driver states: 0=IDLE 1=START_SENT 2=ADDR_W_SENT 3=TX_DATA")
print("                   4=RESTART_SENT 5=ADDR_R_SENT 6=RX_DMA 7=COMPLETE 8=ERROR")
EOF
