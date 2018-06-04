#!/usr/bin/env python
from comms import *
import serial
import sys
import time

if len(sys.argv) != 3:
        print("give me a serial port and address")
        exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

address = int(sys.argv[2])

client = BLDCControllerClient(s)

client.leaveBootloader([address])
time.sleep(0.2)
s.reset_input_buffer()

try:
    #client.writeRegisters([address], [0x1003], [1], [struct.pack('<f', 0.01)]) # FOC direct current Kp
    #client.writeRegisters([address], [0x1004], [1], [struct.pack('<f', 0.0)]) # FOC direct current Ki
    #client.writeRegisters([address], [0x1005], [1], [struct.pack('<f', 0.01)]) # FOC quadrature current Kp
    #client.writeRegisters([address], [0x1006], [1], [struct.pack('<f', 0.0)]) # FOC quadrature current Ki
    #client.writeRegisters([address], [0x1040], [1], [struct.pack('<f', 1e-3)]) # Velocity filter parameter
    #client.writeRegisters([address], [0x1030], [1], [struct.pack('<H', 3000)]) # Control watchdog timeout
    #client.writeRegisters([address], [0x2006], [1], [struct.pack('<f', 0.1)])
    client.setCommand([address], [0.5])
    client.setCurrentControlMode([address])
except IOError:
    print "ioerror"
    pass
time.sleep(0.1)

# try:
#     temperature = struct.unpack('<f', client.readRegisters(address, 0x010c, 1))
#     print(temperature[0])
# except IOError:
#     print "ioerror"
