#!/usr/bin/env python
from comms import *
import serial
import time
import math

e = 0.00001

if len(sys.argv) != 4:
    print("give me a serial port and two addresses")
    exit()

port = sys.argv[1]
s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.01)

address1 = int(sys.argv[2])
address2 = int(sys.argv[3])

client = BLDCControllerClient(s, True)

def waitForStop(address):
    time.sleep(0.1)
    p = readAngle(address)
    time.sleep(0.1)
    n = readAngle(address)
    while math.fabs(n - p) > e:
        time.sleep(0.1)
        p = n
        n = readAngle(address)
    return n

def readAngle(address):
    return struct.unpack('<f', client.readRegisters(address, 0x3000, 1))[0]

def writeAngle(address, value):
    client.writeRegisters(address, 0x2002, 1, struct.pack('<f', value))

def writeCalibration(address):
    calibrations = client.readCalibration(address)
    client.writeRegisters(address, 0x1000, 1, struct.pack('<H', calibrations['angle']) )
    client.writeRegisters(address, 0x2000, 1, struct.pack('<B', 0) )
    client.writeRegisters(address, 0x1002, 1, struct.pack('<B', calibrations['inv']) )
    client.writeRegisters(address, 0x1001, 1, struct.pack('<B', calibrations['epm']) )
    client.writeRegisters(address, 0x1022, 1, struct.pack('<f', calibrations['torque']))

angle1s = [0, 0]
angle2s = [0, 0]

client.leaveBootloader(address1)
s.flush()
client.leaveBootloader(address2)
s.flush()
time.sleep(1)

# Move to top
writeCalibration(address1)
writeAngle(address1, -0.1)
writeCalibration(address2)
writeAngle(address2, -0.1)
waitForStop(address1)
waitForStop(address2)

# Rotate
writeAngle(address1, -5)
angle1s[0] = waitForStop(address1)
angle2s[0] = waitForStop(address2)
print("board %d stop at %f" % (address1, angle1s[0]))
print("board %d stop at %f" % (address2, angle2s[0]))

# Rotate other direction
writeAngle(address1, 0)
writeAngle(address2, -5)
angle2s[1] = waitForStop(address2)
angle1s[1] = waitForStop(address1)
print("board %d stop at %f" % (address1, angle1s[1]))
print("board %d stop at %f" % (address2, angle2s[1]))

# Set to 0
writeAngle(address1, 0)
writeAngle(address2, 0)
print("board %d average at %f" % (address1, sum(angle1s)/len(angle1s)))
print("board %d average at %f" % (address2, sum(angle2s)/len(angle2s)))

# On-board control calibration:
# Set control constants (0x1003 - 0x1022)
# Turn on control
# Read angle/velocity
# Write position/Torque/Velocity setpoint (0x2006 - 0x2008)
