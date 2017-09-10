#!/usr/bin/env python

from comms import *
import serial

s = serial.Serial(port='/dev/tty.usbserial-AI057K87', baudrate=3000000)

print s.BAUDRATES

client = BLDCControllerClient(s)

client.leaveBootloader(0x01)
s.flush()
