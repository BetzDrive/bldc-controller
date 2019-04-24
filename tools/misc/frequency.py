#!/usr/bin/env python
import sys
sys.path.append("..")

from comms import *
import serial
import time

port_default = "/dev/ttyUSB0"

if len(sys.argv) < 2:
        print("give me a serial port and motor_ids(es)")
        exit()

motor_ids = [int(x) for x in sys.argv[1].split(",")]

if len(sys.argv) >= 3:
    port = sys.argv[2]
else:
    port = port_default

s = serial.Serial(port=port, baudrate=1000000, timeout=0.1)

client = BLDCControllerClient(s)

client.leaveBootloader(motor_ids)
s.reset_input_buffer()

for id in motor_ids:
        success = False
        for attempt in range(5):
            try:
                client._ser.read_all()
                print("Calibrating motor %d..." % id)
                calibrations = client.readCalibration([id])
                print(calibrations)
                client.setZeroAngle([id], [calibrations['angle']])
                client.setInvertPhases([id], [calibrations['inv']])
                client.setERevsPerMRev([id], [calibrations['epm']])
                client.setTorqueConstant([id], [calibrations['torque']])
                client.setPositionOffset([id], [calibrations['zero']])
                #client.setZeroAngle([id], [1169])
                #client.setInvertPhases([id], [1])
                #client.setERevsPerMRev([id], [14])
                #client.setTorqueConstant([id], [1.45])
                #client.setPositionOffset([id], [0.0])
                #client.setCurrentControlMode([id])
                #starting_angles[id] = 0.0
                client.writeRegisters([id], [0x1030], [1], [struct.pack('<H', 1000)])
                print("Motor %d ready: supply voltage=%fV", id, client.getVoltage([id])[0])
                success = True
                break
            except Exception as e:
                print(str(e))
                time.sleep(0.2)
        if not success:
            print("Could not calibrate motors")
            exit()


last_time = time.time()

thous_packs = 0

while True:
    client.sum_time = 0
    errors = 0

    # Update user every 1000 packets.
    for _ in range(1000):
        try:
            states = client.setCommandAndGetState(motor_ids, [0.3]*len(motor_ids))
            for state in states:
                if state == None:
                    errors += 1
        except Exception as e:
            errors += 1
            print(str(e))
            pass

    thous_packs += 1

    if errors:
        with open("spinner_log.txt", "a") as myfile:
            myfile.write(str(time.time()) + ", " + str(thous_packs) + ", " + str(errors) + "\n")
        myfile.close()
    
    temp_time = time.time()
    diff = time.time() - last_time
    last_time = temp_time
    print ("k packets: " + str(thous_packs) +
            " Frequency: " + str(1000.0 / diff) + 
            " Millis: " + str(diff/1000.0) +
            " Errors: " + str(errors))
