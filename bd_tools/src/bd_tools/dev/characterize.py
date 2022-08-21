#!/usr/bin/env python
from __future__ import division

import pickle
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from comms import *
from scipy.integrate import odeint
from scipy.optimize import curve_fit

PWM_DUTY_CYCLE = 0.3

if len(sys.argv) >= 3:
    # Data collection mode

    # if len(sys.argv) < 3:
    #     print("give me a serial port and address")
    #     exit()

    port = sys.argv[1]
    s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

    address = int(sys.argv[2])

    client = BLDCControllerClient(s, protocol_v2=True)

    # angle_mapping = {1: 726, 2: 243, 3: 2827, 4: 1125, 5: 7568, 10: 800, 11: 823, 12: 501, 13: 10054, 14: 1008, 15: 775, 16: 22, 17: 1087, 18: 247, 19: 601, 20: 721, 21: 621, 22: 269, 23: 678, 24: 518} # mapping of id to joints

    # needs_flip_phase = [3, 4, 11, 17, 18, 22, 23, 24]

    # has_21_erevs_per_mrev = [2, 13, 18, 19, 20, 21]

    client.leaveBootloader(address)
    time.sleep(0.2)
    s.reset_input_buffer()

    # client.writeRegisters(address, 0x1000, 1, struct.pack('<H', angle_mapping[address]) )
    # client.writeRegisters(address, 0x1002, 1, struct.pack('<B', int(address in needs_flip_phase)) )
    # try:
    #     client.writeRegisters(address, 0x1001, 1, struct.pack('<B', 21 if (address in has_21_erevs_per_mrev) else 14))
    # except:
    #     print "WARNING: Motor driver board does not support erevs_per_mrev, try updating the firmware."

    # Align motor to phase A
    client.writeRegisters(
        address, 0x2003, 1, struct.pack("<f", PWM_DUTY_CYCLE)
    )
    client.writeRegisters(address, 0x2004, 1, struct.pack("<f", 0))
    client.writeRegisters(address, 0x2005, 1, struct.pack("<f", 0))
    client.writeRegisters(
        address, 0x2000, 1, struct.pack("<B", 1)
    )  # Raw PWM control

    time.sleep(0.5)

    client.writeRegisters(address, 0x2003, 1, struct.pack("<f", 0))

    time.sleep(0.5)

    reset = struct.unpack("<B", client.readRegisters(address, 0x300B, 1))[0]
    print("reset: %u" % reset)
    success = struct.unpack("<B", client.readRegisters(address, 0x3009, 1))[0]
    print("success: %u" % success)

    if success:
        client.writeRegisters(
            address, 0x2003, 1, struct.pack("<f", PWM_DUTY_CYCLE)
        )

        time.sleep(0.2)

        client.writeRegisters(address, 0x2003, 1, struct.pack("<f", 0))

        l = struct.unpack("<H", client.readRegisters(address, 0x300A, 1))[0]
        while l == 0:
            l = struct.unpack("<H", client.readRegisters(address, 0x300A, 1))[
                0
            ]
            time.sleep(0.1)
        data = []
        chunk_len = 16
        for i in range(0, int(l / 4), chunk_len):
            a = struct.unpack(
                "<{}f".format(chunk_len),
                client.readRegisters(address, 0x8000 + i, chunk_len),
            )
            data += a

    supply_voltage = struct.unpack(
        "<f", client.readRegisters(address, 0x3004, 1)
    )[0]

    with open("characterize.pkl", "wb") as file:
        pickle.dump({"data": data, "supply_voltage": supply_voltage}, file)
        print("dumped data to file")
else:
    # Use cached data

    with open("characterize.pkl", "rb") as file:
        obj = pickle.load(file)

        if "data" in obj:
            data = obj["data"]
            supply_voltage = obj["supply_voltage"]
        else:
            data = obj
            supply_voltage = 23.9

num_channels = 9
length = len(data) // num_channels

ia = np.array([data[i * num_channels] for i in range(length)])
ib = np.array([data[i * num_channels + 1] for i in range(length)])
ic = np.array([data[i * num_channels + 2] for i in range(length)])
va = np.array([data[i * num_channels + 3] for i in range(length)])
vb = np.array([data[i * num_channels + 4] for i in range(length)])
vc = np.array([data[i * num_channels + 5] for i in range(length)])
vin = np.array([data[i * num_channels + 6] for i in range(length)])
angle = np.array([data[i * num_channels + 7] for i in range(length)])
vel = np.array([data[i * num_channels + 8] for i in range(length)])

time = np.arange(length) / 20000

# Combine current measurements from all three channels
current = (ia - ib - ic) / 2.0

# Find start of step
start_index = np.argmax(current >= 0.1 * np.max(current))

# Find approximate end of step
end_index = np.argmax(current >= 0.95 * np.max(current)) + 100
if end_index > length:
    end_index = length


def model_func(x, a, k, b):
    return a * np.exp(-k * x) + b


p0 = (-1e10, 1e4, 0.25)
opt, pcov = curve_fit(
    model_func, time[start_index:end_index], current[start_index:end_index], p0
)
a, k, b = opt

motor_resistance = supply_voltage * PWM_DUTY_CYCLE / b
motor_inductance = motor_resistance / k


def current_func(y, t0):
    return (
        supply_voltage * PWM_DUTY_CYCLE / motor_inductance
        - motor_resistance / motor_inductance * y
    )


current_sim = odeint(
    current_func, current[start_index], time[start_index:end_index]
)

plt.figure()
plt.plot(
    time[start_index:end_index],
    current[start_index:end_index],
    label="Measured current",
)
plt.plot(
    time[start_index:end_index],
    model_func(time[start_index:end_index], a, k, b),
    label="Fitted current",
)
plt.plot(time[start_index:end_index], current_sim, label="Simulated current")
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.title("Voltage step response")
plt.legend()
plt.show()

# Find effective number of bits for the encoder
encoder_bits = 14
raw_angle = angle / (2 * np.pi) * 2 ** encoder_bits
encoder_enob = encoder_bits - np.log2(np.std(raw_angle))
print(np.std(raw_angle))

# Find RMS current noise
# current_noise = np.sqrt(np.mean(np.square(ic[:start_index])))
current_noise = np.std(ic[:start_index])

print("Resistance (ohms): {}".format(motor_resistance))
print("Inductance (henries): {}".format(motor_inductance))
print("Encoder ENOB: {}".format(encoder_enob))
print("Current stddev (A): {}".format(current_noise))
