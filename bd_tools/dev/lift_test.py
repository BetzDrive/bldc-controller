#!/usr/bin/env python
from tools.comms import *
import serial
import sys
import time
from math import sin, cos, pi

MAX_CYCLES = 1000

target_accurracy = 0.05
dif_gain = 10


def setArmPosition(
    goal, zero_right, zero_left, pos_right, pos_left, duty_cycle, client
):
    goal_left = goal - zero_left
    goal_right = goal - zero_right
    overheated = False
    while (not abs(pos_left - goal_left) / goal_left < target_accurracy) or (
        not abs(pos_right - goal_right) / goal_right < target_accurracy
    ):
        try:
            dif_pos = (pos_right - zero_right) - (pos_left - zero_left)
            left_duty_cycle = (
                duty_cycle * (pos_left - goal_left) / goal_left
            ) + (dif_pos * dif_gain)
            right_duty_cycle = (
                duty_cycle * (pos_right - goal_right) / goal_right
            ) - (dif_pos * dif_gain)
            client.writeRegisters(
                [right_address, left_address],
                [0x2006] * 2,
                [1] * 2,
                [
                    struct.pack("<f", right_duty_cycle),
                    struct.pack("<f", left_duty_cycle),
                ],
            )
            state_right = struct.unpack(
                "<ffffff",
                client.readRegisters([right_address], [0x3000], [6])[0],
            )
            state_left = struct.unpack(
                "<ffffff",
                client.readRegisters([left_address], [0x3000], [6])[0],
            )
            pos_right = state_right[0]
            pos_left = state_left[0]
            temperature = max([state_left[5], state_right[5]])

            if temperature > 70:
                overheated = True
            elif overheated and temperature < 50:
                overheated = False
        except Exception as e:
            print(str(e))
            pass

        if overheated:
            raise "Overheated"
    return pos_right, pos_left


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(
            "give me a serial port, right_address, left_address, and duty cycle"
        )
        exit()

    port = sys.argv[1]
    s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)

    right_address = int(sys.argv[2])
    left_address = int(sys.argv[3])
    duty_cycles = float(sys.argv[4])

    addresses = [right_address, left_address]

    client = BLDCControllerClient(s)

    for address in addresses:
        client.leaveBootloader([address])
        time.sleep(0.2)
        s.reset_input_buffer()

        calibration_obj = client.readCalibration([address])

        client.setZeroAngle([address], [calibration_obj["angle"]])
        client.setInvertPhases([address], [calibration_obj["inv"]])
        client.setERevsPerMRev([address], [calibration_obj["epm"]])
        client.setTorqueConstant([address], [calibration_obj["torque"]])
        client.setPositionOffset([address], [calibration_obj["zero"]])
        if (
            "eac_type" in calibration_obj
            and calibration_obj["eac_type"] == "int8"
        ):
            print("EAC calibration available")
            try:
                client.writeRegisters(
                    [address],
                    [0x1100],
                    [1],
                    [struct.pack("<f", calibration_obj["eac_scale"])],
                )
                client.writeRegisters(
                    [address],
                    [0x1101],
                    [1],
                    [struct.pack("<f", calibration_obj["eac_offset"])],
                )
                eac_table_len = len(calibration_obj["eac_table"])
                slice_len = 64
                for i in range(0, eac_table_len, slice_len):
                    table_slice = calibration_obj["eac_table"][
                        i : i + slice_len
                    ]
                    client.writeRegisters(
                        [address],
                        [0x1200 + i],
                        [len(table_slice)],
                        [
                            struct.pack(
                                "<{}b".format(len(table_slice)), *table_slice
                            )
                        ],
                    )
            except ProtocolError:
                print(
                    "WARNING: Motor driver board does not support encoder angle compensation, try updating the firmware."
                )
        client.setCurrentControlMode([address])
        client.writeRegisters(
            [address], [0x1030], [1], [struct.pack("<H", 1000)]
        )
        client.writeRegisters(
            [address], [0x2000], [1], [struct.pack("<B", 2)]
        )  # Torque control
        # print("Motor %d ready: supply voltage=%fV", address, client.getVoltage(address))

    num_lifts = 0
    start_pos = struct.unpack(
        "<f", client.readRegisters([address], [0x3000], [1])[0]
    )[0]
    overheated = False

    di_list = []
    qi_list = []
    time_list = []

    pos_right = struct.unpack(
        "<ffffff", client.readRegisters([right_address], [0x3000], [6])[0]
    )[0]
    pos_left = struct.unpack(
        "<ffffff", client.readRegisters([left_address], [0x3000], [6])[0]
    )[0]
    zero_right = pos_right
    zero_left = pos_left

    while True:  # num_lifts < MAX_CYCLES:
        if not overheated:
            duty_cycle = duty_cycles
        else:
            duty_cycle = 0.0

        # Lift the arm up
        print("lift")
        pos_right, pos_left = setArmPosition(
            -2, zero_right, zero_left, pos_right, pos_left, duty_cycle, client
        )

        # Put arm down
        print("lower")
        pos_right, pos_left = setArmPosition(
            2, zero_right, zero_left, pos_right, pos_left, duty_cycle, client
        )

        num_lifts += 1
