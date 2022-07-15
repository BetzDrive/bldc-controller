#!/usr/bin/env python3

import argparse
import json
import struct
import time

import matplotlib.pyplot as plt
import numpy as np
import serial

from bd_tools import boards, comms

# 14-bit encoder
encoder_ticks_per_rev = 2 ** 14
phase_state_list = [
    (1, 0, 0),
    (1, 1, 0),
    (0, 1, 0),
    (0, 1, 1),
    (0, 0, 1),
    (1, 0, 1),
]


def parser_args():
    parser = argparse.ArgumentParser(
        description="Calibrate the encoder on a motor controller board."
    )
    boards.addBoardArgs(parser)
    parser.add_argument("duty_cycle", type=float, help="Duty cycle")
    parser.add_argument(
        "--max_steps", type=int, help="Maximum number of steps"
    )
    parser.add_argument("--delay", type=float, help="Delay between steps")
    parser.set_defaults(
        baud_rate=comms.COMM_DEFAULT_BAUD_RATE,
        duty_cycle=0.6,
        max_steps=126,
        delay=0.05,
    )
    return parser.parse_args()


def action(args):
    #
    # Data collection
    #

    board_ids = [int(bid) for bid in args.board_ids.split(",")]
    if len(board_ids) > 1:
        print("Only one motor controller calibrated at a time")
        return

    board_ids = [board_ids[0]]

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)

    client = comms.BLDCControllerClient(ser)

    boards.initBoards(client, board_ids)

    client.leaveBootloader(board_ids)
    time.sleep(0.2)

    client.resetInputBuffer()

    def set_phase_state(phase_state):
        a, b, c = phase_state
        targets = [
            a * args.duty_cycle,
            b * args.duty_cycle,
            c * args.duty_cycle,
        ]

        while True:
            try:
                boards.driveMotor(client, board_ids, targets, "phase")
                break
            except (comms.MalformedPacketError, comms.ProtocolError):
                print("Phase state set failed. Retrying.")

    # Clear currently loaded current offsets
    offset_data = struct.pack("<fff", 0, 0, 0)
    client.writeRegisters(board_ids, [0x1050], [3], [offset_data])

    client.setWatchdogTimeout(board_ids, [1000])
    set_phase_state((0, 0, 0))

    time.sleep(0.2)

    # First, read floating currents to calculate offset
    reset = client.resetRecorderBuffer(board_ids)[0]
    print("reset: %u" % reset)
    success = client.startRecorder(board_ids)[0]
    print("started: %u" % success)

    data = []
    data_length = 0
    while data_length == 0:
        try:
            data_length = client.getRecorderLength(board_ids)[0]
        except comms.MalformedPacketError as e:
            print(e)
            continue
        time.sleep(0.1)

    for i in range(0, data_length, comms.COMM_NUM_RECORDER_ELEMENTS):
        # Grab the recorder data
        try:
            data.extend(client.getRecorderElement(board_ids, [i]))
        except (comms.MalformedPacketError, comms.ProtocolError):
            print("Missed packet")

    f, axarr = plt.subplots(1, sharex=True)

    ia = [e[0] for e in data]
    ib = [e[1] for e in data]
    ic = [e[2] for e in data]
    len_data = len(data)
    ia_offset = sum(ia) / len_data
    ib_offset = sum(ib) / len_data
    ic_offset = sum(ic) / len_data

    print("Phase A Offset:", ia_offset)
    print("Phase B Offset:", ib_offset)
    print("Phase C Offset:", ic_offset)

    # Start one step before phase A to avoid boundary issues
    set_phase_state(phase_state_list[-1])
    time.sleep(args.delay)

    # Forward
    forward_raw_angles = []
    for i in range(args.max_steps):
        set_phase_state(phase_state_list[i % 6])
        time.sleep(args.delay)

        while True:
            try:
                raw_angle = client.getRawRotorPosition(board_ids)[0]
                break
            except (comms.MalformedPacketError, comms.ProtocolError):
                print("Missed packet")

        if (
            i > 4
            and abs(forward_raw_angles[0] - raw_angle)
            < abs(forward_raw_angles[1] - forward_raw_angles[0]) / 3.0
        ):
            break

        forward_raw_angles.append(raw_angle)

    step_count = len(forward_raw_angles)

    # Take one more step to avoid boundary issues
    set_phase_state(phase_state_list[step_count % 6])
    time.sleep(args.delay)

    # Backward
    backward_raw_angles = []
    for i in range(step_count - 1, -1, -1):
        set_phase_state(phase_state_list[i % 6])
        time.sleep(args.delay)

        while True:
            try:
                raw_angle = client.getRawRotorPosition(board_ids)[0]
                break
            except (comms.MalformedPacketError, comms.ProtocolError):
                print("Missed packet")

        backward_raw_angles.append(raw_angle)

    set_phase_state((0, 0, 0))

    ser.close()

    #
    # Data analysis
    #

    # Flip backward raw angles to correspond with forward raw angles
    backward_raw_angles = backward_raw_angles[::-1]

    forward_raw_angles = np.array(forward_raw_angles)
    backward_raw_angles = np.array(backward_raw_angles)

    # Convert to radians
    forward_angles = forward_raw_angles / encoder_ticks_per_rev * 2 * np.pi
    backward_angles = backward_raw_angles / encoder_ticks_per_rev * 2 * np.pi

    # Phase unwrapping
    forward_angles = np.unwrap(forward_angles)
    backward_angles = np.unwrap(backward_angles)

    # Average forward and backward measurements
    angles = (forward_angles + backward_angles) / 2

    # Shift smallest encoder value to front of array
    wrapped_angles = angles % (2 * np.pi)
    zero_index = np.argmin(wrapped_angles)
    angles = np.unwrap(np.roll(wrapped_angles, -zero_index))

    # Guess erevs/mrev
    angle_slope = np.mean(np.diff(angles))
    steps_per_mrev = int(np.round(2 * np.pi / angle_slope))
    erevs_per_mrev = steps_per_mrev // 6  # Could be negative

    # Convert to electrical angle
    elec_angles = angles * erevs_per_mrev

    # Subtract expected trend
    elec_angle_residuals = elec_angles - (
        np.r_[: len(elec_angles)] + zero_index
    ) * (2 * np.pi / 6)

    # Find smallest raw angle aligned with phase A
    elec_angle_offset = np.mean(elec_angle_residuals)
    wrapped_elec_angle_offset = elec_angle_offset % (2 * np.pi)
    if erevs_per_mrev < 0:
        wrapped_elec_angle_offset -= 2 * np.pi
    angle_offset = wrapped_elec_angle_offset / erevs_per_mrev
    erev_start = angle_offset / (2 * np.pi) * encoder_ticks_per_rev

    print("    erev_start: {:5d}".format(int(round(erev_start))))
    print("erevs_per_mrev: {:5d}".format(abs(erevs_per_mrev)))
    print("   flip_phases: {:5d}".format(int(erevs_per_mrev > 0)))

    size = str(input("What size is the motor? (S/L)\n"))
    upload_data = {
        "inv": int(erevs_per_mrev > 0),
        "epm": abs(erevs_per_mrev),
        "angle": int(erev_start),
        "torque": (1.45, 0.6)[size.lower() == "s"],
        "zero": 0.0,
        "ia_off": ia_offset,
        "ib_off": ib_offset,
        "ic_off": ic_offset,
    }

    print("Calibration")
    print(upload_data)

    with open("calibrations.json", "w") as outfile:
        json.dump([upload_data], outfile)

    # mech_angle = np.linspace(
    #    0, 2 * np.pi, len(elec_angle_residuals), endpoint=False
    # )
    # plt.plot(
    #    mech_angle,
    #    (elec_angle_residuals - elec_angle_offset)
    #    / erevs_per_mrev
    #    / (2 * np.pi)
    #    * encoder_ticks_per_rev,
    # )
    # plt.title("Encoder angle residuals")
    # plt.xlabel("Mechanical angle (rad)")
    # plt.ylabel("Encoder angle residual (counts)")
    # plt.show()

    # # Apply smoothing
    # angle_residuals = sps.savgol_filter(angle_residuals, 31, 3, mode="wrap")
    # smoothed_angles = angle_residuals + indices * angle_slope


if __name__ == "__main__":
    action(parser_args())
