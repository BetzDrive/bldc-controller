#!/usr/bin/env python

from __future__ import division
import argparse
from comms import *
import serial
import time
import sys
import numpy as np
from scipy import signal as sps, stats, interpolate
import matplotlib.pyplot as plt
import math
from math import pi
import json
from collections import OrderedDict

# 14-bit encoder
encoder_ticks_per_rev = 2 ** 14
steps_per_erev = 6
enc_ang_corr_table_size = 257

def midpoint_clamping_svm(angle, magnitude):
    dc = magnitude * np.cos(np.array([angle, angle - (2 / 3 * pi), angle - (4 / 3 * pi)])) / np.sqrt(3) + 0.5
    shift = 0.5 * (1.0 - np.min(dc) - np.max(dc))
    return dc + shift

def circular_mean(a, b):
    return np.arctan2(0.5 * (np.sin(a) + np.sin(b)), 0.5 * (np.cos(a) + np.cos(b)))

def wrap_pi(a):
    return (a + pi) % (2 * pi) - pi

def interp_periodic_periodic(x, xp, fp):
    '''
    Interpolates a periodic function of a periodic argument
    '''

    # Wrap x, xp into range [0, 2*pi)
    x = x % (2 * pi)
    xp = xp % (2 * pi)

    # Sort by xp
    sort_indices = np.argsort(xp)
    xp = xp[sort_indices]
    fp = fp[sort_indices]

    # Unwrap fp
    fp = np.unwrap(fp)

    # Extend to avoid boundary issues
    xp_ext = np.concatenate([xp[-1:] - 2 * pi, xp, xp[:1] + 2 * pi])
    fp_ext = np.unwrap(np.concatenate([fp[-1:], fp, fp[:1]]))

    f_interp = np.interp(x, xp_ext, fp_ext, left=np.nan, right=np.nan)

    assert not np.isnan(np.sum(f_interp))

    return f_interp

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate calibration parameters for a single servo (motor and controller board).')
    parser.add_argument('serial', type=str, help='Serial port')
    parser.add_argument('--baud_rate', type=int, help='Serial baud rate')
    parser.add_argument('board_id', type=int, help='Board ID')
    parser.add_argument('duty_cycle', type=float, help='Duty cycle')
    parser.add_argument('--max_steps', type=int, help='Maximum number of steps')
    parser.add_argument('--delay', type=float, help='Delay between steps')
    parser.add_argument('--start_delay', type=float, help='Delay before first step')
    parser.add_argument('--plot', dest='plot', action='store_true', help='Plot the encoder angle compensation table')
    parser.add_argument('--no-plot', dest='plot', action='store_false', help='Don\'t plot the encoder angle compensation table')
    parser.set_defaults(baud_rate=COMM_DEFAULT_BAUD_RATE, duty_cycle=0.6, max_steps=126, delay=0.05, start_delay=0.5, plot=False)
    args = parser.parse_args()

    #
    # Data collection
    #

    ser = serial.Serial(port=args.serial, baudrate=args.baud_rate, timeout=0.1)
    time.sleep(0.1)

    client = BLDCControllerClient(ser, protocol_v2=True)

    client.leaveBootloader(args.board_id)
    time.sleep(0.2) # Wait for the controller to reset
    ser.reset_input_buffer()

    def set_phase_state(step):
        angle = (step % steps_per_erev) / steps_per_erev * 2 * np.pi
        a, b, c = midpoint_clamping_svm(angle, args.duty_cycle)
        client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', a, b, c))

    client.writeRegisters(args.board_id, 0x1030, 1, struct.pack('<H', 3000)) # Control timeout
    client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', 0, 0, 0))
    client.writeRegisters(args.board_id, 0x2000, 1, struct.pack('<B', 1))

    # Start one step before phase A to avoid boundary issues
    set_phase_state(-1)
    time.sleep(args.start_delay)

    # Forward
    forward_raw_angles = []
    for i in range(args.max_steps):
        set_phase_state(i)
        time.sleep(args.delay)

        raw_angle = struct.unpack('<H', client.readRegisters(args.board_id, 0x3010, 1))[0]

        if i >= steps_per_erev and (i % steps_per_erev == 0) and (abs(forward_raw_angles[0] - raw_angle) < abs(forward_raw_angles[steps_per_erev // 2] - forward_raw_angles[0])):
            break

        forward_raw_angles.append(raw_angle)

    step_count = len(forward_raw_angles)

    # Take one more step to avoid boundary issues
    set_phase_state(step_count)
    time.sleep(args.delay)

    # Backward
    backward_raw_angles = []
    for i in range(step_count - 1, -1, -1):
        set_phase_state(i)
        time.sleep(args.delay)

        raw_angle = struct.unpack('<H', client.readRegisters(args.board_id, 0x3010, 1))[0]

        backward_raw_angles.append(raw_angle)

    client.writeRegisters(args.board_id, 0x2003, 3, struct.pack('<fff', 0, 0, 0))
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

    # Average forward and backward angles, taking phase wrapping into account
    enc_angles = circular_mean(forward_angles, backward_angles)

    # enc_angles = np.array([-1.35776474, -1.31001959, -1.26668464, -1.21625502, -1.15988122,
    #    -1.10676714, -1.05231082, -0.98941761, -0.92173071, -0.86171371,
    #    -0.79498554, -0.7223132 , -0.66057048, -0.60036173, -0.53593454,
    #    -0.47515055, -0.42414569, -0.37045636, -0.31638354, -0.27247334,
    #    -0.22952188, -0.18541993, -0.14285196, -0.10507768, -0.06691991,
    #    -0.02684466,  0.00824515,  0.0435267 ,  0.08168448,  0.12291021,
    #     0.16010924,  0.20037624,  0.24812139,  0.29049761,  0.33440781,
    #     0.38637141,  0.44236171,  0.49317482,  0.55165784,  0.61761901,
    #     0.67648553,  0.73707777,  0.80514817,  0.87341031,  0.93304381,
    #     0.99497829,  1.05902199,  1.11213607,  1.16237394,  1.21529628,
    #     1.26687638,  1.30886911,  1.35335455,  1.39899048,  1.43657301,
    #     1.47358029,  1.5126968 ,  1.55181331,  1.58594439,  1.62295167,
    #     1.66417741,  1.7008012 ,  1.73838373,  1.78171869,  1.82792986,
    #     1.87030608,  1.91881822,  1.97289104,  2.02466289,  2.07720173,
    #     2.13779398,  2.20260466,  2.26204642,  2.32666536,  2.39780372,
    #     2.45858771,  2.51783772,  2.58149792,  2.64324065,  2.69386201,
    #     2.74755134,  2.8021994 ,  2.8464931 ,  2.8900198 ,  2.93527224,
    #     2.97841545,  3.01618972,  3.05415575,  3.09461449,  3.12951255,
    #    -3.11858294, -3.08042517, -3.04034992, -3.00372613, -2.96384263,
    #    -2.91782321, -2.87640572, -2.83326252, -2.78110717, -2.72665085,
    #    -2.67602948, -2.6188887 , -2.55331102, -2.49463626, -2.43289353,
    #    -2.36405614, -2.29483526, -2.23385952, -2.17039107, -2.1040464 ,
    #    -2.04920659, -1.996476  , -1.94125269, -1.8881386 , -1.84384491,
    #    -1.79878422, -1.75199781, -1.7128813 , -1.67472353, -1.63388129,
    #    -1.59361429, -1.55890798, -1.52055846, -1.47914097, -1.44059971,
    #    -1.40129145])
    # step_count = len(enc_angles)

    # Move smallest encoder angle aligned with phase A to the front, for consistency
    smallest_index = np.argmin(np.abs(enc_angles[::steps_per_erev])) * steps_per_erev
    enc_angles = np.roll(enc_angles, -smallest_index)

    mech_angle_slope = -np.sign(np.diff(np.unwrap(enc_angles))[0]) # Encoder angles increase clockwise, mechanical angles increase counterclockwise
    mech_angles = np.linspace(0, 2 * pi, step_count, endpoint=False) * mech_angle_slope

    interp_enc_angles = np.linspace(0, 2 * pi, enc_ang_corr_table_size, endpoint=False)
    interp_mech_angles = interp_periodic_periodic(interp_enc_angles, enc_angles, mech_angles)

    # interp_mech_angles is the sum of a (2*pi)-periodic linear component and a (2*pi)-periodic nonlinear component, w.r.t. the encoder angle
    # The linear component has y-intercept mech_angle_offset and slope mech_angle_slope
    # The nonlinear component is mech_angle_residuals
    mech_angle_offset = np.mean(interp_mech_angles - mech_angle_slope * interp_enc_angles)
    mech_angle_residuals = interp_mech_angles - mech_angle_slope * interp_enc_angles - mech_angle_offset

    # Build encoder angle compensation table
    eac_scale = np.ptp(mech_angle_residuals) / 254
    eac_offset = (np.min(mech_angle_residuals) + np.max(mech_angle_residuals)) / 2
    eac_table = np.round((mech_angle_residuals - eac_offset) / eac_scale).astype(np.int8)

    if args.plot:
        plt.figure()
        plt.plot(interp_enc_angles, mech_angle_residuals, label='float')
        plt.plot(interp_enc_angles, eac_scale * eac_table + eac_offset, '.', label='int8')
        plt.title('Encoder angle compensation')
        plt.xlabel('Encoder angle (rad)')
        plt.ylabel('Mechanical angle residuals (rad)')
        plt.legend()
        plt.show()

    enc_tick_offset = (mech_angle_offset % (2 * pi)) / (2 * pi) * encoder_ticks_per_rev
    erevs_per_mrev = int(round(step_count / steps_per_erev))

    calibration_dict = OrderedDict()
    calibration_dict['angle'] = int(round(enc_tick_offset))
    calibration_dict['inv'] = int(erevs_per_mrev > 0)
    calibration_dict['epm'] = abs(erevs_per_mrev)
    calibration_dict['eac_type'] = 'int8'
    calibration_dict['eac_scale'] = eac_scale
    calibration_dict['eac_offset'] = eac_offset
    calibration_dict['eac_table'] = eac_table.tolist()

    print(json.dumps(calibration_dict))
