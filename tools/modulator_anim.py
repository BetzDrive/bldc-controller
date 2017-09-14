#!/usr/bin/env python

import numpy as np
from numpy import cos, pi
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

TBC_SCALE_FACTOR = 2. * math.sqrt(3.) / 3.

def sinusoidal_duty_cycles(angle, amplitude):
    dc_a = cos(angle)
    dc_b = cos(angle - 2. / 3. * pi)
    dc_c = cos(angle - 4. / 3. * pi)

    return np.array([dc_a, dc_b, dc_c]) * amplitude

def tbc_duty_cycles(angle, amplitude):
    dc = sinusoidal_duty_cycles(angle, amplitude * TBC_SCALE_FACTOR)

    top_shift = 1. - np.max(dc, axis=0)
    bottom_shift = 1. + np.min(dc, axis=0)

    return np.where(top_shift < bottom_shift, dc + top_shift, dc - bottom_shift)

angles = np.linspace(0, 4 * pi, 1001)
dc = np.zeros((3, len(angles)))
dc_a, dc_b, dc_c = dc

fig, (ax1, ax2) = plt.subplots(1, 2)

plot_a, = ax1.plot(angles, dc_a, color='r', label='A')
plot_b, = ax1.plot(angles, dc_b, color='g', label='B')
plot_c, = ax1.plot(angles, dc_c, color='b', label='C')
plot_avg, = ax1.plot(angles, np.mean(dc, axis=0), color='k', label='Avg.')
ax1.set_title('Phase voltages')
ax1.set_xlabel('Angle (rad)')
ax1.set_ylabel('Voltage (V)')
ax1.set_ylim(-2, 2)
ax1.legend()

plot_ab, = ax2.plot(angles, dc_a - dc_b, color='y', label='A - B')
plot_bc, = ax2.plot(angles, dc_b - dc_c, color='c', label='B - C')
plot_ca, = ax2.plot(angles, dc_c - dc_a, color='m', label='C - A')
ax2.set_title('Phase voltage differences')
ax2.set_xlabel('Angle (rad)')
ax2.set_ylabel('Voltage difference (V)')
ax2.set_ylim(-4, 4)
ax2.legend()

def anim_update(i):
    if i < 100:
        amplitude = i / 100.
    else:
        amplitude = 1.0 - (i - 100) / 100.
    dc = tbc_duty_cycles(angles, amplitude)
    dc_a, dc_b, dc_c = dc

    plot_a.set_data(angles, dc_a)
    plot_b.set_data(angles, dc_b)
    plot_c.set_data(angles, dc_c)
    plot_avg.set_data(angles, np.mean(dc, axis=0))
    plot_ab.set_data(angles, dc_a - dc_b)
    plot_bc.set_data(angles, dc_b - dc_c)
    plot_ca.set_data(angles, dc_c - dc_a)

    return [plot_a, plot_b, plot_c, plot_ab, plot_bc, plot_ca]

anim = animation.FuncAnimation(fig, anim_update, np.arange(0, 200), interval=25, blit=False)

plt.show()
