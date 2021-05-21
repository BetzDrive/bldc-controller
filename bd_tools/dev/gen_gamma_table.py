#!/usr/bin/env python

import sys

NUM_STEPS = 2 ** 8
MAX_VALUE = 52500
VALUES_PER_LINE = 8
GAMMA = 2.8

sys.stdout.write("const uint16_t led_gamma_table[] = {")

for i in range(NUM_STEPS):
    if i % VALUES_PER_LINE == 0:
        sys.stdout.write("\r\n  ")
    value = int(round((float(i) / (NUM_STEPS - 1)) ** GAMMA * MAX_VALUE))
    sys.stdout.write("{:5d}, ".format(value))

sys.stdout.write("\r\n};\r\n")
