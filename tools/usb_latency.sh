#!/bin/bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 0 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
