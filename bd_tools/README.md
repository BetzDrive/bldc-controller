# bd_tools

This package contains supporting scripts and tools for communicating with and
testing betz drive motor controllers.

There are 3 types of utilities:
* Console based - these print data live in text form
* Live graph - these read data live and plot it as a time-series
* Recorder - these collect a fixed-size buffer that is saved on-board the MCU

## udev latency

Some usb to serial converters support configurable batch transmission delays.
This value is often an integer representing milliseconds. For optimal
performance, there is a udev rule which configures the latency to be 1ms on
connection of a usb serial device. If this rule is not in place/your tranciever
is not fast enough, expect major communication issues.
