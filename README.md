# Getting Started
```
cd ~
git clone https://github.com/berkeleyopenarms/bldc-controller.git
cd bldc-controller
git submodule update --recursive --remote --init
sudo apt-get install gcc-arm-none-eabi
```

## Compiling
```
cd ~/bldc_controller/<firmware or bootloader>
make
```

## Debugging via ST-LINK
To debug during execution, you can attach the debugger at any time using the following code. Keep in mind this is done with a direct connection to the hardware via an ST-Link.
```
cd ~/bldc_controller/<firmware or bootloader>
openocd
----
cd ~/bldc_controller/<firmware or bootloader>/build
gdb-arm-none-eabi <compiled file>.elf
(gdb) target extended-remote :3333
```

This functionality is also built into the make files as `make debug`.

## Uploading via ST-LINK
```
cd ~/bldc_controller/<firmware or bootloader>
make upload
```

## Upload firmware via RS485
```
cd ~/bldc_controller/tools
python upload_firmware.py <serial_port> <board_ids> ~/bldc_controller/firmware/build/motor_controller.bin
```

## Upload bootloader via RS485
Use this with caution. If this fails and a power cycle or reboot occurs, the board will have to be programmed directly. In the case of a failed upload, disable the enumeration procedure and try again with the same board ID. To be safe, first upload and test on an easy-to-remove link such as the gripper or base which are not as difficult to access in case of a failure.

`upload_bootloader.py <serial_port> <board_id> <path_to_file>`

## Serial Transmission Speed Limits
USB Serial device drivers only permit at most one transmission per millisecond. By default, this is set to 16 milliseconds on our usb to RS485 converters. To change this execute the following terminal commands:

```bash
cat /sys/bus/usb-serial/devices/ttyUSB<id_num>/latency_timer
setserial /dev/ttyUSB<id_num> low_latency
cat /sys/bus/usb-serial/devices/ttyUSB<id_num>/latency_timer
```

(when running the arm, this is done automatically by the ROS control stack)

# Python Scripts
These scripts are made to remotely interface with the boards. They can debug, program, and run the boards. They interface using our custom communications protocol. Most scripts have been fitted with argparse which will give a description of the arguments when given the '-h' option. (i.e. `read_sensor.py -h`)

## Motor Calibration
To calibrate the motor board, a motor must first be attached without a load for optimal calibration. The results of calibration are based on the properties of the given motor.

* `calibrate_encoder.py <serial_port> <board_id> <duty_cycle>`
* `upload_calibration.py <serial_port> <board_id>`

1. Run _calibrate_encoder.py_: for 24V Power Supplies the duty cycle used in testing is 0.3 and for 48V or 50V supplies use half (0.15). This will generate a file called calibrations.json
2. Run _upload_calibration.py_ with the updated calibrations.json and the generated file will be loaded onto the board.

After completing these steps, the motor should be controllable.

## Motor Control
Use this script to spin a motor with a given current command. A substantial starting point is 0.5 and increase above this with proper supervision! A negative command will reverse the direction. Description of what arguments each mode takes can be found with the '-h' option.

`control_motor.py <serial_port> <board_id> <mode> <command>`

To control multiple motors,

```
control_motor.py <serial_port> 1,2,3 torque 0.5,0.25,0
control_motor.py <serial_port> 1,2,3 current [0,0.5],[0,0.25],[0,0]
```

## Read Sensor
This script lets a user read from any of the sensors on-board the device.

`read_sensor.py <serial_port> <board_ids> <sensor>`

## View Control Loop
Plots the control target and the resulting quadrature/direct current commands. This is low frequency so use the recorder to get a better sense of the internal state of the motor driver.

## Recording
These scripts start and read off an on-board buffer of the commutation at each control cycle. There are included plotting scripts. This interfaces directly with the Recorder class in the firmware.

# System Design
The BLDC is built on ChibiOS. The operating system allows for quick development and thread management which have been useful for this fast-paced project. To supplement missing features, the STM32 library has been included.

## Common
In both firmware and bootloader, the primary system is the communication code. This is designed to allow users to read/write variables as well as reprogram the board remotely. When a board receives a packet, a small green LED will blink to indicate it has identified the message.

## Bootloader
This small system is primarily meant as a method to allow updates of the firmware without a programmer. On command, it will move the program counter to the start of the firmware code.

### LED Indicator
During normal operation, the bootloader will pulse the RGB LED blue.

## Firmware
The firmware code starts by initializing hardware devices and then splits into a number of threads.

### LED Indicator [low priority]
During normal operation, the firmware will pulse the RGB LED green. If a red or blue pulse is seen while in firmware, the driver chip Fault and/or OTW pins have activated. Refer to the DRV8312 datasheet for the type of fault.

### Sensor Thread [low priority]
Pulls updates from the accelerometer and temperature sensor.
  
### Control Thread [normal priority]
Pulls updates from ADC for current/voltage measurements as well as queues reads to the absolute encoder. With this data, FOC is computed with output duty cycles to STM32 timing hardware.
  
### Communication Thread [high priority]
Used to update motor targets and read current position, velocity, accelerometer data, temperature, etc.

### Independent Watchdog Thread [high priority]
Used to keep the system live while protecting from the chance of a hard fault or any other unforseen errors. When a lock-up occurs, the system will reset into bootloader which will then check the watchdog reset flag. If it is active, the bootloader will return the system to firmware mode.
