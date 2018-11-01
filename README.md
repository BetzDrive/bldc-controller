# Getting Started
```
cd ~
git clone https://github.com/berkeley-open-robotics/bldc-controller.git
cd bldc-controller
git submodule update --recursive --remote --init
sudo apt-get install gcc-arm-none-eabi
```

## Compiling
```
cd ~/bldc_controller/firmware
make
```

## Uploading firmware via ST-LINK
```
cd ~/bldc_controller/firmware
make upload
```

## Upload firmware via RS485
```
cd ~/bldc_controller/tools
python upload_firmware.py <serial_port> <board_id> ~/bldc_controller/firmware/build/motor_controller.bin
```

## Upload bootloader via RS485
Use this with caution. If this fails and a power cycle or reboot occurs, the board will have to be programmed directly. In the case of a failed upload, try again immediately. To be safe, first upload and test on an easy-to-remove link such as the gripper or base which are not as difficult to access in case of a failure.

`upload_bootloader.py <serial_port> <board_id> <path_to_file>`

## Serial Transmission Speed Limits
Serial device drivers only permit at most one transmission per millisecond. By default, this is set to 16 milliseconds on our usb to RS485 converters. To change this execute the following terminal commands:

```bash
cat /sys/bus/usb-serial/devices/ttyUSB<id_num>/latency_timer
setserial /dev/ttyUSB<id_num> low_latency
cat /sys/bus/usb-serial/devices/ttyUSB<id_num>/latency_timer
```

(when running the arm, this is done automatically by the ROS control stack)

# Python Scripts
These scripts are made to remotely interface with the boards. They can debug, program, and run the boards. They interface using our custom communications protocol.

## Motor Calibration
To calibrate the motor board, a motor must first be attached. The results of calibration are based on the properties of the given motor.

* `calibrate_encoder.py <serial_port> <board_id> <duty_cycle>`
* `upload_calibration.py <serial_port> <board_id> <calibration_file.json>`

1. Run _calibrate_encoder.py_: for 24V Power Supplies the duty cycle used in testing is 0.3 and for 48V or 50V supplies use half (0.15).
2. Update _calibrations.json_ found in the _bldc-controller-calibration_ repo.
3. Run _upload_calibration.py_ with the updated calibrations.json as the final argument.

After completing these steps, the motor should be controllable.

## Current Command
Use this script to spin a motor with a given current command. A substantial starting point is 0.5 and increase above this with proper supervision! A negative command will reverse the direction.

`current_command.py <serial_port> <board_id> <command>`
