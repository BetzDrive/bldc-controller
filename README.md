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

# Serial Transmission Speed Limits
Serial device drivers only permit at most one transmission per millisecond. By default, this is set to 16 milliseconds on our usb to rs485 converters. To change this execute the following terminal commands (it may be preferable to add this to one's bashrc or make a bash script for quick execution) The default _id_num_ is 0 but there will be more if multiple serial devices are plugged in:

`cat /sys/bus/usb-serial/devices/ttyUSB<id_num>/latency_timer`

`setserial /dev/ttyUSB<id_num> low_latency`

`cat /sys/bus/usb-serial/devices/ttyUSB<id_num>/latency_timer`

If the number printed to the terminal does not change after executing these lines then the timer needs sudo to change (execute the echo line with sudo).

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

## Torque Control
Use this to spin a motor at a given torque target. A substantial starting pointis 0.5 and increase above this with proper supervision! A negative duty cycle will reverse the direction.

`duty_cycle.py <serial_port> <board_id> <duty_cycle>`

## Upload Bootloader
Use this with caution. If this fails and a power cycle or reboot occurs, the board will have to be programmed directly. In the case of a failed upload, try again immediately. To be safe, first upload and test on an easy-to-remove link such as the gripper or base which are not as difficult to access in case of a failure.

`upload_bootloader.py <serial_port> <board_id> <path_to_file>`



