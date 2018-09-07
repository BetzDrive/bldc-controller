Usage

git clone https://github.com/berkeley-open-robotics/bldc-controller.git
cd bldc-controller
git submodule update --recursive --remote --init
sudo apt-get install gcc-arm-none-eabi


## Making Firmware
cd ~/bldc_controller/firmware
make

# to upload
cd ~/bldc_controller/tools
python upload_firmware.py <serial_port> <board_id> ~/bldc_controller/firmware/build/motor_controller.bin
