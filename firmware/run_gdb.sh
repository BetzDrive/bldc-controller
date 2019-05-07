#!/bin/bash

GDB_LINK="https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2?revision=d830f9dd-cd4f-406d-8672-cca9210dd220?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,8-2018-q4-major"
GDB=arm-none-eabi-gdb

if ! [ -x "$(command -v $GDB)" ] ; then
  echo "$GDB is missing.  You can download it here:"
  echo $GDB_LINK
fi

set -e

PWD=`pwd`

$GDB $PWD/build/motor_controller.elf \
  -ex "target extended-remote | $PWD/openocd_pipe.sh" \
  -x gdbinit.txt
#  -ex "directory ." \
