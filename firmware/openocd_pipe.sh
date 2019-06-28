# This file is not meant to be run directly by the user, it is called by gdb
set -m
trap '' INT

openocd -c "gdb_port pipe" -f "openocd_gdb.cfg" -l /dev/null

