# List of all the board related files.
BOARDSRC = ../chibios/boards/BLDC_SERVO_CONTROLLER/board.c

# Required include directories
BOARDINC = ../chibios/boards/BLDC_SERVO_CONTROLLER

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
