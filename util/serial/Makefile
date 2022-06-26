include serial.mk

CC = gcc

SRCS = $(SERIAL_SRCS) \
	   test/test_serial.cpp
HDRS = $(SERIAL_HDRS)

build_test:
	$(CC) $(SRCS) -I$(HDRS) -o test_serial

run_test: build_test
	./test_serial
