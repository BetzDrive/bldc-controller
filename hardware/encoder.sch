EESchema Schematic File Version 2
LIBS:bldc-controller
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stepper-driver
LIBS:bldc-controller-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 11
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L AS5047D U6
U 1 1 597506AB
P 6500 4000
F 0 "U6" H 6500 3900 60  0000 C CNN
F 1 "AS5047D" H 6500 4100 60  0000 C CNN
F 2 "bldc-controller:TSSOP-14" H 6500 4000 60  0001 C CNN
F 3 "" H 6500 4000 60  0001 C CNN
F 4 "AMS" H 6500 4000 60  0001 C CNN "MFR_NAME"
F 5 "AS5047D-ATSM" H 6500 4000 60  0001 C CNN "MFR_PN"
F 6 "Mouser" H 6500 4000 60  0001 C CNN "DIST_NAME"
F 7 "985-AS5047D-ATSM" H 6500 4000 60  0001 C CNN "DIST_PN"
	1    6500 4000
	1    0    0    -1  
$EndComp
Text HLabel 5600 3400 1    60   Input ~ 0
VDD
Text HLabel 5050 3800 0    60   Input ~ 0
CSN
Text HLabel 5600 3900 0    60   Input ~ 0
CLK
Text HLabel 5600 4000 0    60   Output ~ 0
MISO
Text HLabel 5600 4100 0    60   Input ~ 0
MOSI
NoConn ~ 7200 3900
NoConn ~ 7200 4000
NoConn ~ 7200 4100
NoConn ~ 7200 4500
Connection ~ 5600 4500
Wire Wire Line
	5800 4500 5600 4500
Wire Wire Line
	5600 4300 5600 4600
Wire Wire Line
	5800 4300 5600 4300
Wire Wire Line
	5600 4100 5800 4100
Wire Wire Line
	5600 4000 5800 4000
Wire Wire Line
	5600 3900 5800 3900
Wire Wire Line
	5050 3800 5800 3800
Connection ~ 5600 3500
Wire Wire Line
	5600 3500 5800 3500
Wire Wire Line
	5600 3400 5600 3600
Wire Wire Line
	4400 3500 4400 3600
Wire Wire Line
	4400 3200 4400 3300
Wire Wire Line
	5600 3600 5800 3600
Text HLabel 4400 3200 1    60   Input ~ 0
VDD
Wire Wire Line
	5300 3700 5300 3800
Connection ~ 5300 3800
Text HLabel 5300 3400 1    60   Input ~ 0
VDD
Wire Wire Line
	5300 3400 5300 3500
$Comp
L C_Small C34
U 1 1 597506D0
P 4400 3400
F 0 "C34" H 4410 3470 50  0000 L CNN
F 1 "0.1u" H 4410 3320 50  0000 L CNN
F 2 "bldc-controller:C_0603_IPC_NOMINAL" H 4400 3400 50  0001 C CNN
F 3 "" H 4400 3400 50  0000 C CNN
	1    4400 3400
	1    0    0    -1  
$EndComp
$Comp
L R_Small R28
U 1 1 597506D7
P 5300 3600
F 0 "R28" H 5330 3620 50  0000 L CNN
F 1 "47k" H 5330 3560 50  0000 L CNN
F 2 "bldc-controller:R_0603_IPC_NOMINAL" H 5300 3600 50  0001 C CNN
F 3 "" H 5300 3600 50  0000 C CNN
	1    5300 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR066
U 1 1 597506DE
P 5600 4600
F 0 "#PWR066" H 5600 4350 50  0001 C CNN
F 1 "GND" H 5600 4450 50  0000 C CNN
F 2 "" H 5600 4600 50  0000 C CNN
F 3 "" H 5600 4600 50  0000 C CNN
	1    5600 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR067
U 1 1 597506E4
P 4400 3600
F 0 "#PWR067" H 4400 3350 50  0001 C CNN
F 1 "GND" H 4400 3450 50  0000 C CNN
F 2 "" H 4400 3600 50  0000 C CNN
F 3 "" H 4400 3600 50  0000 C CNN
	1    4400 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 3500 7400 3500
Wire Wire Line
	7200 3600 7400 3600
Text HLabel 7400 3500 2    60   Output ~ 0
A
Text HLabel 7400 3600 2    60   Output ~ 0
B
$EndSCHEMATC
