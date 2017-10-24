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
Sheet 8 11
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
L LSM6DS3US U7
U 1 1 59741623
P 5900 3950
F 0 "U7" H 5900 4000 60  0000 C CNN
F 1 "LSM6DS3US" H 5900 3900 60  0000 C CNN
F 2 "bldc-controller:LGA-14L-STRICT" H 5900 3950 60  0001 C CNN
F 3 "" H 5900 3950 60  0001 C CNN
F 4 "STMicroelectronics" H 5900 3950 60  0001 C CNN "MFR_NAME"
F 5 "LSM6DS3USTR" H 5900 3950 60  0001 C CNN "MFR_PN"
	1    5900 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 4000 4800 4000
Wire Wire Line
	4800 4000 4800 4500
Wire Wire Line
	4900 4300 4800 4300
Connection ~ 4800 4300
Wire Wire Line
	4900 4400 4800 4400
Connection ~ 4800 4400
$Comp
L GND #PWR070
U 1 1 5974180D
P 4800 4500
F 0 "#PWR070" H 4800 4250 50  0001 C CNN
F 1 "GND" H 4800 4350 50  0000 C CNN
F 2 "" H 4800 4500 50  0000 C CNN
F 3 "" H 4800 4500 50  0000 C CNN
	1    4800 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3500 7100 3500
Wire Wire Line
	7100 3500 7100 3700
Wire Wire Line
	6900 3600 7100 3600
Connection ~ 7100 3600
$Comp
L GND #PWR071
U 1 1 5974183F
P 7100 3700
F 0 "#PWR071" H 7100 3450 50  0001 C CNN
F 1 "GND" H 7100 3550 50  0000 C CNN
F 2 "" H 7100 3700 50  0000 C CNN
F 3 "" H 7100 3700 50  0000 C CNN
	1    7100 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3900 7300 3900
Text HLabel 7300 3900 2    60   Output ~ 0
INT1
Wire Wire Line
	4700 3600 4900 3600
Wire Wire Line
	4700 2900 4700 4100
Text HLabel 4700 2900 1    60   Input ~ 0
VDD
Wire Wire Line
	4900 3500 4700 3500
Connection ~ 4700 3500
Wire Wire Line
	6900 4100 7300 4100
Text HLabel 7300 4100 2    60   Output ~ 0
INT2
NoConn ~ 6900 3700
NoConn ~ 6900 4400
Wire Wire Line
	4700 4100 4900 4100
Connection ~ 4700 3600
Wire Wire Line
	4900 3800 4500 3800
Text HLabel 4500 3800 0    60   Input ~ 0
SCL
Wire Wire Line
	4900 3900 4500 3900
Text HLabel 4500 3900 0    60   BiDi ~ 0
SDA
$Comp
L C_Small C36
U 1 1 597437DF
P 4500 3300
F 0 "C36" V 4400 3300 50  0000 C CNN
F 1 "0.1u" V 4600 3300 50  0000 C CNN
F 2 "bldc-controller:C_0603_IPC_NOMINAL" H 4500 3300 50  0001 C CNN
F 3 "" H 4500 3300 50  0000 C CNN
	1    4500 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	4700 3300 4600 3300
Connection ~ 4700 3300
$Comp
L GND #PWR072
U 1 1 597439B6
P 4300 3500
F 0 "#PWR072" H 4300 3250 50  0001 C CNN
F 1 "GND" H 4300 3350 50  0000 C CNN
F 2 "" H 4300 3500 50  0000 C CNN
F 3 "" H 4300 3500 50  0000 C CNN
	1    4300 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3300 4300 3300
Wire Wire Line
	4300 3000 4300 3500
$Comp
L C_Small C35
U 1 1 598555F7
P 4500 3000
F 0 "C35" V 4400 3000 50  0000 C CNN
F 1 "0.1u" V 4600 3000 50  0000 C CNN
F 2 "bldc-controller:C_0603_IPC_NOMINAL" H 4500 3000 50  0001 C CNN
F 3 "" H 4500 3000 50  0000 C CNN
	1    4500 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	4400 3000 4300 3000
Connection ~ 4300 3300
Wire Wire Line
	4600 3000 4700 3000
Connection ~ 4700 3000
$EndSCHEMATC
