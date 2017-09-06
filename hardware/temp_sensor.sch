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
Sheet 11 11
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
L LM75B U10
U 1 1 597F6ABF
P 5900 4050
F 0 "U10" H 5900 4100 60  0000 C CNN
F 1 "LM75B" H 5900 4000 60  0000 C CNN
F 2 "bldc-controller:TSSOP-8" H 5900 4050 60  0001 C CNN
F 3 "" H 5900 4050 60  0001 C CNN
F 4 "NXP" H 5900 4050 60  0001 C CNN "MFR_NAME"
F 5 "LM75BDP,118" H 5900 4050 60  0001 C CNN "MFR_PN"
	1    5900 4050
	1    0    0    -1  
$EndComp
Text HLabel 5000 3700 1    60   Input ~ 0
VDD
Wire Wire Line
	5000 3700 5000 3800
Wire Wire Line
	4800 3800 5100 3800
Wire Wire Line
	5100 4300 5000 4300
Wire Wire Line
	5000 4300 5000 4400
$Comp
L GND #PWR086
U 1 1 597F7067
P 5000 4400
F 0 "#PWR086" H 5000 4150 50  0001 C CNN
F 1 "GND" H 5000 4250 50  0000 C CNN
F 2 "" H 5000 4400 50  0000 C CNN
F 3 "" H 5000 4400 50  0000 C CNN
	1    5000 4400
	1    0    0    -1  
$EndComp
$Comp
L C_Small C50
U 1 1 597F7E46
P 4700 3800
F 0 "C50" V 4650 3850 50  0000 L CNN
F 1 "0.1u" V 4750 3850 50  0000 L CNN
F 2 "bldc-controller:C_0603_IPC_NOMINAL" H 4700 3800 50  0001 C CNN
F 3 "" H 4700 3800 50  0000 C CNN
	1    4700 3800
	0    1    1    0   
$EndComp
Connection ~ 5000 3800
Wire Wire Line
	4600 3800 4500 3800
Wire Wire Line
	4500 3800 4500 3900
$Comp
L GND #PWR087
U 1 1 597F7E95
P 4500 3900
F 0 "#PWR087" H 4500 3650 50  0001 C CNN
F 1 "GND" H 4500 3750 50  0000 C CNN
F 2 "" H 4500 3900 50  0000 C CNN
F 3 "" H 4500 3900 50  0000 C CNN
	1    4500 3900
	1    0    0    -1  
$EndComp
Text HLabel 5000 4000 0    60   Input ~ 0
SCL
Text HLabel 5000 4100 0    60   BiDi ~ 0
SDA
Wire Wire Line
	5000 4000 5100 4000
Wire Wire Line
	5000 4100 5100 4100
$Comp
L GND #PWR088
U 1 1 597F8760
P 6800 4000
F 0 "#PWR088" H 6800 3750 50  0001 C CNN
F 1 "GND" H 6800 3850 50  0000 C CNN
F 2 "" H 6800 4000 50  0000 C CNN
F 3 "" H 6800 4000 50  0000 C CNN
	1    6800 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3800 6800 3800
Wire Wire Line
	6800 3800 6800 4000
Wire Wire Line
	6700 3900 6800 3900
Connection ~ 6800 3900
Wire Wire Line
	6800 4000 6700 4000
Connection ~ 6800 4000
Text HLabel 6800 4300 2    60   Output ~ 0
OS
Wire Wire Line
	6700 4300 6800 4300
$EndSCHEMATC
