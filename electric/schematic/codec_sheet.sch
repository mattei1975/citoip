EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:microchip
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
LIBS:citoip_lib2
LIBS:citoIP-cache
EELAYER 25 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 2 3
Title "Audio codec"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L TLV320AIC3254 U?
U 1 1 5AA93FDC
P 6150 3700
F 0 "U?" H 5000 4900 60  0000 L CNN
F 1 "TLV320AIC3254" H 7400 2500 60  0000 R CNN
F 2 "" H 4950 3750 60  0001 C CNN
F 3 "" H 4950 3750 60  0001 C CNN
	1    6150 3700
	1    0    0    -1  
$EndComp
Text GLabel 12550 4400 2    60   BiDi ~ 0
RADIO_I2S_DIN
Text GLabel 12550 4700 2    60   BiDi ~ 0
RADIO_I2S_FSYNC
Text GLabel 12550 4100 2    60   BiDi ~ 0
RADIO_I2S_DOUT
Text GLabel 12550 3200 2    60   BiDi ~ 0
RADIO_I2C_SCL
Text GLabel 12550 3500 2    60   BiDi ~ 0
RADIO_I2C_SDA
Text GLabel 12550 3800 2    60   BiDi ~ 0
RADIO_I2S_CLK
$Comp
L R R?
U 1 1 5AAA4791
P 12200 3200
F 0 "R?" V 12100 3000 50  0000 R CNN
F 1 "R0R" V 12100 3050 50  0000 L CNN
F 2 "" V 12130 3200 50  0001 C CNN
F 3 "" H 12200 3200 50  0001 C CNN
F 4 "XXX" V 12200 3200 60  0001 C CNN "PN"
F 5 "0603" V 12100 3300 50  0000 L CNN "Shape"
	1    12200 3200
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5AAA479A
P 12200 3500
F 0 "R?" V 12100 3300 50  0000 R CNN
F 1 "R0R" V 12100 3350 50  0000 L CNN
F 2 "" V 12130 3500 50  0001 C CNN
F 3 "" H 12200 3500 50  0001 C CNN
F 4 "XXX" V 12200 3500 60  0001 C CNN "PN"
F 5 "0603" V 12100 3600 50  0000 L CNN "Shape"
	1    12200 3500
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5AAA47A3
P 12200 3800
F 0 "R?" V 12100 3600 50  0000 R CNN
F 1 "R0R" V 12100 3650 50  0000 L CNN
F 2 "" V 12130 3800 50  0001 C CNN
F 3 "" H 12200 3800 50  0001 C CNN
F 4 "XXX" V 12200 3800 60  0001 C CNN "PN"
F 5 "0603" V 12100 3900 50  0000 L CNN "Shape"
	1    12200 3800
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5AAA47AC
P 12200 4100
F 0 "R?" V 12100 3900 50  0000 R CNN
F 1 "R0R" V 12100 3950 50  0000 L CNN
F 2 "" V 12130 4100 50  0001 C CNN
F 3 "" H 12200 4100 50  0001 C CNN
F 4 "XXX" V 12200 4100 60  0001 C CNN "PN"
F 5 "0603" V 12100 4200 50  0000 L CNN "Shape"
	1    12200 4100
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5AAA47B5
P 12200 4400
F 0 "R?" V 12100 4200 50  0000 R CNN
F 1 "R0R" V 12100 4250 50  0000 L CNN
F 2 "" V 12130 4400 50  0001 C CNN
F 3 "" H 12200 4400 50  0001 C CNN
F 4 "XXX" V 12200 4400 60  0001 C CNN "PN"
F 5 "0603" V 12100 4500 50  0000 L CNN "Shape"
	1    12200 4400
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5AAA47BE
P 12200 4700
F 0 "R?" V 12100 4500 50  0000 R CNN
F 1 "R0R" V 12100 4550 50  0000 L CNN
F 2 "" V 12130 4700 50  0001 C CNN
F 3 "" H 12200 4700 50  0001 C CNN
F 4 "XXX" V 12200 4700 60  0001 C CNN "PN"
F 5 "0603" V 12100 4800 50  0000 L CNN "Shape"
	1    12200 4700
	0    -1   1    0   
$EndComp
Text GLabel 11850 3200 0    60   BiDi ~ 0
CODEC_I2C_SCL
Text GLabel 11850 3500 0    60   BiDi ~ 0
CODEC_I2C_SDA
Text GLabel 11850 3800 0    60   BiDi ~ 0
CODEC_I2S_CLK
Text GLabel 11850 4100 0    60   BiDi ~ 0
CODEC_I2S_DOUT
Text GLabel 11850 4400 0    60   BiDi ~ 0
CODEC_I2S_DIN
Text GLabel 11850 4700 0    60   BiDi ~ 0
CODEC_I2S_FSYNC
Wire Wire Line
	12550 3200 12350 3200
Wire Wire Line
	12550 3500 12350 3500
Wire Wire Line
	12550 3800 12350 3800
Wire Wire Line
	12550 4100 12350 4100
Wire Wire Line
	12550 4400 12350 4400
Wire Wire Line
	12550 4700 12350 4700
Wire Wire Line
	12050 3200 11850 3200
Wire Wire Line
	12050 3500 11850 3500
Wire Wire Line
	12050 3800 11850 3800
Wire Wire Line
	12050 4100 11850 4100
Wire Wire Line
	12050 4400 11850 4400
Wire Wire Line
	12050 4700 11850 4700
$EndSCHEMATC
