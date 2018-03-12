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
Sheet 1 1
Title "Wi-fi"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CC3220SF U?
U 1 1 5AA449F6
P 7350 5700
F 0 "U?" H 6550 8500 60  0000 C CNN
F 1 "CC3220SF" H 8300 3000 60  0000 R CNN
F 2 "" H 7350 5700 60  0001 C CNN
F 3 "" H 7350 5700 60  0001 C CNN
	1    7350 5700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5AA44C29
P 6000 1750
F 0 "R?" H 6050 1800 50  0000 L CNN
F 1 "R0R" H 6050 1700 50  0000 L CNN
F 2 "" V 5930 1750 50  0001 C CNN
F 3 "" H 6000 1750 50  0001 C CNN
F 4 "3990600" V 6000 1750 60  0001 C CNN "PN"
	1    6000 1750
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA44FCA
P 6200 2550
F 0 "C?" H 6300 2650 50  0000 L CNN
F 1 "0.1uF" H 6300 2550 50  0000 L CNN
F 2 "" H 6238 2400 50  0001 C CNN
F 3 "" H 6200 2550 50  0001 C CNN
F 4 "10V" H 6300 2450 50  0000 L CNN "Voltage"
F 5 "0402" H 6300 2350 50  0000 L CNN "Shape"
	1    6200 2550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA454C5
P 5600 2550
F 0 "C?" H 5700 2650 50  0000 L CNN
F 1 "0.1uF" H 5700 2550 50  0000 L CNN
F 2 "" H 5638 2400 50  0001 C CNN
F 3 "" H 5600 2550 50  0001 C CNN
F 4 "10V" H 5700 2450 50  0000 L CNN "Voltage"
F 5 "0402" H 5700 2350 50  0000 L CNN "Shape"
	1    5600 2550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA458D7
P 5000 2550
F 0 "C?" H 5100 2650 50  0000 L CNN
F 1 "4.7uF" H 5100 2550 50  0000 L CNN
F 2 "" H 5038 2400 50  0001 C CNN
F 3 "" H 5000 2550 50  0001 C CNN
F 4 "10V" H 5100 2450 50  0000 L CNN "Voltage"
F 5 "0603" H 5100 2350 50  0000 L CNN "Shape"
	1    5000 2550
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5AA45D12
P 4800 1750
F 0 "R?" H 4850 1800 50  0000 L CNN
F 1 "R0R" H 4850 1700 50  0000 L CNN
F 2 "" V 4730 1750 50  0001 C CNN
F 3 "" H 4800 1750 50  0001 C CNN
F 4 "3990600" V 4800 1750 60  0001 C CNN "PN"
	1    4800 1750
	1    0    0    -1  
$EndComp
Text GLabel 5100 1250 0    60   Input ~ 0
V_BAT
$Comp
L C C?
U 1 1 5AA547D9
P 4400 2550
F 0 "C?" H 4500 2650 50  0000 L CNN
F 1 "4.7uF" H 4500 2550 50  0000 L CNN
F 2 "" H 4438 2400 50  0001 C CNN
F 3 "" H 4400 2550 50  0001 C CNN
F 4 "10V" H 4500 2450 50  0000 L CNN "Voltage"
F 5 "0603" H 4500 2350 50  0000 L CNN "Shape"
	1    4400 2550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA5481A
P 3800 2550
F 0 "C?" H 3900 2650 50  0000 L CNN
F 1 "4.7uF" H 3900 2550 50  0000 L CNN
F 2 "" H 3838 2400 50  0001 C CNN
F 3 "" H 3800 2550 50  0001 C CNN
F 4 "10V" H 3900 2450 50  0000 L CNN "Voltage"
F 5 "0603" H 3900 2350 50  0000 L CNN "Shape"
	1    3800 2550
	1    0    0    -1  
$EndComp
$Comp
L L L?
U 1 1 5AA54852
P 2800 2000
F 0 "L?" V 2850 2050 50  0000 L CNN
F 1 "2.2uH" V 2850 2000 50  0000 R CNN
F 2 "" H 2800 2000 50  0001 C CNN
F 3 "" H 2800 2000 50  0001 C CNN
F 4 "LQM2HPN2R2MG0L" V 2800 2000 60  0001 C CNN "PN"
	1    2800 2000
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 5AA54E02
P 2100 2500
F 0 "C?" H 2200 2600 50  0000 L CNN
F 1 "10uF" H 2200 2500 50  0000 L CNN
F 2 "" H 2138 2350 50  0001 C CNN
F 3 "" H 2100 2500 50  0001 C CNN
F 4 "10V" H 2200 2400 50  0000 L CNN "Voltage"
F 5 "0603" H 2200 2300 50  0000 L CNN "Shape"
	1    2100 2500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA54FBA
P 2700 2500
F 0 "C?" H 2800 2600 50  0000 L CNN
F 1 "0.1uF" H 2800 2500 50  0000 L CNN
F 2 "" H 2738 2350 50  0001 C CNN
F 3 "" H 2700 2500 50  0001 C CNN
F 4 "10V" H 2800 2400 50  0000 L CNN "Voltage"
F 5 "0603" H 2800 2300 50  0000 L CNN "Shape"
	1    2700 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA55210
P 2100 2750
F 0 "#PWR?" H 2100 2500 50  0001 C CNN
F 1 "GND" H 2100 2600 50  0000 C CNN
F 2 "" H 2100 2750 50  0001 C CNN
F 3 "" H 2100 2750 50  0001 C CNN
	1    2100 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA5523A
P 2700 2750
F 0 "#PWR?" H 2700 2500 50  0001 C CNN
F 1 "GND" H 2700 2600 50  0000 C CNN
F 2 "" H 2700 2750 50  0001 C CNN
F 3 "" H 2700 2750 50  0001 C CNN
	1    2700 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA552EA
P 3800 2250
F 0 "#PWR?" H 3800 2000 50  0001 C CNN
F 1 "GND" H 3800 2100 50  0000 C CNN
F 2 "" H 3800 2250 50  0001 C CNN
F 3 "" H 3800 2250 50  0001 C CNN
	1    3800 2250
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA55357
P 4400 2250
F 0 "#PWR?" H 4400 2000 50  0001 C CNN
F 1 "GND" H 4400 2100 50  0000 C CNN
F 2 "" H 4400 2250 50  0001 C CNN
F 3 "" H 4400 2250 50  0001 C CNN
	1    4400 2250
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA55383
P 5000 2250
F 0 "#PWR?" H 5000 2000 50  0001 C CNN
F 1 "GND" H 5000 2100 50  0000 C CNN
F 2 "" H 5000 2250 50  0001 C CNN
F 3 "" H 5000 2250 50  0001 C CNN
	1    5000 2250
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA553AF
P 5600 2300
F 0 "#PWR?" H 5600 2050 50  0001 C CNN
F 1 "GND" H 5600 2150 50  0000 C CNN
F 2 "" H 5600 2300 50  0001 C CNN
F 3 "" H 5600 2300 50  0001 C CNN
	1    5600 2300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA553DB
P 6200 2300
F 0 "#PWR?" H 6200 2050 50  0001 C CNN
F 1 "GND" H 6200 2150 50  0000 C CNN
F 2 "" H 6200 2300 50  0001 C CNN
F 3 "" H 6200 2300 50  0001 C CNN
	1    6200 2300
	-1   0    0    1   
$EndComp
$Comp
L C C?
U 1 1 5AA55626
P 2700 3250
F 0 "C?" H 2800 3350 50  0000 L CNN
F 1 "0.1uF" H 2800 3250 50  0000 L CNN
F 2 "" H 2738 3100 50  0001 C CNN
F 3 "" H 2700 3250 50  0001 C CNN
F 4 "10V" H 2800 3150 50  0000 L CNN "Voltage"
F 5 "0603" H 2800 3050 50  0000 L CNN "Shape"
	1    2700 3250
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA55680
P 2100 3200
F 0 "C?" H 2200 3300 50  0000 L CNN
F 1 "0.1uF" H 2200 3200 50  0000 L CNN
F 2 "" H 2138 3050 50  0001 C CNN
F 3 "" H 2100 3200 50  0001 C CNN
F 4 "10V" H 2200 3100 50  0000 L CNN "Voltage"
F 5 "0603" H 2200 3000 50  0000 L CNN "Shape"
	1    2100 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA557B7
P 2100 3450
F 0 "#PWR?" H 2100 3200 50  0001 C CNN
F 1 "GND" H 2100 3300 50  0000 C CNN
F 2 "" H 2100 3450 50  0001 C CNN
F 3 "" H 2100 3450 50  0001 C CNN
	1    2100 3450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA557E9
P 2700 3450
F 0 "#PWR?" H 2700 3200 50  0001 C CNN
F 1 "GND" H 2700 3300 50  0000 C CNN
F 2 "" H 2700 3450 50  0001 C CNN
F 3 "" H 2700 3450 50  0001 C CNN
	1    2700 3450
	1    0    0    -1  
$EndComp
$Comp
L L L?
U 1 1 5AA5670B
P 2650 3800
F 0 "L?" V 2700 3850 50  0000 L CNN
F 1 "1uH" V 2700 3800 50  0000 R CNN
F 2 "" H 2650 3800 50  0001 C CNN
F 3 "" H 2650 3800 50  0001 C CNN
F 4 "LQM2HPN1R0MG0L" V 2650 3800 60  0001 C CNN "PN"
	1    2650 3800
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 5AA56BDE
P 2100 4500
F 0 "C?" H 2200 4600 50  0000 L CNN
F 1 "22uF" H 2200 4500 50  0000 L CNN
F 2 "" H 2138 4350 50  0001 C CNN
F 3 "" H 2100 4500 50  0001 C CNN
F 4 "10V" H 2200 4400 50  0000 L CNN "Voltage"
F 5 "0603" H 2200 4300 50  0000 L CNN "Shape"
	1    2100 4500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA56CD4
P 2500 4500
F 0 "C?" H 2600 4600 50  0000 L CNN
F 1 "22uF" H 2600 4500 50  0000 L CNN
F 2 "" H 2538 4350 50  0001 C CNN
F 3 "" H 2500 4500 50  0001 C CNN
F 4 "10V" H 2600 4400 50  0000 L CNN "Voltage"
F 5 "0603" H 2600 4300 50  0000 L CNN "Shape"
	1    2500 4500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA56D38
P 2900 4500
F 0 "C?" H 3000 4600 50  0000 L CNN
F 1 "0.1uF" H 3000 4500 50  0000 L CNN
F 2 "" H 2938 4350 50  0001 C CNN
F 3 "" H 2900 4500 50  0001 C CNN
F 4 "10V" H 3000 4400 50  0000 L CNN "Voltage"
F 5 "0603" H 3000 4300 50  0000 L CNN "Shape"
	1    2900 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA56E8D
P 2100 4750
F 0 "#PWR?" H 2100 4500 50  0001 C CNN
F 1 "GND" H 2100 4600 50  0000 C CNN
F 2 "" H 2100 4750 50  0001 C CNN
F 3 "" H 2100 4750 50  0001 C CNN
	1    2100 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA56EED
P 2500 4750
F 0 "#PWR?" H 2500 4500 50  0001 C CNN
F 1 "GND" H 2500 4600 50  0000 C CNN
F 2 "" H 2500 4750 50  0001 C CNN
F 3 "" H 2500 4750 50  0001 C CNN
	1    2500 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA56F3C
P 2900 4750
F 0 "#PWR?" H 2900 4500 50  0001 C CNN
F 1 "GND" H 2900 4600 50  0000 C CNN
F 2 "" H 2900 4750 50  0001 C CNN
F 3 "" H 2900 4750 50  0001 C CNN
	1    2900 4750
	1    0    0    -1  
$EndComp
$Comp
L L L?
U 1 1 5AA57CDF
P 2450 5550
F 0 "L?" V 2500 5600 50  0000 L CNN
F 1 "2.2uH" V 2500 5550 50  0000 R CNN
F 2 "" H 2450 5550 50  0001 C CNN
F 3 "" H 2450 5550 50  0001 C CNN
F 4 "LQM2HPN2R2MG0L" V 2450 5550 60  0001 C CNN "PN"
	1    2450 5550
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 5AA583B5
P 2100 6000
F 0 "C?" H 2200 6100 50  0000 L CNN
F 1 "10uF" H 2200 6000 50  0000 L CNN
F 2 "" H 2138 5850 50  0001 C CNN
F 3 "" H 2100 6000 50  0001 C CNN
F 4 "10V" H 2200 5900 50  0000 L CNN "Voltage"
F 5 "0603" H 2200 5800 50  0000 L CNN "Shape"
	1    2100 6000
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA58851
P 2600 6000
F 0 "C?" H 2700 6100 50  0000 L CNN
F 1 "0.1uF" H 2700 6000 50  0000 L CNN
F 2 "" H 2638 5850 50  0001 C CNN
F 3 "" H 2600 6000 50  0001 C CNN
F 4 "10V" H 2700 5900 50  0000 L CNN "Voltage"
F 5 "0603" H 2700 5800 50  0000 L CNN "Shape"
	1    2600 6000
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA588CF
P 3100 6000
F 0 "C?" H 3200 6100 50  0000 L CNN
F 1 "0.1uF" H 3200 6000 50  0000 L CNN
F 2 "" H 3138 5850 50  0001 C CNN
F 3 "" H 3100 6000 50  0001 C CNN
F 4 "10V" H 3200 5900 50  0000 L CNN "Voltage"
F 5 "0603" H 3200 5800 50  0000 L CNN "Shape"
	1    3100 6000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA58D4F
P 2100 6250
F 0 "#PWR?" H 2100 6000 50  0001 C CNN
F 1 "GND" H 2100 6100 50  0000 C CNN
F 2 "" H 2100 6250 50  0001 C CNN
F 3 "" H 2100 6250 50  0001 C CNN
	1    2100 6250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA58D99
P 2600 6250
F 0 "#PWR?" H 2600 6000 50  0001 C CNN
F 1 "GND" H 2600 6100 50  0000 C CNN
F 2 "" H 2600 6250 50  0001 C CNN
F 3 "" H 2600 6250 50  0001 C CNN
	1    2600 6250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA58DE3
P 3100 6250
F 0 "#PWR?" H 3100 6000 50  0001 C CNN
F 1 "GND" H 3100 6100 50  0000 C CNN
F 2 "" H 3100 6250 50  0001 C CNN
F 3 "" H 3100 6250 50  0001 C CNN
	1    3100 6250
	1    0    0    -1  
$EndComp
$Comp
L L L?
U 1 1 5AA59A8A
P 2600 6700
F 0 "L?" V 2650 6750 50  0000 L CNN
F 1 "10uH" V 2650 6700 50  0000 R CNN
F 2 "" H 2600 6700 50  0001 C CNN
F 3 "" H 2600 6700 50  0001 C CNN
F 4 "CBC2518T100M" V 2600 6700 60  0001 C CNN "PN"
	1    2600 6700
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 5AA59FFB
P 2100 7150
F 0 "C?" H 2200 7250 50  0000 L CNN
F 1 "10uF" H 2200 7150 50  0000 L CNN
F 2 "" H 2138 7000 50  0001 C CNN
F 3 "" H 2100 7150 50  0001 C CNN
F 4 "10V" H 2200 7050 50  0000 L CNN "Voltage"
F 5 "0603" H 2200 6950 50  0000 L CNN "Shape"
	1    2100 7150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA5A08A
P 2600 7200
F 0 "C?" H 2700 7300 50  0000 L CNN
F 1 "0.1uF" H 2700 7200 50  0000 L CNN
F 2 "" H 2638 7050 50  0001 C CNN
F 3 "" H 2600 7200 50  0001 C CNN
F 4 "10V" H 2700 7100 50  0000 L CNN "Voltage"
F 5 "0603" H 2700 7000 50  0000 L CNN "Shape"
	1    2600 7200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA5A16E
P 3050 7250
F 0 "C?" H 3150 7350 50  0000 L CNN
F 1 "0.1uF" H 3150 7250 50  0000 L CNN
F 2 "" H 3088 7100 50  0001 C CNN
F 3 "" H 3050 7250 50  0001 C CNN
F 4 "10V" H 3150 7150 50  0000 L CNN "Voltage"
F 5 "0603" H 3150 7050 50  0000 L CNN "Shape"
	1    3050 7250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA5AA50
P 2100 7400
F 0 "#PWR?" H 2100 7150 50  0001 C CNN
F 1 "GND" H 2100 7250 50  0000 C CNN
F 2 "" H 2100 7400 50  0001 C CNN
F 3 "" H 2100 7400 50  0001 C CNN
	1    2100 7400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA5AAA6
P 2600 7450
F 0 "#PWR?" H 2600 7200 50  0001 C CNN
F 1 "GND" H 2600 7300 50  0000 C CNN
F 2 "" H 2600 7450 50  0001 C CNN
F 3 "" H 2600 7450 50  0001 C CNN
	1    2600 7450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA5AAFC
P 3050 7500
F 0 "#PWR?" H 3050 7250 50  0001 C CNN
F 1 "GND" H 3050 7350 50  0000 C CNN
F 2 "" H 3050 7500 50  0001 C CNN
F 3 "" H 3050 7500 50  0001 C CNN
	1    3050 7500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA5B170
P 10150 8200
F 0 "#PWR?" H 10150 7950 50  0001 C CNN
F 1 "GND" H 10150 8050 50  0000 C CNN
F 2 "" H 10150 8200 50  0001 C CNN
F 3 "" H 10150 8200 50  0001 C CNN
	1    10150 8200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA5B5BC
P 9700 7950
F 0 "C?" H 9800 8050 50  0000 L CNN
F 1 "0.1uF" H 9800 7950 50  0000 L CNN
F 2 "" H 9738 7800 50  0001 C CNN
F 3 "" H 9700 7950 50  0001 C CNN
F 4 "10V" H 9800 7850 50  0000 L CNN "Voltage"
F 5 "0603" H 9800 7750 50  0000 L CNN "Shape"
	1    9700 7950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5AA5B893
P 10600 7950
F 0 "C?" H 10700 8050 50  0000 L CNN
F 1 "0.1uF" H 10700 7950 50  0000 L CNN
F 2 "" H 10638 7800 50  0001 C CNN
F 3 "" H 10600 7950 50  0001 C CNN
F 4 "10V" H 10700 7850 50  0000 L CNN "Voltage"
F 5 "0603" H 10700 7750 50  0000 L CNN "Shape"
	1    10600 7950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA5BA3F
P 9700 8200
F 0 "#PWR?" H 9700 7950 50  0001 C CNN
F 1 "GND" H 9700 8050 50  0000 C CNN
F 2 "" H 9700 8200 50  0001 C CNN
F 3 "" H 9700 8200 50  0001 C CNN
	1    9700 8200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA5BA9E
P 10600 8200
F 0 "#PWR?" H 10600 7950 50  0001 C CNN
F 1 "GND" H 10600 8050 50  0000 C CNN
F 2 "" H 10600 8200 50  0001 C CNN
F 3 "" H 10600 8200 50  0001 C CNN
	1    10600 8200
	1    0    0    -1  
$EndComp
$Comp
L Crystal_GND24 Y?
U 1 1 5AA5BE95
P 10150 7650
F 0 "Y?" H 10275 7850 50  0000 L CNN
F 1 "40 MHz" H 10275 7775 50  0000 L CNN
F 2 "" H 10150 7650 50  0001 C CNN
F 3 "" H 10150 7650 50  0001 C CNN
	1    10150 7650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AA5C015
P 10150 7400
F 0 "#PWR?" H 10150 7150 50  0001 C CNN
F 1 "GND" H 10150 7250 50  0000 C CNN
F 2 "" H 10150 7400 50  0001 C CNN
F 3 "" H 10150 7400 50  0001 C CNN
	1    10150 7400
	-1   0    0    1   
$EndComp
Wire Wire Line
	6000 3050 6300 3050
Wire Wire Line
	6200 3050 6200 2700
Wire Wire Line
	6200 2400 6200 2300
Wire Wire Line
	5600 2400 5600 2300
Wire Wire Line
	5400 3250 6300 3250
Wire Wire Line
	5600 3250 5600 2700
Connection ~ 6200 3050
Wire Wire Line
	5400 2000 5400 3250
Connection ~ 5600 3250
Wire Wire Line
	4800 3450 6300 3450
Wire Wire Line
	5000 3450 5000 2700
Wire Wire Line
	4200 3650 6300 3650
Wire Wire Line
	4400 3650 4400 2700
Wire Wire Line
	4400 2400 4400 2250
Wire Wire Line
	5000 2400 5000 2250
Wire Wire Line
	4800 1900 4800 3450
Connection ~ 5000 3450
Wire Wire Line
	4200 3650 4200 2000
Connection ~ 4400 3650
Wire Wire Line
	6000 1900 6000 3050
Wire Wire Line
	5400 2000 6000 2000
Connection ~ 6000 2000
Wire Wire Line
	3650 2000 4800 2000
Connection ~ 4800 2000
Wire Wire Line
	3800 2700 3800 3850
Wire Wire Line
	3650 3850 6300 3850
Wire Wire Line
	3800 2250 3800 2400
Wire Wire Line
	3650 3850 3650 2000
Connection ~ 4200 2000
Wire Wire Line
	4800 1600 4800 1400
Wire Wire Line
	4800 1400 6000 1400
Wire Wire Line
	6000 1250 6000 1600
Wire Wire Line
	5100 1250 6000 1250
Connection ~ 6000 1400
Wire Wire Line
	2650 2000 2100 2000
Wire Wire Line
	2100 2000 2100 2350
Wire Wire Line
	2700 2350 2700 2150
Wire Wire Line
	2100 2150 3550 2150
Connection ~ 2100 2150
Connection ~ 2700 2150
Wire Wire Line
	2100 2750 2100 2650
Wire Wire Line
	2700 2750 2700 2650
Wire Wire Line
	2100 3000 3500 3000
Wire Wire Line
	2100 3000 2100 3050
Wire Wire Line
	2100 3450 2100 3350
Wire Wire Line
	2700 3450 2700 3400
Wire Wire Line
	3600 2000 3600 4050
Wire Wire Line
	3600 4050 6300 4050
Connection ~ 3800 3850
Wire Wire Line
	3550 2150 3550 4250
Wire Wire Line
	3550 4250 6300 4250
Wire Wire Line
	3500 3000 3500 4450
Wire Wire Line
	3500 4450 6300 4450
Wire Wire Line
	2700 3100 2700 3050
Wire Wire Line
	2700 3050 3450 3050
Wire Wire Line
	3450 4550 6300 4550
Wire Wire Line
	3200 2150 3200 3050
Connection ~ 3200 3000
Connection ~ 3200 3050
Connection ~ 3200 2150
Wire Wire Line
	3450 3050 3450 4550
Wire Wire Line
	2950 2000 3600 2000
Wire Wire Line
	2800 3800 3400 3800
Wire Wire Line
	3400 3800 3400 4750
Wire Wire Line
	3400 4750 6300 4750
Wire Wire Line
	3350 4950 6300 4950
Wire Wire Line
	3350 4950 3350 3900
Wire Wire Line
	3350 3900 2100 3900
Wire Wire Line
	2100 3900 2100 3800
Wire Wire Line
	2100 3800 2500 3800
Wire Wire Line
	2100 4350 2100 4150
Wire Wire Line
	2100 4150 3300 4150
Wire Wire Line
	2900 4150 2900 4350
Wire Wire Line
	2500 4350 2500 4150
Connection ~ 2500 4150
Wire Wire Line
	2100 4750 2100 4650
Wire Wire Line
	2500 4750 2500 4650
Wire Wire Line
	2900 4750 2900 4650
Wire Wire Line
	3300 4150 3300 5350
Wire Wire Line
	3300 5150 6300 5150
Connection ~ 2900 4150
Wire Wire Line
	3300 5350 6300 5350
Connection ~ 3300 5150
Wire Wire Line
	6300 5550 2600 5550
Wire Wire Line
	2300 5550 2100 5550
Wire Wire Line
	2100 5550 2100 5850
Wire Wire Line
	2100 5750 6300 5750
Connection ~ 2100 5750
Wire Wire Line
	2600 5850 2600 5750
Connection ~ 2600 5750
Wire Wire Line
	3100 5850 3100 5750
Connection ~ 3100 5750
Wire Wire Line
	2100 6250 2100 6150
Wire Wire Line
	2600 6250 2600 6150
Wire Wire Line
	3100 6250 3100 6150
Wire Wire Line
	6300 5850 3550 5850
Wire Wire Line
	3550 5850 3550 5750
Connection ~ 3550 5750
Wire Wire Line
	2750 6700 3600 6700
Wire Wire Line
	3600 6700 3600 6050
Wire Wire Line
	3600 6050 6300 6050
Wire Wire Line
	3650 6250 6300 6250
Wire Wire Line
	3650 6250 3650 6750
Wire Wire Line
	3650 6750 2100 6750
Wire Wire Line
	2100 6750 2100 6700
Wire Wire Line
	2100 6700 2450 6700
Wire Wire Line
	2100 7000 2100 6850
Wire Wire Line
	2100 6850 3700 6850
Wire Wire Line
	3700 6850 3700 6450
Wire Wire Line
	3700 6450 6300 6450
Wire Wire Line
	6300 6650 3750 6650
Wire Wire Line
	3750 6650 3750 6900
Wire Wire Line
	3750 6900 2600 6900
Wire Wire Line
	2600 6900 2600 7050
Wire Wire Line
	3050 7100 3050 6950
Wire Wire Line
	3050 6950 3800 6950
Wire Wire Line
	3800 6950 3800 6850
Wire Wire Line
	3800 6850 6300 6850
Wire Wire Line
	2100 7400 2100 7300
Wire Wire Line
	2600 7450 2600 7350
Wire Wire Line
	3050 7500 3050 7400
Wire Wire Line
	8500 7650 10000 7650
Wire Wire Line
	10300 7650 10600 7650
Wire Wire Line
	10600 7150 10600 7800
Wire Wire Line
	9700 7150 9700 7550
Wire Wire Line
	9700 7550 8500 7550
Wire Wire Line
	10600 8200 10600 8100
Wire Wire Line
	9700 8200 9700 8100
Wire Wire Line
	9700 7800 9700 7650
Connection ~ 9700 7650
Connection ~ 10600 7650
Wire Wire Line
	9700 7150 10600 7150
Wire Wire Line
	10150 7400 10150 7450
Wire Wire Line
	10150 8200 10150 7850
$Comp
L Band_pass_filter FL?
U 1 1 5AA5CA66
P 9700 3400
F 0 "FL?" H 9450 3650 60  0000 L CNN
F 1 "Band_pass_filter" H 9800 3200 60  0000 C CNN
F 2 "" H 9650 3700 60  0001 C CNN
F 3 "" H 9650 3700 60  0001 C CNN
F 4 "DEA202450BT-1294C1-H" H 9700 3400 60  0001 C CNN "PN"
	1    9700 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3250 9250 3250
$Comp
L GND #PWR?
U 1 1 5AA5D082
P 10300 3600
F 0 "#PWR?" H 10300 3350 50  0001 C CNN
F 1 "GND" H 10300 3450 50  0000 C CNN
F 2 "" H 10300 3600 50  0001 C CNN
F 3 "" H 10300 3600 50  0001 C CNN
	1    10300 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 3500 10300 3500
Wire Wire Line
	10300 3400 10300 3600
Wire Wire Line
	10200 3400 10300 3400
Connection ~ 10300 3500
$Comp
L L L?
U 1 1 5AA5DC29
P 10950 3250
F 0 "L?" V 11000 3300 50  0000 L CNN
F 1 "3.3uH" V 11000 3250 50  0000 R CNN
F 2 "" H 10950 3250 50  0001 C CNN
F 3 "" H 10950 3250 50  0001 C CNN
F 4 "LQG15HS3N3S02D" V 10950 3250 60  0001 C CNN "PN"
	1    10950 3250
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 5AA5E317
P 11300 3450
F 0 "C?" H 11400 3550 50  0000 L CNN
F 1 "0.5pF" H 11400 3450 50  0000 L CNN
F 2 "" H 11338 3300 50  0001 C CNN
F 3 "" H 11300 3450 50  0001 C CNN
F 4 "10V" H 11400 3350 50  0000 L CNN "Voltage"
F 5 "0603" H 11400 3250 50  0000 L CNN "Shape"
	1    11300 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 3250 10800 3250
Wire Wire Line
	11100 3250 11300 3250
Wire Wire Line
	11300 3250 11300 3300
$Comp
L GND #PWR?
U 1 1 5AA5E7C3
P 11300 3700
F 0 "#PWR?" H 11300 3450 50  0001 C CNN
F 1 "GND" H 11300 3550 50  0000 C CNN
F 2 "" H 11300 3700 50  0001 C CNN
F 3 "" H 11300 3700 50  0001 C CNN
	1    11300 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	11300 3700 11300 3600
$EndSCHEMATC