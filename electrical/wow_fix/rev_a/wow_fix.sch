EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "WOW Fix"
Date "2021-02-14"
Rev "A"
Comp "Homebrew Robotics Club"
Comment1 "Copyright © 2021 by Hombrew Robotics Club"
Comment2 "License: MIT"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L HR2:WOW_IN;M2x6S CN1
U 1 1 602A126D
P 5000 3400
F 0 "CN1" H 5600 3550 50  0000 C CNN
F 1 "WOW_IN;M2x6S" H 5450 2150 50  0000 C CNN
F 2 "HR2:PinHeader_2x06_P2.54mm_Vertical_Shrouded" H 5200 3450 60  0001 L CNN
F 3 "WOW Input Connector" H 5200 3250 60  0001 L CNN
F 4 "WOW Input Connector" H 5200 3150 60  0001 L CNN "desc"
	1    5000 3400
	-1   0    0    -1  
$EndComp
$Comp
L HR2:WOW_OUT;M2x6S CN2
U 1 1 602A2D90
P 6500 3400
F 0 "CN2" H 6700 3550 50  0000 L CNN
F 1 "WOW_OUT;M2x6S" H 6650 2150 50  0000 L CNN
F 2 "HR2:PinHeader_2x06_P2.54mm_Vertical_Shrouded" H 6700 3450 60  0001 L CNN
F 3 "WOW Output Connector" H 6700 3250 60  0001 L CNN
F 4 "WOW Output Connector" H 6700 3150 60  0001 L CNN "desc"
	1    6500 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 3400 5100 3400
Wire Wire Line
	5100 3400 5100 3500
Wire Wire Line
	5100 3500 5000 3500
Wire Wire Line
	5000 3600 5100 3600
Wire Wire Line
	5100 3600 5100 3700
Wire Wire Line
	5100 3700 5000 3700
Wire Wire Line
	5000 4400 5100 4400
Wire Wire Line
	5100 4400 5100 4500
Wire Wire Line
	5100 4500 5000 4500
Wire Wire Line
	5000 4200 5100 4200
Wire Wire Line
	5100 4200 5100 4300
Wire Wire Line
	5100 4300 5000 4300
Wire Wire Line
	6500 3400 6400 3400
Wire Wire Line
	6400 3400 6400 3500
Wire Wire Line
	6400 3500 6500 3500
Wire Wire Line
	6500 3600 6400 3600
Wire Wire Line
	6400 3600 6400 3700
Wire Wire Line
	6400 3700 6500 3700
Wire Wire Line
	6500 4400 6400 4400
Wire Wire Line
	6400 4400 6400 4500
Wire Wire Line
	6400 4500 6500 4500
Wire Wire Line
	6500 4200 6400 4200
Wire Wire Line
	6400 4200 6400 4300
Wire Wire Line
	6400 4300 6500 4300
Wire Wire Line
	5100 3500 6400 3500
Connection ~ 5100 3500
Connection ~ 6400 3500
Wire Wire Line
	5100 3600 6400 3600
Connection ~ 5100 3600
Connection ~ 6400 3600
Wire Wire Line
	5100 4300 6400 4300
Connection ~ 5100 4300
Connection ~ 6400 4300
Wire Wire Line
	5100 4400 6400 4400
Connection ~ 5100 4400
Connection ~ 6400 4400
Wire Wire Line
	5000 4100 6500 4100
Wire Wire Line
	5000 4000 6500 4000
Wire Wire Line
	5000 3900 6500 3900
Wire Wire Line
	5000 3800 6500 3800
Text Label 5500 3500 0    50   ~ 0
POW_IN12
Text Label 5500 4400 0    50   ~ 0
POW_IN34
Text Label 5500 3600 0    50   ~ 0
GND12
Text Label 5500 4300 0    50   ~ 0
GND34
Text Label 5500 3800 0    50   ~ 0
COML
Text Label 5500 3900 0    50   ~ 0
COMH
Text Label 5500 4000 0    50   ~ 0
STOPL
Text Label 5500 4100 0    50   ~ 0
STOPH
$EndSCHEMATC
