EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "Sonar Adapter"
Date "2021-03-13"
Rev "A"
Comp "Homebrew Robototics Club"
Comment1 "MIT License"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	5000 3500 5500 3500
Wire Wire Line
	5500 3600 5000 3600
Wire Wire Line
	5000 3700 5500 3700
Wire Wire Line
	5500 3800 5000 3800
$Comp
L HR2:SONAR;M1x4RA CN1
U 1 1 604D3250
P 4300 3500
F 0 "CN1" H 4600 3650 50  0000 C CNN
F 1 "SONAR;M1x4RA" H 4650 3050 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 4500 3550 60  0001 L CNN
F 3 "HR2 Sonar In Adapter" H 4500 3350 60  0001 L CNN
F 4 "HR2 Sonar In Adapter" H 4500 3250 60  0001 L CNN "desc"
	1    4300 3500
	1    0    0    -1  
$EndComp
$Comp
L HR2:SONAR;F1x4RA CN2
U 1 1 604D37B7
P 6200 3800
F 0 "CN2" H 6700 3350 50  0000 R CNN
F 1 "SONAR;F1x4RA" H 6800 3950 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 6400 3850 60  0001 L CNN
F 3 "HR2 Sonar Out Adapter" H 6400 3650 60  0001 L CNN
F 4 "HR2 Sonar Out Adapter" H 6400 3550 60  0001 L CNN "desc"
	1    6200 3800
	-1   0    0    1   
$EndComp
Text Label 5150 3500 0    50   ~ 0
GND
Text Label 5150 3600 0    50   ~ 0
ECHO
Text Label 5150 3700 0    50   ~ 0
TRIG
Text Label 5150 3800 0    50   ~ 0
VCC
$EndSCHEMATC
