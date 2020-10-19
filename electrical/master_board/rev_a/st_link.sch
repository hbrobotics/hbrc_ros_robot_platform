EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 10
Title "HR2 ST-Link"
Date "2020-10-17"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 6500 3650 2    50   Input ~ 0
3.3V
Text HLabel 6500 3750 2    50   Output ~ 0
SWCLK
Text HLabel 6500 3850 2    50   Input ~ 0
GND
Text HLabel 6500 3950 2    50   BiDi ~ 0
SWDIO
Text HLabel 6500 4050 2    50   Output ~ 0
~NRST
Text HLabel 6500 4150 2    50   UnSpc ~ 0
SWO
Text HLabel 6500 4250 2    50   Input ~ 0
STL_RX
Text HLabel 6500 4350 2    50   Output ~ 0
STL_TX
$Comp
L HR2:STADAPTER;F2x4 CN58
U 1 1 5F4B5155
P 5200 3650
F 0 "CN58" H 5600 3800 50  0000 C CNN
F 1 "STADAPTER;F2x4" H 5600 2800 50  0000 C CNN
F 2 "HR2:STADAPTER_F2x4" H 5400 3700 60  0001 L CNN
F 3 "ST Adapter Mating Connector" H 5400 3500 60  0001 L CNN
F 4 "ST Adapter Mating Connector" H 5400 3600 60  0001 L CNN "manf#"
	1    5200 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3650 6500 3650
Wire Wire Line
	6500 3750 6000 3750
Wire Wire Line
	6000 3850 6500 3850
Wire Wire Line
	6500 3950 6000 3950
Wire Wire Line
	6000 4050 6500 4050
Wire Wire Line
	6500 4150 6000 4150
Wire Wire Line
	6000 4250 6500 4250
Wire Wire Line
	6000 4350 6500 4350
$EndSCHEMATC
