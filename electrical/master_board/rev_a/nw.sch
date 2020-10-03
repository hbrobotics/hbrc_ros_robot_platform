EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 12 12
Title "HR2 NE PCB"
Date "2020-06-13"
Rev "A"
Comp "Homebrew Robotics Club"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L HR2:HOLE;M2.5 H?
U 1 1 5F2F7B9C
P 1500 5550
F 0 "H?" H 1450 5600 60  0000 L CNN
F 1 "HOLE;M2.5" H 1300 5500 60  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 1700 5600 60  0001 L CNN
F 3 "M2.5 Mounting Hole" H 1700 5400 60  0001 L CNN
	1    1500 5550
	1    0    0    -1  
$EndComp
$Comp
L HR2:HOLE;M2.5 H?
U 1 1 5F2F8125
P 1500 5300
F 0 "H?" H 1450 5350 60  0000 L CNN
F 1 "HOLE;M2.5" H 1300 5250 60  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 1700 5350 60  0001 L CNN
F 3 "M2.5 Mounting Hole" H 1700 5150 60  0001 L CNN
	1    1500 5300
	1    0    0    -1  
$EndComp
$Comp
L HR2:GROVE;20x20 GV?
U 1 1 5F2F999C
P 1500 5050
F 0 "GV?" H 1450 5100 60  0000 L CNN
F 1 "GROVE;20x20" H 1200 5000 60  0000 L CNN
F 2 "HR2:GROVE20x20" H 1700 5100 60  0001 L CNN
F 3 "Grove 20x20 Module" H 1700 4900 60  0001 L CNN
F 4 "Grove 20x20 Module" H 1700 5000 60  0001 L CNN "manf#"
	1    1500 5050
	1    0    0    -1  
$EndComp
$Comp
L HR2:GROVE;20x20R GV?
U 1 1 5F303BB1
P 1500 5800
F 0 "GV?" H 1400 5850 60  0000 L CNN
F 1 "GROVE;20x20R" H 1150 5750 60  0000 L CNN
F 2 "HR2:GROVE20x20R" H 1700 5850 60  0001 L CNN
F 3 "Grove 20x20 Module" H 1700 5650 60  0001 L CNN
F 4 "Grove 20x20 Module" H 1700 5750 60  0001 L CNN "manf#"
	1    1500 5800
	1    0    0    -1  
$EndComp
Text Notes 900  6100 0    50   ~ 0
Note: GV7 is the other half of GV3 on center board
$Comp
L Device:LED D?
U 1 1 5F30BC5B
P 5000 4800
F 0 "D?" V 5100 4900 50  0000 R CNN
F 1 "LED;GRNRA" V 4900 5250 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 5000 4800 50  0001 C CNN
F 3 "~" H 5000 4800 50  0001 C CNN
	1    5000 4800
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D?
U 1 1 5F30DD43
P 5900 4800
F 0 "D?" V 6000 4900 50  0000 R CNN
F 1 "LED;GRNRA" V 5800 5250 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 5900 4800 50  0001 C CNN
F 3 "~" H 5900 4800 50  0001 C CNN
	1    5900 4800
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D?
U 1 1 5F30E439
P 6800 4800
F 0 "D?" V 6900 4900 50  0000 R CNN
F 1 "LED;GRNRA" V 6700 5250 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 6800 4800 50  0001 C CNN
F 3 "~" H 6800 4800 50  0001 C CNN
	1    6800 4800
	0    -1   -1   0   
$EndComp
$Comp
L HR2:BRIDGE_NW_OUTER;M1x2+M2x5 CN?
U 1 1 5F3877C6
P 1300 4300
F 0 "CN?" H 1650 4450 50  0000 C CNN
F 1 "BRIDGE_NW_OUTER;M1x2+M2x5" H 1650 4050 50  0000 C CNN
F 2 "HR2:BRIDGE_NW_OUTER_M1x2+M2x5" H 1500 4350 60  0001 L CNN
F 3 "" H 1500 4150 60  0001 L CNN
F 4 "NW Outer Bridge Connectors" H 1500 4050 60  0001 L CNN "Field5"
	1    1300 4300
	1    0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NW_OUTER;M1x2+M2x5 CN?
U 2 1 5F389BE1
P 1300 2850
F 0 "CN?" H 1650 3000 50  0000 C CNN
F 1 "BRIDGE_NW_OUTER;M1x2+M2x5" H 1650 1800 50  0000 C CNN
F 2 "HR2:BRIDGE_NW_OUTER_M1x2+M2x5" H 1500 2900 60  0001 L CNN
F 3 "" H 1500 2700 60  0001 L CNN
F 4 "NW Outer Bridge Connectors" H 1500 2600 60  0001 L CNN "Field5"
	2    1300 2850
	1    0    0    -1  
$EndComp
Text Label 2100 3750 0    50   ~ 0
GND
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F433B12
P 4900 4400
F 0 "Q?" H 4750 4500 50  0000 L CNN
F 1 "2N7000;SOT23" H 4300 4350 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5100 4325 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 4900 4400 50  0001 L CNN
	1    4900 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F434F01
P 4750 3500
F 0 "R?" V 4650 3450 50  0000 C CNN
F 1 "330Ω;1608" V 4850 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4790 3490 50  0001 C CNN
F 3 "~" H 4750 3500 50  0001 C CNN
	1    4750 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F437EB2
P 4750 3200
F 0 "R?" V 4650 3150 50  0000 C CNN
F 1 "330Ω;1608" V 4850 3200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4790 3190 50  0001 C CNN
F 3 "~" H 4750 3200 50  0001 C CNN
	1    4750 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F43808D
P 4750 2900
F 0 "R?" V 4650 2850 50  0000 C CNN
F 1 "330Ω;1608" V 4850 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4790 2890 50  0001 C CNN
F 3 "~" H 4750 2900 50  0001 C CNN
	1    4750 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	4600 2900 4500 2900
Wire Wire Line
	4500 2900 4500 3200
Wire Wire Line
	4500 3200 4600 3200
Wire Wire Line
	4500 3200 4500 3500
Wire Wire Line
	4500 3500 4600 3500
Connection ~ 4500 3200
Wire Wire Line
	4900 2900 5000 2900
Wire Wire Line
	5000 2900 5000 3200
Wire Wire Line
	4900 3500 5000 3500
Connection ~ 5000 3500
Wire Wire Line
	5000 3500 5000 4200
Wire Wire Line
	4900 3200 5000 3200
Connection ~ 5000 3200
Wire Wire Line
	5000 3200 5000 3500
Wire Wire Line
	5000 4650 5000 4600
Wire Wire Line
	4500 2900 4500 2600
Connection ~ 4500 2900
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F43B291
P 5800 4400
F 0 "Q?" H 5650 4500 50  0000 L CNN
F 1 "2N7000;SOT23" H 5200 4350 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6000 4325 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 5800 4400 50  0001 L CNN
	1    5800 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F43B29B
P 5650 3500
F 0 "R?" V 5550 3450 50  0000 C CNN
F 1 "330Ω;1608" V 5750 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5690 3490 50  0001 C CNN
F 3 "~" H 5650 3500 50  0001 C CNN
	1    5650 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F43B2A5
P 5650 3200
F 0 "R?" V 5550 3150 50  0000 C CNN
F 1 "330Ω;1608" V 5750 3200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5690 3190 50  0001 C CNN
F 3 "~" H 5650 3200 50  0001 C CNN
	1    5650 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F43B2AF
P 5650 2900
F 0 "R?" V 5550 2850 50  0000 C CNN
F 1 "330Ω;1608" V 5750 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5690 2890 50  0001 C CNN
F 3 "~" H 5650 2900 50  0001 C CNN
	1    5650 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	5500 2900 5400 2900
Wire Wire Line
	5400 2900 5400 3200
Wire Wire Line
	5400 3200 5500 3200
Wire Wire Line
	5400 3200 5400 3500
Wire Wire Line
	5400 3500 5500 3500
Connection ~ 5400 3200
Wire Wire Line
	5800 2900 5900 2900
Wire Wire Line
	5900 2900 5900 3200
Wire Wire Line
	5800 3500 5900 3500
Connection ~ 5900 3500
Wire Wire Line
	5900 3500 5900 4200
Wire Wire Line
	5800 3200 5900 3200
Connection ~ 5900 3200
Wire Wire Line
	5900 3200 5900 3500
Wire Wire Line
	5900 4650 5900 4600
Wire Wire Line
	5400 2900 5400 2600
Connection ~ 5400 2900
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F4407CF
P 6700 4400
F 0 "Q?" H 6550 4500 50  0000 L CNN
F 1 "2N7000;SOT23" H 6100 4350 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6900 4325 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 6700 4400 50  0001 L CNN
	1    6700 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F4407D9
P 6550 3500
F 0 "R?" V 6450 3450 50  0000 C CNN
F 1 "330Ω;1608" V 6650 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6590 3490 50  0001 C CNN
F 3 "~" H 6550 3500 50  0001 C CNN
	1    6550 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F4407E3
P 6550 3200
F 0 "R?" V 6450 3150 50  0000 C CNN
F 1 "330Ω;1608" V 6650 3200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6590 3190 50  0001 C CNN
F 3 "~" H 6550 3200 50  0001 C CNN
	1    6550 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F4407ED
P 6550 2900
F 0 "R?" V 6450 2850 50  0000 C CNN
F 1 "330Ω;1608" V 6650 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6590 2890 50  0001 C CNN
F 3 "~" H 6550 2900 50  0001 C CNN
	1    6550 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	6400 2900 6300 2900
Wire Wire Line
	6300 2900 6300 3200
Wire Wire Line
	6300 3200 6400 3200
Wire Wire Line
	6300 3200 6300 3500
Wire Wire Line
	6300 3500 6400 3500
Connection ~ 6300 3200
Wire Wire Line
	6700 2900 6800 2900
Wire Wire Line
	6800 2900 6800 3200
Wire Wire Line
	6700 3500 6800 3500
Connection ~ 6800 3500
Wire Wire Line
	6800 3500 6800 4200
Wire Wire Line
	6700 3200 6800 3200
Connection ~ 6800 3200
Wire Wire Line
	6800 3200 6800 3500
Wire Wire Line
	6800 4650 6800 4600
Wire Wire Line
	6300 2900 6300 2600
Connection ~ 6300 2900
Wire Wire Line
	4500 2600 5400 2600
Connection ~ 5400 2600
Wire Wire Line
	5400 2600 6300 2600
Wire Wire Line
	5000 4950 5000 5050
Wire Wire Line
	5000 5050 5900 5050
Wire Wire Line
	5900 5050 5900 4950
Wire Wire Line
	5900 5050 6800 5050
Wire Wire Line
	6800 5050 6800 4950
Connection ~ 5900 5050
Wire Wire Line
	6300 3800 6300 4400
Wire Wire Line
	6300 4400 6500 4400
Wire Wire Line
	5350 4400 5600 4400
Wire Wire Line
	2500 5050 5000 5050
Connection ~ 5000 5050
Text Label 2100 2850 0    50   ~ 0
LED5
Wire Wire Line
	5350 4100 5350 4400
Text Label 2100 2950 0    50   ~ 0
LED4
Text Label 2100 3050 0    50   ~ 0
LED6
Text Label 2100 3350 0    50   ~ 0
TRIG
Text Label 2100 3150 0    50   ~ 0
5V
Text Label 2100 3550 0    50   ~ 0
ECHO
NoConn ~ 6300 6300
Wire Wire Line
	2500 3750 2500 5050
$Comp
L HR2:HCSR04H;F1X4 CN?
U 1 1 5F0B8A7B
P 3300 3450
F 0 "CN?" H 3550 3600 50  0000 C CNN
F 1 "HCSR04H;F1X4" H 3650 3000 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4H" H 3500 3500 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 3500 3300 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 3500 3400 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (High Profile Receptacle)" H 3500 3200 60  0001 L CNN "Field5"
	1    3300 3450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2600 3750 2500 3750
Connection ~ 4500 2600
Text Label 2100 4400 0    50   ~ 0
9V
Wire Wire Line
	2000 4400 3300 4400
Wire Wire Line
	3300 4400 3300 2600
Wire Wire Line
	3300 2600 4500 2600
Wire Wire Line
	2000 3750 2500 3750
Connection ~ 2500 3750
Wire Wire Line
	2000 3550 2300 3550
Wire Wire Line
	2300 3550 2300 3650
Wire Wire Line
	2300 3650 2600 3650
Wire Wire Line
	2000 3350 2400 3350
Wire Wire Line
	2400 3350 2400 3550
Wire Wire Line
	2400 3550 2600 3550
Wire Wire Line
	2600 3450 2500 3450
Wire Wire Line
	2500 3450 2500 3150
Wire Wire Line
	2500 3150 2000 3150
Wire Wire Line
	2000 2950 3600 2950
Wire Wire Line
	3600 2950 3600 4400
Wire Wire Line
	3600 4400 4700 4400
Wire Wire Line
	2000 2850 3700 2850
Wire Wire Line
	3700 2850 3700 4100
Wire Wire Line
	3700 4100 5350 4100
Wire Wire Line
	2000 3050 3800 3050
Wire Wire Line
	3800 3050 3800 3800
Wire Wire Line
	3800 3800 6300 3800
NoConn ~ 2000 3250
NoConn ~ 2000 3450
NoConn ~ 2000 3650
NoConn ~ 2000 4300
$EndSCHEMATC
