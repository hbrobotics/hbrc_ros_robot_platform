EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 12 11
Title "HR2 Master Board"
Date "2020-10-03"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L HR2:HCSR04H;F1X4 CN8
U 1 1 5EE6BF38
P 5500 1600
F 0 "CN8" H 5900 1750 50  0000 C CNN
F 1 "HCSR04;F1x4H" H 5850 1150 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4H" H 5700 1650 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5700 1450 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5700 1550 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (High Profile Receptacle)" H 5700 1350 60  0001 L CNN "Field5"
	1    5500 1600
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HOLE;M2.5 H2
U 1 1 5F2F7E7C
P 7300 5100
F 0 "H2" H 7200 5150 60  0000 L CNN
F 1 "HOLE;M2.5" H 7050 5050 60  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 7500 5150 60  0001 L CNN
F 3 "M2.5 Mounting Hole" H 7500 4950 60  0001 L CNN
	1    7300 5100
	1    0    0    -1  
$EndComp
$Comp
L HR2:HOLE;M2.5 H1
U 1 1 5F2F85AA
P 7300 4800
F 0 "H1" H 7200 4850 60  0000 L CNN
F 1 "HOLE;M2.5" H 7050 4750 60  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 7500 4850 60  0001 L CNN
F 3 "M2.5 Mounting Hole" H 7500 4650 60  0001 L CNN
	1    7300 4800
	1    0    0    -1  
$EndComp
$Comp
L HR2:GROVE;20x20 GV1
U 1 1 5F2F9731
P 7300 4500
F 0 "GV1" H 7200 4550 60  0000 L CNN
F 1 "GROVE;20x20" H 7000 4450 60  0000 L CNN
F 2 "HR2:GROVE20x20" H 7500 4550 60  0001 L CNN
F 3 "Grove 20x20 Module" H 7500 4350 60  0001 L CNN
F 4 "Grove 20x20 Module" H 7500 4450 60  0001 L CNN "manf#"
	1    7300 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5F30B259
P 6150 2300
F 0 "D2" V 6250 2400 50  0000 R CNN
F 1 "LED;GRNRA" V 6050 2750 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 6150 2300 50  0001 C CNN
F 3 "~" H 6150 2300 50  0001 C CNN
	1    6150 2300
	0    -1   -1   0   
$EndComp
Text Label 4100 3450 0    50   ~ 0
GND
$Comp
L Transistor_FET:2N7000 Q4
U 1 1 5F4294E7
P 6050 3150
F 0 "Q4" H 5950 3300 50  0000 L CNN
F 1 "2N7000;SOT23" H 5450 3100 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6250 3075 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 6050 3150 50  0001 L CNN
	1    6050 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R43
U 1 1 5F429CFC
P 7500 1950
F 0 "R43" V 7400 1850 50  0000 L CNN
F 1 "330Ω;1608" V 7600 1750 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7540 1940 50  0001 C CNN
F 3 "~" H 7500 1950 50  0001 C CNN
	1    7500 1950
	0    1    1    0   
$EndComp
Text Notes 3500 5800 0    50   ~ 0
Notes:\n1. GRNRA = Green Right Angle Green\n2. Nominal Part:\n     Mfg Part #: TLPG5600\n     Digi-Key part #: TLPG5600-ND\n     Forward Voltage Drop: 2.4V\n     Absolute max. current: 200mA\n     Target maximum current: I=60mA\n     Resistor: R = (9V - 2.4V) / 75mA = 6.6V / .060A = 110Ω\n     Watts: W = I**2*R = (.060 * .060) * 110 = .396W ~=.4W\n     Use 3 x 330Ω in parallel to get 110Ω\n     Watts per resistor: Wr = .4W / 3 = .133W ~= .125W\n     1/8W or larger should work.\n3. LED's are labeled from 1 to 16 in a counter-clockwise\n   direction starting in the NW quadrant nearest the X axis.
$Comp
L Device:R_US R42
U 1 1 5F42F445
P 7500 1650
F 0 "R42" V 7400 1550 50  0000 L CNN
F 1 "330Ω;1608" V 7600 1450 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7540 1640 50  0001 C CNN
F 3 "~" H 7500 1650 50  0001 C CNN
	1    7500 1650
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R41
U 1 1 5F42F75B
P 7500 1350
F 0 "R41" V 7400 1250 50  0000 L CNN
F 1 "330Ω;1608" V 7600 1150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7540 1340 50  0001 C CNN
F 3 "~" H 7500 1350 50  0001 C CNN
	1    7500 1350
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 1650 5650 1650
Wire Wire Line
	5650 1650 5650 1950
Wire Wire Line
	5750 1950 5650 1950
Wire Wire Line
	6050 1950 6150 1950
Wire Wire Line
	6150 1950 6150 1650
Wire Wire Line
	6150 1650 6050 1650
Wire Wire Line
	6150 1650 6150 1350
Wire Wire Line
	6150 1350 6050 1350
Connection ~ 6150 1650
Wire Wire Line
	5650 1650 5650 1350
Wire Wire Line
	5650 1350 5750 1350
Connection ~ 5650 1650
Wire Wire Line
	6150 2150 6150 1950
Connection ~ 6150 1950
Wire Wire Line
	6150 2950 6150 2450
$Comp
L Device:LED D3
U 1 1 5F436F00
P 6950 2300
F 0 "D3" V 7050 2400 50  0000 R CNN
F 1 "LED;GRNRA" V 6850 2750 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 6950 2300 50  0001 C CNN
F 3 "~" H 6950 2300 50  0001 C CNN
	1    6950 2300
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_FET:2N7000 Q5
U 1 1 5F436F0A
P 6850 3150
F 0 "Q5" H 6750 3300 50  0000 L CNN
F 1 "2N7000;SOT23" H 6250 3100 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7050 3075 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 6850 3150 50  0001 L CNN
	1    6850 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R40
U 1 1 5F436F14
P 6700 1950
F 0 "R40" V 6600 1850 50  0000 L CNN
F 1 "330Ω;1608" V 6800 1750 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6740 1940 50  0001 C CNN
F 3 "~" H 6700 1950 50  0001 C CNN
	1    6700 1950
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R39
U 1 1 5F436F1E
P 6700 1650
F 0 "R39" V 6600 1550 50  0000 L CNN
F 1 "330Ω;1608" V 6800 1450 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6740 1640 50  0001 C CNN
F 3 "~" H 6700 1650 50  0001 C CNN
	1    6700 1650
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R38
U 1 1 5F436F28
P 6700 1350
F 0 "R38" V 6600 1250 50  0000 L CNN
F 1 "330Ω;1608" V 6800 1150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6740 1340 50  0001 C CNN
F 3 "~" H 6700 1350 50  0001 C CNN
	1    6700 1350
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 1650 6450 1650
Wire Wire Line
	6450 1650 6450 1950
Wire Wire Line
	6550 1950 6450 1950
Wire Wire Line
	6850 1950 6950 1950
Wire Wire Line
	6950 1950 6950 1650
Wire Wire Line
	6950 1650 6850 1650
Wire Wire Line
	6950 1650 6950 1350
Wire Wire Line
	6950 1350 6850 1350
Connection ~ 6950 1650
Wire Wire Line
	6450 1650 6450 1350
Wire Wire Line
	6450 1350 6550 1350
Connection ~ 6450 1650
Wire Wire Line
	6950 2150 6950 1950
Connection ~ 6950 1950
Wire Wire Line
	6950 2950 6950 2450
Wire Wire Line
	5650 1350 5650 1050
Wire Wire Line
	5650 1050 6450 1050
Wire Wire Line
	6450 1050 6450 1350
Connection ~ 5650 1350
Connection ~ 6450 1350
$Comp
L Device:LED D4
U 1 1 5F43BB9D
P 7750 2300
F 0 "D4" V 7850 2400 50  0000 R CNN
F 1 "LED;GRNRA" V 7650 2750 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 7750 2300 50  0001 C CNN
F 3 "~" H 7750 2300 50  0001 C CNN
	1    7750 2300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R37
U 1 1 5F43BBB1
P 5900 1950
F 0 "R37" V 5800 1850 50  0000 L CNN
F 1 "330Ω;1608" V 6000 1750 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5940 1940 50  0001 C CNN
F 3 "~" H 5900 1950 50  0001 C CNN
	1    5900 1950
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R36
U 1 1 5F43BBBB
P 5900 1650
F 0 "R36" V 5800 1550 50  0000 L CNN
F 1 "330Ω;1608" V 6000 1450 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5940 1640 50  0001 C CNN
F 3 "~" H 5900 1650 50  0001 C CNN
	1    5900 1650
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R35
U 1 1 5F43BBC5
P 5900 1350
F 0 "R35" V 5800 1250 50  0000 L CNN
F 1 "330Ω;1608" V 6000 1150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5940 1340 50  0001 C CNN
F 3 "~" H 5900 1350 50  0001 C CNN
	1    5900 1350
	0    1    1    0   
$EndComp
Wire Wire Line
	7350 1650 7250 1650
Wire Wire Line
	7250 1650 7250 1950
Wire Wire Line
	7350 1950 7250 1950
Wire Wire Line
	7650 1950 7750 1950
Wire Wire Line
	7750 1950 7750 1650
Wire Wire Line
	7750 1650 7650 1650
Wire Wire Line
	7750 1650 7750 1350
Wire Wire Line
	7750 1350 7650 1350
Connection ~ 7750 1650
Wire Wire Line
	7250 1650 7250 1350
Wire Wire Line
	7250 1350 7350 1350
Connection ~ 7250 1650
Wire Wire Line
	7750 2150 7750 1950
Connection ~ 7750 1950
Wire Wire Line
	7750 2950 7750 2450
Wire Wire Line
	7250 1050 7250 1350
Connection ~ 7250 1350
Wire Wire Line
	6450 1050 7250 1050
Connection ~ 6450 1050
Wire Wire Line
	6150 3350 6150 3450
Wire Wire Line
	6150 3450 6950 3450
Wire Wire Line
	6950 3450 6950 3350
Wire Wire Line
	6950 3450 7750 3450
Wire Wire Line
	7750 3450 7750 3350
Connection ~ 6950 3450
Wire Wire Line
	4000 3450 4600 3450
Connection ~ 6150 3450
Wire Wire Line
	4000 1050 5650 1050
Connection ~ 5650 1050
Wire Wire Line
	4000 2300 4400 2300
Wire Wire Line
	4400 2300 4400 1700
Wire Wire Line
	4400 1700 4800 1700
Wire Wire Line
	4000 2200 4500 2200
Wire Wire Line
	4500 2200 4500 1800
Wire Wire Line
	4500 1800 4800 1800
Wire Wire Line
	4800 1900 4600 1900
Wire Wire Line
	4600 1900 4600 3450
Connection ~ 4600 3450
Wire Wire Line
	4600 3450 6150 3450
Wire Wire Line
	4000 1600 4800 1600
$Comp
L Transistor_FET:2N7000 Q6
U 1 1 5F43BBA7
P 7650 3150
F 0 "Q6" H 7550 3300 50  0000 L CNN
F 1 "2N7000;SOT23" H 7050 3100 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7850 3075 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 7650 3150 50  0001 L CNN
	1    7650 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3150 4800 3150
Wire Wire Line
	4800 3150 4800 2600
Wire Wire Line
	4800 2600 4000 2600
Wire Wire Line
	4000 2500 4900 2500
Wire Wire Line
	4900 2500 4900 2850
Wire Wire Line
	4900 2850 6550 2850
Wire Wire Line
	6550 2850 6550 3150
Wire Wire Line
	6550 3150 6650 3150
Wire Wire Line
	7450 3150 7350 3150
Wire Wire Line
	7350 3150 7350 2700
Wire Wire Line
	7350 2700 5000 2700
Wire Wire Line
	5000 2700 5000 2400
Wire Wire Line
	5000 2400 4000 2400
Text Label 4100 2400 0    50   ~ 0
LED3
Text Label 4100 2500 0    50   ~ 0
LED2
Text Label 4100 2600 0    50   ~ 0
LED1
Text Label 4100 2300 0    50   ~ 0
TRIG1
Text Label 4100 2200 0    50   ~ 0
ECHO1
Text Label 4100 1600 0    50   ~ 0
5V
Text Label 4100 1050 0    50   ~ 0
9V
Text Notes 3500 4000 0    50   ~ 0
Note: CN101 is actually 4 connectors that are treated one to keep\n   them properly aligned with C102.
Text HLabel 4000 1050 0    50   Input ~ 0
9V
Text HLabel 4000 1600 0    50   Input ~ 0
5V
Text HLabel 4000 3450 0    50   Input ~ 0
GND
Text HLabel 4000 2300 0    50   Input ~ 0
TRIG1
Text HLabel 4000 2400 0    50   Input ~ 0
LED1
Text HLabel 4000 2500 0    50   Input ~ 0
LED2
Text HLabel 4000 2600 0    50   Input ~ 0
LED3
Text HLabel 4000 2200 0    50   Output ~ 0
ECHO1
$EndSCHEMATC
