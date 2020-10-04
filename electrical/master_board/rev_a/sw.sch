EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 11 13
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
L HR2:HOLE;M2.5 H4
U 1 1 5F2F7A6B
P 2650 4450
F 0 "H4" H 2600 4500 50  0000 L CNN
F 1 "HOLE;M2.5" H 2450 4400 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 2850 4500 60  0001 L CNN
F 3 "M2.5 Mounting Hole" H 2850 4300 60  0001 L CNN
	1    2650 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D11
U 1 1 5F30CB7A
P 5400 4150
F 0 "D11" V 5500 4250 50  0000 R CNN
F 1 "LED;GRNRA" V 5300 4600 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 5400 4150 50  0001 C CNN
F 3 "~" H 5400 4150 50  0001 C CNN
	1    5400 4150
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D13
U 1 1 5F30CF92
P 7200 4150
F 0 "D13" V 7300 4300 50  0000 R CNN
F 1 "LED;GRNRA" V 7100 4600 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 7200 4150 50  0001 C CNN
F 3 "~" H 7200 4150 50  0001 C CNN
	1    7200 4150
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D14
U 1 1 5F30EB24
P 8100 4150
F 0 "D14" V 8200 4300 50  0000 R CNN
F 1 "LED;GRNRA" V 8000 4600 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 8100 4150 50  0001 C CNN
F 3 "~" H 8100 4150 50  0001 C CNN
	1    8100 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8100 4900 8100 4800
Text Label 6000 4900 0    50   ~ 0
GND
$Comp
L Transistor_FET:2N7000 Q16
U 1 1 5F39F103
P 8000 4600
F 0 "Q16" H 7850 4750 50  0000 L CNN
F 1 "2N7000;SOT23" H 7400 4550 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8200 4525 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 8000 4600 50  0001 L CNN
	1    8000 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R72
U 1 1 5F3A3A4B
P 7850 3500
F 0 "R72" V 7750 3400 50  0000 L CNN
F 1 "330Ω;1608" V 7950 3300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7890 3490 50  0001 C CNN
F 3 "~" H 7850 3500 50  0001 C CNN
	1    7850 3500
	0    1    1    0   
$EndComp
Text Notes 2800 6450 0    50   ~ 0
Notes:\n1. GRNRA = Green Right Angle Green\n2. Nominal Part:\n     Mfg Part #: TLPG5600\n     Digi-Key part #: TLPG5600-ND\n     Forward Voltage Drop: 2.4V\n     Absolute max. current: 200mA\n     Target maximum current: I=60mA\n     Resistor: R = (9V - 2.4V) / 75mA = 6.6V / .060A = 110Ω\n     Watts: W = I**2*R = (.060 * .060) * 110 = .396W ~~=.4W\n     Use 3 x 330Ω in parallel to get 110Ω\n     Watts per resistor: Wr = .4W / 3 = .133W ~~= .125W\n     1/8W or larger should work.\n3. LED's and transistors are labeled from 1 to 16 in a clockwise\n     direction starting in the NW quadrant near the X axis.  The\n     resesitors are labeld Ra, Rb, and Rc, where a=3*n-2, b=3*n-1,\n     and c=3*n, where n is the LED number.
Wire Wire Line
	8100 4400 8100 4300
$Comp
L Device:R_US R71
U 1 1 5F3A820E
P 7850 3200
F 0 "R71" V 7750 3100 50  0000 L CNN
F 1 "330Ω;1608" V 7950 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7890 3190 50  0001 C CNN
F 3 "~" H 7850 3200 50  0001 C CNN
	1    7850 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R73
U 1 1 5F3A878F
P 7850 3800
F 0 "R73" V 7750 3700 50  0000 L CNN
F 1 "330Ω;1608" V 7950 3600 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7890 3790 50  0001 C CNN
F 3 "~" H 7850 3800 50  0001 C CNN
	1    7850 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 3800 8100 3800
Wire Wire Line
	8100 3800 8100 3500
Wire Wire Line
	8100 3200 8000 3200
Wire Wire Line
	8000 3500 8100 3500
Connection ~ 8100 3500
Wire Wire Line
	8100 3500 8100 3200
Wire Wire Line
	7700 3800 7600 3800
Wire Wire Line
	7600 3800 7600 3500
Wire Wire Line
	7600 3200 7700 3200
Wire Wire Line
	7700 3500 7600 3500
Connection ~ 7600 3500
Wire Wire Line
	7600 3500 7600 3200
Wire Wire Line
	8100 4000 8100 3800
Connection ~ 8100 3800
Wire Wire Line
	7200 4900 7200 4800
$Comp
L Transistor_FET:2N7000 Q15
U 1 1 5F3B5AC4
P 7100 4600
F 0 "Q15" H 6950 4750 50  0000 L CNN
F 1 "2N7000;SOT23" H 6500 4550 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7300 4525 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 7100 4600 50  0001 L CNN
	1    7100 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R69
U 1 1 5F3B5ACE
P 6950 3500
F 0 "R69" V 6850 3400 50  0000 L CNN
F 1 "330Ω;1608" V 7050 3300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6990 3490 50  0001 C CNN
F 3 "~" H 6950 3500 50  0001 C CNN
	1    6950 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	7200 4400 7200 4300
$Comp
L Device:R_US R68
U 1 1 5F3B5AD9
P 6950 3200
F 0 "R68" V 6850 3100 50  0000 L CNN
F 1 "330Ω;1608" V 7050 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6990 3190 50  0001 C CNN
F 3 "~" H 6950 3200 50  0001 C CNN
	1    6950 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R70
U 1 1 5F3B5AE3
P 6950 3800
F 0 "R70" V 6850 3700 50  0000 L CNN
F 1 "330Ω;1608" V 7050 3600 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6990 3790 50  0001 C CNN
F 3 "~" H 6950 3800 50  0001 C CNN
	1    6950 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	7100 3800 7200 3800
Wire Wire Line
	7200 3800 7200 3500
Wire Wire Line
	7200 3200 7100 3200
Wire Wire Line
	7100 3500 7200 3500
Connection ~ 7200 3500
Wire Wire Line
	7200 3500 7200 3200
Wire Wire Line
	6800 3800 6700 3800
Wire Wire Line
	6700 3800 6700 3500
Wire Wire Line
	6700 3200 6800 3200
Wire Wire Line
	6800 3500 6700 3500
Connection ~ 6700 3500
Wire Wire Line
	6700 3500 6700 3200
Wire Wire Line
	7200 4000 7200 3800
Connection ~ 7200 3800
Wire Wire Line
	5900 3800 5800 3800
Wire Wire Line
	5800 3800 5800 3500
Wire Wire Line
	5800 3200 5900 3200
Wire Wire Line
	5900 3500 5800 3500
Connection ~ 5800 3500
Wire Wire Line
	5800 3500 5800 3200
Wire Wire Line
	5400 4900 5400 4800
$Comp
L Transistor_FET:2N7000 Q13
U 1 1 5F3B8D4A
P 5300 4600
F 0 "Q13" H 5150 4750 50  0000 L CNN
F 1 "2N7000;SOT23" H 4700 4550 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5500 4525 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 5300 4600 50  0001 L CNN
	1    5300 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R63
U 1 1 5F3B8D54
P 5150 3500
F 0 "R63" V 5050 3400 50  0000 L CNN
F 1 "330Ω;1608" V 5250 3300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5190 3490 50  0001 C CNN
F 3 "~" H 5150 3500 50  0001 C CNN
	1    5150 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 4400 5400 4300
$Comp
L Device:R_US R62
U 1 1 5F3B8D5F
P 5150 3200
F 0 "R62" V 5050 3100 50  0000 L CNN
F 1 "330Ω;1608" V 5250 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5190 3190 50  0001 C CNN
F 3 "~" H 5150 3200 50  0001 C CNN
	1    5150 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R64
U 1 1 5F3B8D69
P 5150 3800
F 0 "R64" V 5050 3700 50  0000 L CNN
F 1 "330Ω;1608" V 5250 3600 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5190 3790 50  0001 C CNN
F 3 "~" H 5150 3800 50  0001 C CNN
	1    5150 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 3800 5400 3800
Wire Wire Line
	5400 3800 5400 3500
Wire Wire Line
	5400 3200 5300 3200
Wire Wire Line
	5300 3500 5400 3500
Connection ~ 5400 3500
Wire Wire Line
	5400 3500 5400 3200
Wire Wire Line
	5000 3800 4900 3800
Wire Wire Line
	4900 3800 4900 3500
Wire Wire Line
	4900 3200 5000 3200
Wire Wire Line
	5000 3500 4900 3500
Connection ~ 4900 3500
Wire Wire Line
	4900 3500 4900 3200
Wire Wire Line
	5400 4000 5400 3800
Connection ~ 5400 3800
Wire Wire Line
	4500 4900 4500 4800
$Comp
L Transistor_FET:2N7000 Q12
U 1 1 5F3BEA19
P 4400 4600
F 0 "Q12" H 4250 4750 50  0000 L CNN
F 1 "2N7000;SOT23" H 3800 4550 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4600 4525 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 4400 4600 50  0001 L CNN
	1    4400 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R60
U 1 1 5F3BEA23
P 4250 3500
F 0 "R60" V 4150 3400 50  0000 L CNN
F 1 "330Ω;1608" V 4350 3300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4290 3490 50  0001 C CNN
F 3 "~" H 4250 3500 50  0001 C CNN
	1    4250 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R59
U 1 1 5F3BEA2E
P 4250 3200
F 0 "R59" V 4150 3100 50  0000 L CNN
F 1 "330Ω;1608" V 4350 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4290 3190 50  0001 C CNN
F 3 "~" H 4250 3200 50  0001 C CNN
	1    4250 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R61
U 1 1 5F3BEA38
P 4250 3800
F 0 "R61" V 4150 3700 50  0000 L CNN
F 1 "330Ω;1608" V 4350 3600 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4290 3790 50  0001 C CNN
F 3 "~" H 4250 3800 50  0001 C CNN
	1    4250 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	4400 3800 4500 3800
Wire Wire Line
	4500 3800 4500 3500
Wire Wire Line
	4500 3200 4400 3200
Wire Wire Line
	4400 3500 4500 3500
Connection ~ 4500 3500
Wire Wire Line
	4500 3500 4500 3200
Wire Wire Line
	4100 3800 4000 3800
Wire Wire Line
	4000 3800 4000 3500
Wire Wire Line
	4000 3200 4100 3200
Wire Wire Line
	4100 3500 4000 3500
Connection ~ 4000 3500
Wire Wire Line
	4000 3500 4000 3200
Connection ~ 4500 3800
Wire Wire Line
	3300 2800 3900 2800
Wire Wire Line
	8100 4900 7200 4900
Wire Wire Line
	7200 4900 6300 4900
Wire Wire Line
	5400 4900 4500 4900
Wire Wire Line
	4500 4400 4500 4300
Wire Wire Line
	4500 4000 4500 3800
$Comp
L Device:LED D10
U 1 1 5F30BFB1
P 4500 4150
F 0 "D10" V 4600 4250 50  0000 R CNN
F 1 "LED;GRNRA" V 4400 4600 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 4500 4150 50  0001 C CNN
F 3 "~" H 4500 4150 50  0001 C CNN
	1    4500 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5400 4900 6300 4900
Connection ~ 6300 3800
Wire Wire Line
	6300 4000 6300 3800
Wire Wire Line
	6300 3500 6300 3200
Connection ~ 6300 3500
Wire Wire Line
	6200 3500 6300 3500
Wire Wire Line
	6300 3200 6200 3200
Wire Wire Line
	6300 3800 6300 3500
Wire Wire Line
	6200 3800 6300 3800
$Comp
L Device:R_US R67
U 1 1 5F3B8D25
P 6050 3800
F 0 "R67" V 5950 3700 50  0000 L CNN
F 1 "330Ω;1608" V 6150 3600 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6090 3790 50  0001 C CNN
F 3 "~" H 6050 3800 50  0001 C CNN
	1    6050 3800
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R65
U 1 1 5F3B8D1B
P 6050 3200
F 0 "R65" V 5950 3100 50  0000 L CNN
F 1 "330Ω;1608" V 6150 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6090 3190 50  0001 C CNN
F 3 "~" H 6050 3200 50  0001 C CNN
	1    6050 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 4400 6300 4300
$Comp
L Device:R_US R66
U 1 1 5F3B8D10
P 6050 3500
F 0 "R66" V 5950 3400 50  0000 L CNN
F 1 "330Ω;1608" V 6150 3300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6090 3490 50  0001 C CNN
F 3 "~" H 6050 3500 50  0001 C CNN
	1    6050 3500
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:2N7000 Q14
U 1 1 5F3B8D06
P 6200 4600
F 0 "Q14" H 6050 4750 50  0000 L CNN
F 1 "2N7000;SOT23" H 5600 4550 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6400 4525 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 6200 4600 50  0001 L CNN
	1    6200 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 4900 6300 4800
$Comp
L Device:LED D12
U 1 1 5F30CF88
P 6300 4150
F 0 "D12" V 6400 4250 50  0000 R CNN
F 1 "LED;GRNRA" V 6200 4600 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 6300 4150 50  0001 C CNN
F 3 "~" H 6300 4150 50  0001 C CNN
	1    6300 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3900 4600 3900 2800
Wire Wire Line
	5100 4600 4800 4600
Wire Wire Line
	4800 4600 4800 2700
Wire Wire Line
	3300 2700 4800 2700
Wire Wire Line
	6000 4600 5700 4600
Wire Wire Line
	5700 4600 5700 2600
Wire Wire Line
	3300 2600 5700 2600
Wire Wire Line
	6900 4600 6600 4600
Wire Wire Line
	6600 4600 6600 2500
Wire Wire Line
	3300 2500 6600 2500
Wire Wire Line
	7800 4600 7500 4600
Wire Wire Line
	7500 4600 7500 2400
Wire Wire Line
	3300 2400 7500 2400
Connection ~ 4500 4900
Connection ~ 5400 4900
Connection ~ 6300 4900
Connection ~ 7200 4900
Wire Wire Line
	3300 2200 4000 2200
Wire Wire Line
	4000 2200 4000 3200
Connection ~ 4000 3200
Wire Wire Line
	4000 2200 4900 2200
Wire Wire Line
	4900 2200 4900 3200
Connection ~ 4000 2200
Connection ~ 4900 3200
Wire Wire Line
	4900 2200 5800 2200
Wire Wire Line
	5800 2200 5800 3200
Connection ~ 4900 2200
Connection ~ 5800 3200
Wire Wire Line
	5800 2200 6700 2200
Wire Wire Line
	6700 2200 6700 3200
Connection ~ 5800 2200
Connection ~ 6700 3200
Wire Wire Line
	6700 2200 7600 2200
Wire Wire Line
	7600 2200 7600 3200
Connection ~ 6700 2200
Connection ~ 7600 3200
Text Label 3950 4600 0    50   ~ 0
LED7
Text Label 4850 4600 0    50   ~ 0
LED8
Text Label 5750 4600 0    50   ~ 0
LED9
Text Label 6650 4600 0    50   ~ 0
LED10
Text Label 7550 4600 0    50   ~ 0
LED11
Text Label 6000 2200 0    50   ~ 0
9V
Text Notes 2150 4250 0    50   ~ 0
Note: Mounting Hole for above\narm expansion plates.
Wire Wire Line
	4200 4600 3900 4600
Wire Wire Line
	3300 4900 4500 4900
Text HLabel 3300 4900 0    50   Input ~ 0
GND
Text HLabel 3300 2200 0    50   Input ~ 0
9V
Text HLabel 3300 2400 0    50   Input ~ 0
LED11
Text HLabel 3300 2500 0    50   Input ~ 0
LED10
Text HLabel 3300 2600 0    50   Input ~ 0
LED9
Text HLabel 3300 2700 0    50   Input ~ 0
LED8
Text HLabel 3300 2800 0    50   Input ~ 0
LED7
$EndSCHEMATC
