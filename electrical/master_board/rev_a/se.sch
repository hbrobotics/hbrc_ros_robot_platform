EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 9 13
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
L HR2:HOLE;M2.5 H?
U 1 1 5F3FF6FB
P 2600 4100
AR Path="/5F3DFBDE/5F3FF6FB" Ref="H?"  Part="1" 
AR Path="/5F3FF6FB" Ref="H3"  Part="1" 
AR Path="/5F79E273/5F3FF6FB" Ref="H3"  Part="1" 
F 0 "H3" H 2550 4150 50  0000 L CNN
F 1 "HOLE;M2.5" H 2400 4050 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 2800 4150 60  0001 L CNN
F 3 "M2.5 Mounting Hole" H 2800 3950 60  0001 L CNN
	1    2600 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 5F3FF701
P 5350 3800
AR Path="/5F3DFBDE/5F3FF701" Ref="D?"  Part="1" 
AR Path="/5F3FF701" Ref="D15"  Part="1" 
AR Path="/5F79E273/5F3FF701" Ref="D6"  Part="1" 
F 0 "D6" V 5450 3950 50  0000 R CNN
F 1 "LED;GRNRA" V 5250 4250 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 5350 3800 50  0001 C CNN
F 3 "~" H 5350 3800 50  0001 C CNN
	1    5350 3800
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D?
U 1 1 5F3FF707
P 7150 3800
AR Path="/5F3DFBDE/5F3FF707" Ref="D?"  Part="1" 
AR Path="/5F3FF707" Ref="D13"  Part="1" 
AR Path="/5F79E273/5F3FF707" Ref="D8"  Part="1" 
F 0 "D8" V 7250 3950 50  0000 R CNN
F 1 "LED;GRNRA" V 7050 4250 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 7150 3800 50  0001 C CNN
F 3 "~" H 7150 3800 50  0001 C CNN
	1    7150 3800
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D?
U 1 1 5F3FF70D
P 8050 3800
AR Path="/5F3DFBDE/5F3FF70D" Ref="D?"  Part="1" 
AR Path="/5F3FF70D" Ref="D12"  Part="1" 
AR Path="/5F79E273/5F3FF70D" Ref="D9"  Part="1" 
F 0 "D9" V 8150 3950 50  0000 R CNN
F 1 "LED;GRNRA" V 7950 4250 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 8050 3800 50  0001 C CNN
F 3 "~" H 8050 3800 50  0001 C CNN
	1    8050 3800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8050 4550 8050 4450
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F3FF71D
P 7950 4250
AR Path="/5F3DFBDE/5F3FF71D" Ref="Q?"  Part="1" 
AR Path="/5F3FF71D" Ref="Q12"  Part="1" 
AR Path="/5F79E273/5F3FF71D" Ref="Q11"  Part="1" 
F 0 "Q11" H 7800 4400 50  0000 L CNN
F 1 "2N7000;SOT23" H 7350 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8150 4175 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 7950 4250 50  0001 L CNN
	1    7950 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF723
P 7800 3150
AR Path="/5F3DFBDE/5F3FF723" Ref="R?"  Part="1" 
AR Path="/5F3FF723" Ref="R37"  Part="1" 
AR Path="/5F79E273/5F3FF723" Ref="R57"  Part="1" 
F 0 "R57" V 7700 3050 50  0000 L CNN
F 1 "330Ω;1608" V 7900 2950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7840 3140 50  0001 C CNN
F 3 "~" H 7800 3150 50  0001 C CNN
	1    7800 3150
	0    1    1    0   
$EndComp
Text Notes 2750 6100 0    50   ~ 0
Notes:\n1. GRNRA = Green Right Angle Green\n2. Nominal Part:\n     Mfg Part #: TLPG5600\n     Digi-Key part #: TLPG5600-ND\n     Forward Voltage Drop: 2.4V\n     Absolute max. current: 200mA\n     Target maximum current: I=60mA\n     Resistor: R = (9V - 2.4V) / 75mA = 6.6V / .060A = 110Ω\n     Watts: W = I**2*R = (.060 * .060) * 110 = .396W ~=.4W\n     Use 3 x 330Ω in parallel to get 110Ω\n     Watts per resistor: Wr = .4W / 3 = .133W ~= .125W\n     1/8W or larger should work.\n3. LED's are labeled from 1 to 16 in a counter-clockwise\n     direction starting in the NW quadrant nearest the X axis.
Wire Wire Line
	8050 4050 8050 3950
$Comp
L Device:R_US R?
U 1 1 5F3FF72B
P 7800 2850
AR Path="/5F3DFBDE/5F3FF72B" Ref="R?"  Part="1" 
AR Path="/5F3FF72B" Ref="R38"  Part="1" 
AR Path="/5F79E273/5F3FF72B" Ref="R56"  Part="1" 
F 0 "R56" V 7700 2750 50  0000 L CNN
F 1 "330Ω;1608" V 7900 2650 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7840 2840 50  0001 C CNN
F 3 "~" H 7800 2850 50  0001 C CNN
	1    7800 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF731
P 7800 3450
AR Path="/5F3DFBDE/5F3FF731" Ref="R?"  Part="1" 
AR Path="/5F3FF731" Ref="R36"  Part="1" 
AR Path="/5F79E273/5F3FF731" Ref="R58"  Part="1" 
F 0 "R58" V 7700 3350 50  0000 L CNN
F 1 "330Ω;1608" V 7900 3250 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7840 3440 50  0001 C CNN
F 3 "~" H 7800 3450 50  0001 C CNN
	1    7800 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	7950 3450 8050 3450
Wire Wire Line
	8050 3450 8050 3150
Wire Wire Line
	8050 2850 7950 2850
Wire Wire Line
	7950 3150 8050 3150
Connection ~ 8050 3150
Wire Wire Line
	8050 3150 8050 2850
Wire Wire Line
	7650 3450 7550 3450
Wire Wire Line
	7550 3450 7550 3150
Wire Wire Line
	7550 2850 7650 2850
Wire Wire Line
	7650 3150 7550 3150
Connection ~ 7550 3150
Wire Wire Line
	7550 3150 7550 2850
Wire Wire Line
	8050 3650 8050 3450
Connection ~ 8050 3450
Wire Wire Line
	7150 4550 7150 4450
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F3FF746
P 7050 4250
AR Path="/5F3DFBDE/5F3FF746" Ref="Q?"  Part="1" 
AR Path="/5F3FF746" Ref="Q13"  Part="1" 
AR Path="/5F79E273/5F3FF746" Ref="Q10"  Part="1" 
F 0 "Q10" H 6900 4400 50  0000 L CNN
F 1 "2N7000;SOT23" H 6450 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7250 4175 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 7050 4250 50  0001 L CNN
	1    7050 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF74C
P 6900 3150
AR Path="/5F3DFBDE/5F3FF74C" Ref="R?"  Part="1" 
AR Path="/5F3FF74C" Ref="R40"  Part="1" 
AR Path="/5F79E273/5F3FF74C" Ref="R54"  Part="1" 
F 0 "R54" V 6800 3050 50  0000 L CNN
F 1 "330Ω;1608" V 7000 2950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6940 3140 50  0001 C CNN
F 3 "~" H 6900 3150 50  0001 C CNN
	1    6900 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	7150 4050 7150 3950
$Comp
L Device:R_US R?
U 1 1 5F3FF753
P 6900 2850
AR Path="/5F3DFBDE/5F3FF753" Ref="R?"  Part="1" 
AR Path="/5F3FF753" Ref="R41"  Part="1" 
AR Path="/5F79E273/5F3FF753" Ref="R53"  Part="1" 
F 0 "R53" V 6800 2750 50  0000 L CNN
F 1 "330Ω;1608" V 7000 2650 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6940 2840 50  0001 C CNN
F 3 "~" H 6900 2850 50  0001 C CNN
	1    6900 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF759
P 6900 3450
AR Path="/5F3DFBDE/5F3FF759" Ref="R?"  Part="1" 
AR Path="/5F3FF759" Ref="R39"  Part="1" 
AR Path="/5F79E273/5F3FF759" Ref="R55"  Part="1" 
F 0 "R55" V 6800 3350 50  0000 L CNN
F 1 "330Ω;1608" V 7000 3250 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6940 3440 50  0001 C CNN
F 3 "~" H 6900 3450 50  0001 C CNN
	1    6900 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 3450 7150 3450
Wire Wire Line
	7150 3450 7150 3150
Wire Wire Line
	7150 2850 7050 2850
Wire Wire Line
	7050 3150 7150 3150
Connection ~ 7150 3150
Wire Wire Line
	7150 3150 7150 2850
Wire Wire Line
	6750 3450 6650 3450
Wire Wire Line
	6650 3450 6650 3150
Wire Wire Line
	6650 2850 6750 2850
Wire Wire Line
	6750 3150 6650 3150
Connection ~ 6650 3150
Wire Wire Line
	6650 3150 6650 2850
Wire Wire Line
	7150 3650 7150 3450
Connection ~ 7150 3450
Wire Wire Line
	5850 3450 5750 3450
Wire Wire Line
	5750 3450 5750 3150
Wire Wire Line
	5750 2850 5850 2850
Wire Wire Line
	5850 3150 5750 3150
Connection ~ 5750 3150
Wire Wire Line
	5750 3150 5750 2850
Wire Wire Line
	5350 4550 5350 4450
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F3FF774
P 5250 4250
AR Path="/5F3DFBDE/5F3FF774" Ref="Q?"  Part="1" 
AR Path="/5F3FF774" Ref="Q15"  Part="1" 
AR Path="/5F79E273/5F3FF774" Ref="Q8"  Part="1" 
F 0 "Q8" H 5100 4400 50  0000 L CNN
F 1 "2N7000;SOT23" H 4650 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5450 4175 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 5250 4250 50  0001 L CNN
	1    5250 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF77A
P 5100 3150
AR Path="/5F3DFBDE/5F3FF77A" Ref="R?"  Part="1" 
AR Path="/5F3FF77A" Ref="R46"  Part="1" 
AR Path="/5F79E273/5F3FF77A" Ref="R48"  Part="1" 
F 0 "R48" V 5000 3050 50  0000 L CNN
F 1 "330Ω;1608" V 5200 2950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5140 3140 50  0001 C CNN
F 3 "~" H 5100 3150 50  0001 C CNN
	1    5100 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 4050 5350 3950
$Comp
L Device:R_US R?
U 1 1 5F3FF781
P 5100 2850
AR Path="/5F3DFBDE/5F3FF781" Ref="R?"  Part="1" 
AR Path="/5F3FF781" Ref="R47"  Part="1" 
AR Path="/5F79E273/5F3FF781" Ref="R47"  Part="1" 
F 0 "R47" V 5000 2750 50  0000 L CNN
F 1 "330Ω;1608" V 5200 2650 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5140 2840 50  0001 C CNN
F 3 "~" H 5100 2850 50  0001 C CNN
	1    5100 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF787
P 5100 3450
AR Path="/5F3DFBDE/5F3FF787" Ref="R?"  Part="1" 
AR Path="/5F3FF787" Ref="R45"  Part="1" 
AR Path="/5F79E273/5F3FF787" Ref="R49"  Part="1" 
F 0 "R49" V 5000 3350 50  0000 L CNN
F 1 "330Ω;1608" V 5200 3250 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5140 3440 50  0001 C CNN
F 3 "~" H 5100 3450 50  0001 C CNN
	1    5100 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	5250 3450 5350 3450
Wire Wire Line
	5350 3450 5350 3150
Wire Wire Line
	5350 2850 5250 2850
Wire Wire Line
	5250 3150 5350 3150
Connection ~ 5350 3150
Wire Wire Line
	5350 3150 5350 2850
Wire Wire Line
	4950 3450 4850 3450
Wire Wire Line
	4850 3450 4850 3150
Wire Wire Line
	4850 2850 4950 2850
Wire Wire Line
	4950 3150 4850 3150
Connection ~ 4850 3150
Wire Wire Line
	4850 3150 4850 2850
Wire Wire Line
	5350 3650 5350 3450
Connection ~ 5350 3450
Wire Wire Line
	4450 4550 4450 4450
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F3FF79C
P 4350 4250
AR Path="/5F3DFBDE/5F3FF79C" Ref="Q?"  Part="1" 
AR Path="/5F3FF79C" Ref="Q16"  Part="1" 
AR Path="/5F79E273/5F3FF79C" Ref="Q7"  Part="1" 
F 0 "Q7" H 4200 4400 50  0000 L CNN
F 1 "2N7000;SOT23" H 3750 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4550 4175 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 4350 4250 50  0001 L CNN
	1    4350 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF7A2
P 4200 3150
AR Path="/5F3DFBDE/5F3FF7A2" Ref="R?"  Part="1" 
AR Path="/5F3FF7A2" Ref="R49"  Part="1" 
AR Path="/5F79E273/5F3FF7A2" Ref="R45"  Part="1" 
F 0 "R45" V 4100 3050 50  0000 L CNN
F 1 "330Ω;1608" V 4300 2950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4240 3140 50  0001 C CNN
F 3 "~" H 4200 3150 50  0001 C CNN
	1    4200 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF7A8
P 4200 2850
AR Path="/5F3DFBDE/5F3FF7A8" Ref="R?"  Part="1" 
AR Path="/5F3FF7A8" Ref="R50"  Part="1" 
AR Path="/5F79E273/5F3FF7A8" Ref="R44"  Part="1" 
F 0 "R44" V 4100 2750 50  0000 L CNN
F 1 "330Ω;1608" V 4300 2650 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4240 2840 50  0001 C CNN
F 3 "~" H 4200 2850 50  0001 C CNN
	1    4200 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF7AE
P 4200 3450
AR Path="/5F3DFBDE/5F3FF7AE" Ref="R?"  Part="1" 
AR Path="/5F3FF7AE" Ref="R48"  Part="1" 
AR Path="/5F79E273/5F3FF7AE" Ref="R46"  Part="1" 
F 0 "R46" V 4100 3350 50  0000 L CNN
F 1 "330Ω;1608" V 4300 3250 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4240 3440 50  0001 C CNN
F 3 "~" H 4200 3450 50  0001 C CNN
	1    4200 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 3450 4450 3450
Wire Wire Line
	4450 3450 4450 3150
Wire Wire Line
	4450 2850 4350 2850
Wire Wire Line
	4350 3150 4450 3150
Connection ~ 4450 3150
Wire Wire Line
	4450 3150 4450 2850
Wire Wire Line
	4050 3450 3950 3450
Wire Wire Line
	3950 3450 3950 3150
Wire Wire Line
	3950 2850 4050 2850
Wire Wire Line
	4050 3150 3950 3150
Connection ~ 3950 3150
Wire Wire Line
	3950 3150 3950 2850
Connection ~ 4450 3450
Wire Wire Line
	3200 2550 3850 2550
Wire Wire Line
	8050 4550 7150 4550
Wire Wire Line
	7150 4550 6250 4550
Wire Wire Line
	5350 4550 4450 4550
Wire Wire Line
	4450 4050 4450 3950
Wire Wire Line
	4450 3650 4450 3450
$Comp
L Device:LED D?
U 1 1 5F3FF7C8
P 4450 3800
AR Path="/5F3DFBDE/5F3FF7C8" Ref="D?"  Part="1" 
AR Path="/5F3FF7C8" Ref="D16"  Part="1" 
AR Path="/5F79E273/5F3FF7C8" Ref="D5"  Part="1" 
F 0 "D5" V 4550 3950 50  0000 R CNN
F 1 "LED;GRNRA" V 4350 4250 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 4450 3800 50  0001 C CNN
F 3 "~" H 4450 3800 50  0001 C CNN
	1    4450 3800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 4550 6250 4550
Connection ~ 6250 3450
Wire Wire Line
	6250 3650 6250 3450
Wire Wire Line
	6250 3150 6250 2850
Connection ~ 6250 3150
Wire Wire Line
	6150 3150 6250 3150
Wire Wire Line
	6250 2850 6150 2850
Wire Wire Line
	6250 3450 6250 3150
Wire Wire Line
	6150 3450 6250 3450
$Comp
L Device:R_US R?
U 1 1 5F3FF7D7
P 6000 3450
AR Path="/5F3DFBDE/5F3FF7D7" Ref="R?"  Part="1" 
AR Path="/5F3FF7D7" Ref="R42"  Part="1" 
AR Path="/5F79E273/5F3FF7D7" Ref="R52"  Part="1" 
F 0 "R52" V 5900 3350 50  0000 L CNN
F 1 "330Ω;1608" V 6100 3250 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6040 3440 50  0001 C CNN
F 3 "~" H 6000 3450 50  0001 C CNN
	1    6000 3450
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5F3FF7DD
P 6000 2850
AR Path="/5F3DFBDE/5F3FF7DD" Ref="R?"  Part="1" 
AR Path="/5F3FF7DD" Ref="R44"  Part="1" 
AR Path="/5F79E273/5F3FF7DD" Ref="R50"  Part="1" 
F 0 "R50" V 5900 2750 50  0000 L CNN
F 1 "330Ω;1608" V 6100 2650 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6040 2840 50  0001 C CNN
F 3 "~" H 6000 2850 50  0001 C CNN
	1    6000 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 4050 6250 3950
$Comp
L Device:R_US R?
U 1 1 5F3FF7E4
P 6000 3150
AR Path="/5F3DFBDE/5F3FF7E4" Ref="R?"  Part="1" 
AR Path="/5F3FF7E4" Ref="R43"  Part="1" 
AR Path="/5F79E273/5F3FF7E4" Ref="R51"  Part="1" 
F 0 "R51" V 5900 3050 50  0000 L CNN
F 1 "330Ω;1608" V 6100 2950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6040 3140 50  0001 C CNN
F 3 "~" H 6000 3150 50  0001 C CNN
	1    6000 3150
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F3FF7EA
P 6150 4250
AR Path="/5F3DFBDE/5F3FF7EA" Ref="Q?"  Part="1" 
AR Path="/5F3FF7EA" Ref="Q14"  Part="1" 
AR Path="/5F79E273/5F3FF7EA" Ref="Q9"  Part="1" 
F 0 "Q9" H 6000 4400 50  0000 L CNN
F 1 "2N7000;SOT23" H 5550 4200 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6350 4175 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 6150 4250 50  0001 L CNN
	1    6150 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 4550 6250 4450
$Comp
L Device:LED D?
U 1 1 5F3FF7F1
P 6250 3800
AR Path="/5F3DFBDE/5F3FF7F1" Ref="D?"  Part="1" 
AR Path="/5F3FF7F1" Ref="D14"  Part="1" 
AR Path="/5F79E273/5F3FF7F1" Ref="D7"  Part="1" 
F 0 "D7" V 6350 3950 50  0000 R CNN
F 1 "LED;GRNRA" V 6150 4250 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 6250 3800 50  0001 C CNN
F 3 "~" H 6250 3800 50  0001 C CNN
	1    6250 3800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 4250 3850 2550
Wire Wire Line
	4750 4250 4750 2450
Wire Wire Line
	3200 2450 4750 2450
Wire Wire Line
	5650 4250 5650 2350
Wire Wire Line
	3200 2350 5650 2350
Wire Wire Line
	6850 4250 6550 4250
Wire Wire Line
	6550 4250 6550 2250
Wire Wire Line
	3200 2250 6550 2250
Wire Wire Line
	7750 4250 7450 4250
Connection ~ 4450 4550
Connection ~ 5350 4550
Connection ~ 6250 4550
Connection ~ 7150 4550
Wire Wire Line
	3950 2050 3950 2850
Connection ~ 3950 2850
Wire Wire Line
	3950 2050 4850 2050
Wire Wire Line
	4850 2050 4850 2850
Connection ~ 4850 2850
Wire Wire Line
	4850 2050 5750 2050
Wire Wire Line
	5750 2050 5750 2850
Connection ~ 4850 2050
Connection ~ 5750 2850
Wire Wire Line
	5750 2050 6650 2050
Wire Wire Line
	6650 2050 6650 2850
Connection ~ 5750 2050
Connection ~ 6650 2850
Wire Wire Line
	6650 2050 7550 2050
Wire Wire Line
	7550 2050 7550 2850
Connection ~ 6650 2050
Connection ~ 7550 2850
Text Label 3900 4250 0    50   ~ 0
LED16
Text Label 4800 4250 0    50   ~ 0
LED15
Text Label 5700 4250 0    50   ~ 0
LED14
Text Label 6600 4250 0    50   ~ 0
LED13
Text Label 7500 4250 0    50   ~ 0
LED12
Text Label 5750 4550 0    50   ~ 0
GND
Text Label 5850 2050 0    50   ~ 0
9V
Text Notes 2750 6250 0    50   ~ 0
Note: LED's are labeled from 1 to 16 in a clockwise\ndirection starting in the NW quadrant near the X axis.
Text Notes 2100 3900 0    50   ~ 0
Note: Mounting Hole for above\narm expansion plates.
Wire Wire Line
	3200 2050 3950 2050
Connection ~ 3950 2050
Wire Wire Line
	3200 2150 7450 2150
Wire Wire Line
	7450 2150 7450 4250
NoConn ~ 1500 1800
Text HLabel 3200 4550 0    50   Input ~ 0
GND
Wire Wire Line
	3200 4550 4450 4550
Text HLabel 3200 2050 0    50   Input ~ 0
9V
Text HLabel 3200 2150 0    50   Input ~ 0
LED12
Text HLabel 3200 2250 0    50   Input ~ 0
LED13
Text HLabel 3200 2350 0    50   Input ~ 0
LED14
Text HLabel 3200 2450 0    50   Input ~ 0
LED15
Text HLabel 3200 2550 0    50   Input ~ 0
LED16
Wire Wire Line
	3850 4250 4150 4250
Wire Wire Line
	5050 4250 4750 4250
Wire Wire Line
	5950 4250 5650 4250
$EndSCHEMATC
