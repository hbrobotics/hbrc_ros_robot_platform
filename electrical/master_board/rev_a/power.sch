EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 2 10
Title "HR2 Power Supply"
Date "2020-11-22"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	7200 2700 7300 2700
Wire Wire Line
	7600 2900 7200 2900
NoConn ~ 7200 3500
$Comp
L Device:R_US R1
U 1 1 5F4AAFD8
P 2500 4150
F 0 "R1" H 2500 4300 50  0000 L CNN
F 1 "100KΩ;1608" H 2550 4050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2540 4140 50  0001 C CNN
F 3 "~" H 2500 4150 50  0001 C CNN
	1    2500 4150
	1    0    0    -1  
$EndComp
Text Notes 3100 3550 0    50   ~ 0
Note: Power On/Off circuit is\nfrom David Jones EEVBlog #262
$Comp
L Device:Q_NPN_BEC Q1
U 1 1 5F4ABAA0
P 2600 4700
F 0 "Q1" H 2500 4850 50  0000 L CNN
F 1 "2N2222;SOT23" H 2050 4550 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2800 4800 50  0001 C CNN
F 3 "~" H 2600 4700 50  0001 C CNN
	1    2600 4700
	-1   0    0    -1  
$EndComp
Text Notes 2650 5400 0    50   ~ 0
Note: The bipolar transtistors are generic\n  small signal NPN SOT23 in a BEC (i.e\n  pin1 = Base, pin 2 = Emitter,\n  pi 3 = Collector) configuration.\n  Pretty much any NPN will do, so\n  substitutions should be pretty safe.
$Comp
L Device:Q_NPN_BEC Q3
U 1 1 5F4AC33E
P 5000 5000
F 0 "Q3" H 5190 5046 50  0000 L CNN
F 1 "2N2222;SOT23" H 5190 4955 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5200 5100 50  0001 C CNN
F 3 "~" H 5000 5000 50  0001 C CNN
	1    5000 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 5F4AC83B
P 3800 4250
F 0 "R2" H 3650 4350 50  0000 L CNN
F 1 "100KΩ;1608" H 3850 4150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3840 4240 50  0001 C CNN
F 3 "~" H 3800 4250 50  0001 C CNN
	1    3800 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R4
U 1 1 5F4ACD3F
P 5100 4250
F 0 "R4" H 4950 4350 50  0000 L CNN
F 1 "100KΩ;1608" H 5150 4150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5140 4240 50  0001 C CNN
F 3 "~" H 5100 4250 50  0001 C CNN
	1    5100 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R3
U 1 1 5F4ACF72
P 4400 4250
F 0 "R3" H 4250 4350 50  0000 L CNN
F 1 "1MΩ;1608" H 4450 4150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4440 4240 50  0001 C CNN
F 3 "~" H 4400 4250 50  0001 C CNN
	1    4400 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 3800 2500 4000
Wire Wire Line
	2500 4300 2500 4400
Wire Wire Line
	2500 4400 3200 4400
Wire Wire Line
	2500 4500 2500 4400
Connection ~ 2500 4400
Wire Wire Line
	3800 4100 3800 3800
Wire Wire Line
	2800 4700 3800 4700
Wire Wire Line
	3800 4700 3800 4400
$Comp
L Switch:SW_Push SW1
U 1 1 5F4B612A
P 4100 4700
F 0 "SW1" H 3950 4800 50  0000 C CNN
F 1 "BUTTON;6x3.5" H 4050 4600 50  0000 C CNN
F 2 "HR2:BUTTON_6x3.5" H 4100 4900 50  0001 C CNN
F 3 "~" H 4100 4900 50  0001 C CNN
	1    4100 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 4700 3900 4700
Connection ~ 3800 4700
Connection ~ 2500 3800
Wire Wire Line
	3800 3800 4400 3800
Wire Wire Line
	4400 3800 4400 4100
Connection ~ 3800 3800
Wire Wire Line
	4300 4700 5100 4700
Wire Wire Line
	5100 4700 5100 4800
Wire Wire Line
	5100 4700 5100 4400
Connection ~ 5100 4700
Wire Wire Line
	4800 5000 4400 5000
Wire Wire Line
	4400 5000 4400 4400
$Comp
L Device:C C1
U 1 1 5F4BF513
P 4400 5250
F 0 "C1" H 4250 5350 50  0000 L CNN
F 1 "22µF;1608" H 4450 5150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4438 5100 50  0001 C CNN
F 3 "~" H 4400 5250 50  0001 C CNN
	1    4400 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 5400 4400 5500
Wire Wire Line
	4400 5500 5100 5500
Wire Wire Line
	5100 5500 5100 5200
Wire Wire Line
	2500 4900 2500 5000
Connection ~ 4400 5500
Connection ~ 4400 3800
Wire Wire Line
	4400 5100 4400 5000
Connection ~ 4400 5000
Connection ~ 5100 5500
Connection ~ 7600 5500
$Comp
L HR2:U3V70xMATE;F1x4+F1x5 CN?
U 1 1 5F5303B1
P 6400 3500
AR Path="/5F5303B1" Ref="CN?"  Part="1" 
AR Path="/5F52F39E/5F5303B1" Ref="CN93"  Part="1" 
F 0 "CN93" H 6700 2550 50  0000 C CNN
F 1 "POLOLU_U3V70F9;F1x4+F1x5" H 6800 3650 50  0000 C CNN
F 2 "HR2:U3V70xMATE_F1x4+F1x5" H 6600 3550 60  0001 L CNN
F 3 "Pololu U3V70x Voltage Regulator Mating Connector" H 6600 3350 60  0001 L CNN
F 4 "Pololu U3V70x Voltage Regulator Mating Connector" H 6600 3450 60  0001 L CNN "manf#"
	1    6400 3500
	1    0    0    1   
$EndComp
Wire Wire Line
	7200 3400 7600 3400
Connection ~ 7600 3400
$Comp
L HR2:AP2114HA-3.3TRG1_1A;SOT223 VR1
U 1 1 5F4C5F9D
P 6100 4500
F 0 "VR1" H 6350 4650 50  0000 C CNN
F 1 "AP2114HA-3.3TRG1_1A;SOT223" H 6650 4250 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 6300 4550 60  0001 L CNN
F 3 "1A 3.3V LDO Voltage Linear Regulator" H 6300 4350 60  0001 L CNN
F 4 "1A 3.3V LDO Voltage Linear Regulator" H 6300 4250 60  0001 L CNN "Field5"
	1    6100 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 3000 7600 2900
Wire Wire Line
	7200 3000 7600 3000
Connection ~ 7600 3300
Wire Wire Line
	7200 3300 7600 3300
Wire Wire Line
	7600 3400 7600 3300
Connection ~ 7600 3000
Wire Wire Line
	7600 3300 7600 3000
$Comp
L Device:C C2
U 1 1 5F4D49AE
P 5750 4550
F 0 "C2" H 5800 4650 50  0000 L CNN
F 1 "4.7µF;1608" H 5300 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5788 4400 50  0001 C CNN
F 3 "~" H 5750 4550 50  0001 C CNN
	1    5750 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F4D53A8
P 7400 5050
F 0 "C3" H 7250 5150 50  0000 L CNN
F 1 "4.7µF;1608" H 6950 4950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7438 4900 50  0001 C CNN
F 3 "~" H 7400 5050 50  0001 C CNN
	1    7400 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4900 7400 4600
Wire Wire Line
	7400 5200 7400 5500
Wire Wire Line
	7400 5500 6000 5500
Wire Wire Line
	5750 4700 5750 5500
Wire Wire Line
	5750 5500 6000 5500
Connection ~ 6000 5500
Connection ~ 7400 5500
Connection ~ 5750 5500
Wire Wire Line
	5750 5500 5100 5500
Text Label 10000 5500 2    50   ~ 0
GND
$Comp
L Device:CP1 C6
U 1 1 5F4F1B8B
P 9000 3150
F 0 "C6" H 9050 3250 50  0000 L CNN
F 1 "470µF,25Vmin;D10P5H13max" H 7850 3050 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 9000 3150 50  0001 C CNN
F 3 "~" H 9000 3150 50  0001 C CNN
	1    9000 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R5
U 1 1 5F4D39A5
P 2600 6250
F 0 "R5" H 2450 6350 50  0000 L CNN
F 1 "330Ω;1608" H 2650 6150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2640 6240 50  0001 C CNN
F 3 "~" H 2600 6250 50  0001 C CNN
	1    2600 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5F4D441C
P 2600 6900
F 0 "D1" V 2700 7000 50  0000 R CNN
F 1 "LEDGRN;1608" V 2600 6800 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 2600 6900 50  0001 C CNN
F 3 "~" H 2600 6900 50  0001 C CNN
	1    2600 6900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2600 6750 2600 6400
Wire Wire Line
	2600 7050 2600 7150
Text Notes 4500 3000 0    50   ~ 0
Note: When the motors are running full bore\n  up a hill, the current drain becomes significant.\n  Use two USB connectors to maximize current\n  from the USB battery pack.
$Comp
L Device:CP1 C?
U 1 1 5F7AF36B
P 9200 1450
AR Path="/5FA4A874/5F7AF36B" Ref="C?"  Part="1" 
AR Path="/5F52F39E/5F7AF36B" Ref="C4"  Part="1" 
F 0 "C4" H 9250 1550 50  0000 L CNN
F 1 "1000µF@10Vmin;D10P5H13max" H 7950 1350 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 9200 1450 50  0001 C CNN
F 3 "~" H 9200 1450 50  0001 C CNN
	1    9200 1450
	1    0    0    -1  
$EndComp
Text Notes 4600 2350 0    50   ~ 0
Note:\n1. The capacitor is a radial through hole electrolytic with a diameter\n   of 10mm and lead pitch of 5mm.  The capacitance is 1000µF at 16V.\n2. The ideal diode should keep the 5V rail clean while the P5V rail\n   gets abused.\n
Text Notes 7700 4100 0    50   ~ 0
9V for Motors and WOWBus\ngenerated by Pololu Boost regulator.
Text Notes 8300 750  0    50   ~ 0
P5V is a "dirty" 5 volt supply used\nfor lighting LED's, servos, etc.
Text Notes 6200 5000 0    50   ~ 0
3.3V for miscellanious\ndevices that need it.
Wire Wire Line
	6100 4600 6000 4600
Wire Wire Line
	6000 4600 6000 5500
Text Label 3000 4700 0    50   ~ 0
PWR_SW_A
Text Label 4450 4700 0    50   ~ 0
PWR_SW_B
Text Label 2650 4400 0    50   ~ 0
PWR_FET_G
Text Label 4450 5000 0    50   ~ 0
PWR_QBASE
Text Label 2600 6550 0    50   ~ 0
3.3V_LED
$Comp
L Connector:TestPoint TP1
U 1 1 5FB55C4E
P 1700 800
F 0 "TP1" H 1758 918 50  0000 L CNN
F 1 "USB5V" H 1758 827 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 1900 800 50  0001 C CNN
F 3 "~" H 1900 800 50  0001 C CNN
	1    1700 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 900  1700 800 
$Comp
L Jumper:Jumper_3_Open JP2
U 1 1 5FB5A2C2
P 1900 7150
F 0 "JP2" H 2050 7050 50  0000 C CNN
F 1 "LED_EN;M1x3" H 1900 7300 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 1900 7150 50  0001 C CNN
F 3 "~" H 1900 7150 50  0001 C CNN
	1    1900 7150
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R15
U 1 1 5FD17541
P 3400 6250
F 0 "R15" H 3250 6350 50  0000 L CNN
F 1 "470Ω;1608" H 3450 6150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3440 6240 50  0001 C CNN
F 3 "~" H 3400 6250 50  0001 C CNN
	1    3400 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 5FD1788D
P 3400 6900
F 0 "D4" V 3500 7000 50  0000 R CNN
F 1 "LEDGRN;1608" V 3400 6800 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3400 6900 50  0001 C CNN
F 3 "~" H 3400 6900 50  0001 C CNN
	1    3400 6900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3400 6750 3400 6400
Wire Wire Line
	3400 7050 3400 7150
Text Label 3400 6550 0    50   ~ 0
5V_LED
$Comp
L Device:R_US R16
U 1 1 5FD263B0
P 4200 6250
F 0 "R16" H 4050 6350 50  0000 L CNN
F 1 "1KΩ;1608" H 4250 6150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4240 6240 50  0001 C CNN
F 3 "~" H 4200 6250 50  0001 C CNN
	1    4200 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D5
U 1 1 5FD2672C
P 4200 6900
F 0 "D5" V 4300 7000 50  0000 R CNN
F 1 "LEDGRN;1608" V 4200 6800 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 4200 6900 50  0001 C CNN
F 3 "~" H 4200 6900 50  0001 C CNN
	1    4200 6900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4200 6750 4200 6400
Wire Wire Line
	4200 7050 4200 7150
Text Label 4200 6550 0    50   ~ 0
9V_LED
$Comp
L Device:R_US R17
U 1 1 5FD2673A
P 5000 6250
F 0 "R17" H 4850 6350 50  0000 L CNN
F 1 "470Ω;1608" H 5050 6150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5040 6240 50  0001 C CNN
F 3 "~" H 5000 6250 50  0001 C CNN
	1    5000 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D6
U 1 1 5FD26744
P 5000 6900
F 0 "D6" V 5100 7000 50  0000 R CNN
F 1 "LEDGRN;1608" V 5000 6800 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5000 6900 50  0001 C CNN
F 3 "~" H 5000 6900 50  0001 C CNN
	1    5000 6900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5000 6100 5000 6000
Wire Wire Line
	5000 6750 5000 6400
Wire Wire Line
	5000 7050 5000 7150
Text Label 5000 6550 0    50   ~ 0
P5V_LED
Wire Wire Line
	4200 5900 4200 6100
Wire Wire Line
	3400 5800 3400 6100
Wire Wire Line
	2600 5700 2600 6100
Wire Wire Line
	5000 7150 4200 7150
Connection ~ 2600 7150
Wire Wire Line
	2600 7150 2150 7150
Connection ~ 3400 7150
Wire Wire Line
	3400 7150 2600 7150
Connection ~ 4200 7150
Wire Wire Line
	4200 7150 3400 7150
Wire Wire Line
	7600 5500 7400 5500
Wire Wire Line
	7400 4600 7100 4600
Wire Wire Line
	7600 5500 7700 5500
Connection ~ 9600 1200
Connection ~ 9200 1200
Connection ~ 9850 2900
Connection ~ 9000 2900
Wire Wire Line
	8500 2600 7900 2600
Wire Wire Line
	9000 2900 8900 2900
$Comp
L Jumper:Jumper_2_Open JP6
U 1 1 5FBC51EE
P 8700 2600
F 0 "JP6" H 8500 2650 50  0000 C CNN
F 1 "SHUNT;M1x2" H 8700 2500 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8700 2600 50  0001 C CNN
F 3 "~" H 8700 2600 50  0001 C CNN
	1    8700 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 2900 7900 2900
Wire Wire Line
	7900 900  7900 1200
Wire Wire Line
	8550 900  7900 900 
Wire Wire Line
	9200 1200 8900 1200
Wire Wire Line
	9200 900  9200 1200
Wire Wire Line
	8950 900  9200 900 
$Comp
L Jumper:Jumper_2_Open JP5
U 1 1 5FB9F9CC
P 8750 900
F 0 "JP5" H 8550 950 50  0000 C CNN
F 1 "SHUNT;M1x2" H 8750 800 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8750 900 50  0001 C CNN
F 3 "~" H 8750 900 50  0001 C CNN
	1    8750 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 1200 7900 1200
Wire Wire Line
	9850 2900 10000 2900
Wire Wire Line
	9600 1200 9850 1200
Wire Wire Line
	9850 1200 9850 1050
$Comp
L Connector:TestPoint TP8
U 1 1 5FB39289
P 9850 1050
F 0 "TP8" H 9908 1168 50  0000 L CNN
F 1 "P5V" H 9908 1077 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 10050 1050 50  0001 C CNN
F 3 "~" H 10050 1050 50  0001 C CNN
	1    9850 1050
	1    0    0    -1  
$EndComp
Text Notes 3500 7350 0    50   ~ 0
Power On LED's
Text Label 10000 1200 2    50   ~ 0
P5V
Text HLabel 10000 1200 2    50   Output ~ 0
P5V
Text Label 10000 2900 2    50   ~ 0
9V
Text HLabel 10000 5500 2    50   Output ~ 0
GND
Text HLabel 10000 2900 2    50   Output ~ 0
9V
Wire Wire Line
	5000 6000 9600 6000
Wire Wire Line
	9200 1200 9600 1200
Wire Wire Line
	4200 5900 9500 5900
Wire Wire Line
	3400 5800 9400 5800
Wire Wire Line
	2600 5700 9300 5700
Text HLabel 10000 2200 2    50   Output ~ 0
5V
Text Label 10000 2200 2    50   ~ 0
5V
Text HLabel 10000 4900 2    50   Output ~ 0
3.3V
Text Label 10000 4900 2    50   ~ 0
3.3V
$Comp
L Connector:TestPoint TP10
U 1 1 5FB4D981
P 9800 4700
F 0 "TP10" H 9858 4818 50  0000 L CNN
F 1 "3.3V" H 9858 4727 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 10000 4700 50  0001 C CNN
F 3 "~" H 10000 4700 50  0001 C CNN
	1    9800 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 4700 9800 4900
Wire Wire Line
	8000 2200 7900 2200
$Comp
L Jumper:Jumper_2_Open JP3
U 1 1 5FBB6B5B
P 8700 1900
F 0 "JP3" H 8500 1950 50  0000 C CNN
F 1 "SHUNT;M1x2" H 8700 1800 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8700 1900 50  0001 C CNN
F 3 "~" H 8700 1900 50  0001 C CNN
	1    8700 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 1900 9200 1900
Wire Wire Line
	9200 1900 9200 2200
Wire Wire Line
	9200 2200 8900 2200
Wire Wire Line
	8500 1900 7900 1900
Connection ~ 9200 2200
Wire Wire Line
	8000 4900 7900 4900
$Comp
L Jumper:Jumper_2_Open JP4
U 1 1 5FBCAFF7
P 8700 4500
F 0 "JP4" H 8500 4550 50  0000 C CNN
F 1 "SHUNT;M1x2" H 8700 4400 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8700 4500 50  0001 C CNN
F 3 "~" H 8700 4500 50  0001 C CNN
	1    8700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 4500 9000 4500
Wire Wire Line
	9000 4500 9000 4900
Wire Wire Line
	9000 4900 8900 4900
Connection ~ 9000 4900
$Comp
L Connector:TestPoint TP9
U 1 1 5FC189F6
P 9850 2000
F 0 "TP9" H 9908 2118 50  0000 L CNN
F 1 "5V" H 9908 2027 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 10050 2000 50  0001 C CNN
F 3 "~" H 10050 2000 50  0001 C CNN
	1    9850 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 2000 9850 2200
Connection ~ 9850 2200
Wire Wire Line
	9800 4900 10000 4900
Wire Wire Line
	9000 4900 9300 4900
Wire Wire Line
	9850 2200 10000 2200
Wire Wire Line
	9600 1200 9600 6000
Wire Wire Line
	9300 4900 9300 5700
Connection ~ 9300 4900
Wire Wire Line
	9300 4900 9800 4900
Wire Wire Line
	9500 2900 9850 2900
Connection ~ 7900 1900
Wire Wire Line
	7900 1900 7900 2200
Wire Wire Line
	7400 4600 7900 4600
Connection ~ 7400 4600
Connection ~ 7900 4600
Wire Wire Line
	7900 4600 7900 4900
Connection ~ 9800 4900
$Comp
L Connector:TestPoint TP11
U 1 1 5FE99A87
P 9800 5300
F 0 "TP11" H 9858 5418 50  0000 L CNN
F 1 "GND" H 9858 5327 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 10000 5300 50  0001 C CNN
F 3 "~" H 10000 5300 50  0001 C CNN
	1    9800 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 5300 9800 5500
Connection ~ 9800 5500
Wire Wire Line
	9800 5500 10000 5500
$Comp
L Connector:TestPoint TP2
U 1 1 5FEA1D9F
P 7700 5300
F 0 "TP2" H 7758 5418 50  0000 L CNN
F 1 "GND" H 7758 5327 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 7900 5300 50  0001 C CNN
F 3 "~" H 7900 5300 50  0001 C CNN
	1    7700 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 5300 7700 5500
Connection ~ 7700 5500
Wire Wire Line
	7700 5500 8000 5500
$Comp
L Connector:TestPoint TP3
U 1 1 5FEABD3A
P 8000 5300
F 0 "TP3" H 8058 5418 50  0000 L CNN
F 1 "GND" H 8058 5327 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 8200 5300 50  0001 C CNN
F 3 "~" H 8200 5300 50  0001 C CNN
	1    8000 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 5300 8000 5500
Connection ~ 8000 5500
Wire Wire Line
	8000 5500 8300 5500
$Comp
L Connector:TestPoint TP4
U 1 1 5FEB49E8
P 8300 5300
F 0 "TP4" H 8358 5418 50  0000 L CNN
F 1 "GND" H 8358 5327 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 8500 5300 50  0001 C CNN
F 3 "~" H 8500 5300 50  0001 C CNN
	1    8300 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 5300 8300 5500
$Comp
L Connector:TestPoint TP5
U 1 1 5FEB4DE5
P 8600 5300
F 0 "TP5" H 8658 5418 50  0000 L CNN
F 1 "GND" H 8658 5327 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 8800 5300 50  0001 C CNN
F 3 "~" H 8800 5300 50  0001 C CNN
	1    8600 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 5300 8600 5500
Connection ~ 8300 5500
Wire Wire Line
	8300 5500 8600 5500
Connection ~ 8600 5500
Wire Wire Line
	8600 5500 8900 5500
$Comp
L Connector:TestPoint TP6
U 1 1 5FEBDC3D
P 8900 5300
F 0 "TP6" H 8958 5418 50  0000 L CNN
F 1 "GND" H 8958 5327 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 9100 5300 50  0001 C CNN
F 3 "~" H 9100 5300 50  0001 C CNN
	1    8900 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 5300 8900 5500
Connection ~ 8900 5500
Wire Wire Line
	8900 5500 9800 5500
Text Notes 7800 5650 0    50   ~ 0
Extra Ground Test Points
NoConn ~ 1650 7150
Text Notes 1600 7550 0    50   ~ 0
Enable/Disble Power LED's
NoConn ~ 8150 50  
Text Label 7900 900  0    50   ~ 0
SHUNT_P5V
Text Notes 7700 3850 0    50   ~ 0
To measure current, remove 0Ω resistor\nand place a current meter across jumper.
Wire Wire Line
	9200 1300 9200 1200
Wire Wire Line
	9200 1600 9200 1700
Wire Wire Line
	9000 3000 9000 2900
Wire Wire Line
	9000 3300 9000 3400
Wire Wire Line
	9000 3400 7600 3400
Wire Wire Line
	9200 3500 9200 3600
Connection ~ 7600 3600
Wire Wire Line
	7600 3600 7600 3400
Wire Wire Line
	7600 3600 7600 5500
Wire Wire Line
	7600 3600 9200 3600
Wire Wire Line
	9000 2900 9200 2900
Connection ~ 9500 2900
Wire Wire Line
	9500 2900 9500 5900
Wire Wire Line
	9200 3200 9200 2900
Connection ~ 9200 2900
Wire Wire Line
	9200 2900 9500 2900
Wire Wire Line
	9850 2900 9850 2700
$Comp
L Connector:TestPoint TP7
U 1 1 5FB40221
P 9850 2700
F 0 "TP7" H 9908 2818 50  0000 L CNN
F 1 "9V" H 9908 2727 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 10050 2700 50  0001 C CNN
F 3 "~" H 10050 2700 50  0001 C CNN
	1    9850 2700
	1    0    0    -1  
$EndComp
Text Label 7900 1900 0    50   ~ 0
SHUNT_5V
Text Label 7900 4500 0    50   ~ 0
SHUNT_3.3
$Comp
L Device:CP1 C5
U 1 1 5F50C5A8
P 9200 3350
F 0 "C5" H 9250 3450 50  0000 L CNN
F 1 "470µF,25Vmin;D10P5H13max" H 8050 3200 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 9200 3350 50  0001 C CNN
F 3 "~" H 9200 3350 50  0001 C CNN
	1    9200 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 3100 7400 3100
Wire Wire Line
	7400 3100 7400 3200
Wire Wire Line
	7400 3200 7200 3200
Wire Wire Line
	9200 2400 9200 2200
Connection ~ 7400 3200
Wire Wire Line
	6100 4500 6000 4500
Wire Wire Line
	6000 4500 6000 4100
Wire Wire Line
	6000 4100 5750 4100
Wire Wire Line
	5750 4100 5750 4400
$Comp
L HR2:SHUNT R22
U 1 1 5FB51E58
P 8000 1200
F 0 "R22" H 8250 1350 50  0000 C CNN
F 1 "0Ω;1608" H 8450 1050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8200 1250 60  0001 L CNN
F 3 "0 Ohm Power Shunt" H 8200 1050 60  0001 L CNN
F 4 "0 Ohm Power Shunt" H 8200 950 60  0001 L CNN "Field5"
	1    8000 1200
	1    0    0    -1  
$EndComp
$Comp
L HR2:SHUNT R63
U 1 1 5FBD5070
P 8000 2900
F 0 "R63" H 8250 3050 50  0000 C CNN
F 1 "0Ω;1608" H 8450 2750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8200 2950 60  0001 L CNN
F 3 "0 Ohm Power Shunt" H 8200 2750 60  0001 L CNN
F 4 "0 Ohm Power Shunt" H 8200 2650 60  0001 L CNN "Field5"
	1    8000 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2600 7900 2700
Wire Wire Line
	7300 2700 7300 2800
Wire Wire Line
	7200 2800 7300 2800
Text Label 7900 2600 0    50   ~ 0
SHUNT_9V
$Comp
L HR2:SHUNT R20
U 1 1 5FC2832D
P 8000 2200
F 0 "R20" H 8250 2350 50  0000 C CNN
F 1 "0Ω;1608" H 8450 2050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8200 2250 60  0001 L CNN
F 3 "0 Ohm Power Shunt" H 8200 2050 60  0001 L CNN
F 4 "0 Ohm Power Shunt" H 8200 1950 60  0001 L CNN "Field5"
	1    8000 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 4500 7900 4500
Wire Wire Line
	7900 4500 7900 4600
$Comp
L HR2:SHUNT R21
U 1 1 5FCDBB98
P 8000 4900
F 0 "R21" H 8250 5050 50  0000 C CNN
F 1 "0Ω;1608" H 8450 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8200 4950 60  0001 L CNN
F 3 "0 Ohm Power Shunt" H 8200 4750 60  0001 L CNN
F 4 "0 Ohm Power Shunt" H 8200 4650 60  0001 L CNN "Field5"
	1    8000 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 2700 7900 2700
Connection ~ 7300 2700
Connection ~ 7900 2700
Wire Wire Line
	7900 2700 7900 2900
Wire Wire Line
	6000 4100 7400 4100
Wire Wire Line
	7400 3200 7400 4100
Connection ~ 6000 4100
$Comp
L HR2_MISC:Q_PWR_PMOS_GSD Q2
U 1 1 5FCF4323
P 3200 3900
F 0 "Q2" V 3100 3800 50  0000 C CNN
F 1 "PFET_6A;SOT23" V 3450 3900 50  0000 C CNN
F 2 "" H 3400 4000 50  0001 C CNN
F 3 "~" H 3200 3900 50  0001 C CNN
	1    3200 3900
	0    1    -1   0   
$EndComp
Wire Wire Line
	3400 3800 3800 3800
Wire Wire Line
	3200 4100 3200 4200
$Comp
L HR2:USB-C;USB-C CN2
U 1 1 5FCFB047
P 600 3800
F 0 "CN2" H 900 3950 50  0000 C CNN
F 1 "USB-C;USB-C" H 1050 1250 50  0000 C CNN
F 2 "Connector_USB:1USB_C_Receptacle_JAE_DX07S024WJ1R350" H 800 3850 60  0001 L CNN
F 3 "http://www.jae.com/z-en/pdf_download_exec.cfm?param=SJ117219.pdf" H 800 3650 60  0001 L CNN
F 4 "http://www.jae.com/z-en/pdf_download_exec.cfm?param=SJ117219.pdf" H 800 3750 60  0001 L CNN "manf#"
F 5 "USB-C Connector" H 800 3550 60  0001 L CNN "desc"
	1    600  3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 6100 1900 6100
Connection ~ 1900 6100
Wire Wire Line
	1900 6100 1900 7000
Wire Wire Line
	1600 6000 1900 6000
Connection ~ 1900 6000
Wire Wire Line
	1900 6000 1900 6100
Wire Wire Line
	1600 2900 1700 2900
Wire Wire Line
	1700 2900 1700 3000
Wire Wire Line
	1600 3000 1700 3000
Connection ~ 1700 3000
Wire Wire Line
	1700 3000 1700 3100
Wire Wire Line
	1600 3100 1700 3100
Connection ~ 1700 3100
Wire Wire Line
	1600 3200 1700 3200
Wire Wire Line
	1600 5800 1900 5800
Connection ~ 1900 5800
Wire Wire Line
	1900 5800 1900 5900
Wire Wire Line
	1600 5900 1900 5900
Connection ~ 1900 5900
Wire Wire Line
	1900 5900 1900 6000
Wire Wire Line
	1600 4100 1800 4100
Wire Wire Line
	1800 4100 1800 4000
Wire Wire Line
	1700 900  1600 900 
Wire Wire Line
	1600 1000 1700 1000
Connection ~ 1700 1000
Wire Wire Line
	1700 1000 1700 900 
Wire Wire Line
	1600 1100 1700 1100
Connection ~ 1700 1100
Wire Wire Line
	1700 1100 1700 1000
Wire Wire Line
	1600 1200 1700 1200
Wire Wire Line
	1700 1200 1700 1100
Wire Wire Line
	1600 3800 1800 3800
Wire Wire Line
	1600 3900 1800 3900
Connection ~ 1800 3900
Wire Wire Line
	1800 3900 1800 3800
Wire Wire Line
	1600 4000 1800 4000
Connection ~ 1800 4000
Wire Wire Line
	1800 4000 1800 3900
Text Label 2400 5500 0    50   ~ 0
GND
Wire Wire Line
	5100 3600 5100 4100
Wire Wire Line
	2500 3600 2500 3800
Wire Wire Line
	2500 3800 3000 3800
Wire Wire Line
	2500 3600 5100 3600
NoConn ~ 1600 1300
NoConn ~ 1600 1400
NoConn ~ 1600 1500
NoConn ~ 1600 1600
NoConn ~ 1600 1700
NoConn ~ 1600 1800
NoConn ~ 1600 1900
NoConn ~ 1600 2000
NoConn ~ 1600 2100
NoConn ~ 1600 2200
NoConn ~ 1600 2300
NoConn ~ 1600 2400
NoConn ~ 1600 2500
NoConn ~ 1600 2600
NoConn ~ 1600 3300
NoConn ~ 1600 4200
NoConn ~ 1600 4300
NoConn ~ 1600 4400
NoConn ~ 1600 4500
NoConn ~ 1600 4600
NoConn ~ 1600 4700
NoConn ~ 1600 4800
NoConn ~ 1600 4900
NoConn ~ 1600 5000
NoConn ~ 1600 5100
NoConn ~ 1600 5200
NoConn ~ 1600 5300
NoConn ~ 1600 5400
NoConn ~ 1600 5500
NoConn ~ 1600 6200
$Comp
L HR2:USB-C;USB-C,POGND CN1
U 1 1 5FCF242C
P 600 900
F 0 "CN1" H 850 1050 50  0000 C CNN
F 1 "USB-C;USB-C,POGND" H 1100 -1650 50  0000 C CNN
F 2 "Connector_USB:1USB_C_Receptacle_JAE_DX07S024WJ1R350" H 800 950 60  0001 L CNN
F 3 "USB-C Connector" H 800 750 60  0001 L CNN
F 4 "http://www.jae.com/z-en/pdf_download_exec.cfm?param=SJ117219.pdf" H 800 850 60  0001 L CNN "manf#"
F 5 "USB-C Connector" H 800 650 60  0001 L CNN "desc"
	1    600  900 
	1    0    0    -1  
$EndComp
Text Notes 2250 650  0    50   ~ 0
Note: The following URL explains how to get 1.5A@5V from USB-C.\nhttps://www.st.com/resource/en/technical_article/dm00496853-overview-of-usb-type-c-and-power-delivery-technologies-stmicroelectronics.pdf
$Comp
L Device:R_US R72
U 1 1 5FD0DD63
P 2150 2700
F 0 "R72" V 2050 2550 50  0000 L CNN
F 1 "10KΩ;1608" V 2250 2500 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2190 2690 50  0001 C CNN
F 3 "~" H 2150 2700 50  0001 C CNN
	1    2150 2700
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R71
U 1 1 5FD612B2
P 2150 2400
F 0 "R71" V 2050 2250 50  0000 L CNN
F 1 "10KΩ;1608" V 2250 2200 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2190 2390 50  0001 C CNN
F 3 "~" H 2150 2400 50  0001 C CNN
	1    2150 2400
	0    1    1    0   
$EndComp
Wire Wire Line
	1600 2800 1800 2800
Wire Wire Line
	1800 2800 1800 2700
Wire Wire Line
	1800 2700 2000 2700
Wire Wire Line
	1700 2700 1700 2400
Wire Wire Line
	1700 2400 2000 2400
Wire Wire Line
	2300 2400 2400 2400
Wire Wire Line
	2400 2400 2400 2500
Wire Wire Line
	2300 2700 2400 2700
Connection ~ 2400 2700
$Comp
L Device:R_US R70
U 1 1 5FDAC6CA
P 2050 5600
F 0 "R70" V 1950 5450 50  0000 L CNN
F 1 "10KΩ;1608" V 2150 5400 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2090 5590 50  0001 C CNN
F 3 "~" H 2050 5600 50  0001 C CNN
	1    2050 5600
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 3100 1700 3200
Wire Wire Line
	1600 2700 1700 2700
$Comp
L Device:R_US R69
U 1 1 5FDF2974
P 2050 5300
F 0 "R69" V 1950 5150 50  0000 L CNN
F 1 "10KΩ;1608" V 2150 5100 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2090 5290 50  0001 C CNN
F 3 "~" H 2050 5300 50  0001 C CNN
	1    2050 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	1600 5600 1700 5600
Wire Wire Line
	1700 5600 1700 5300
Wire Wire Line
	1700 5300 1900 5300
Wire Wire Line
	1900 5600 1800 5600
Wire Wire Line
	1800 5600 1800 5700
Wire Wire Line
	1800 5700 1600 5700
Wire Wire Line
	2200 5300 2400 5300
Connection ~ 2400 5300
Wire Wire Line
	2400 5300 2400 5500
Wire Wire Line
	2400 5500 2400 5600
Connection ~ 2400 5500
Wire Wire Line
	2400 5800 1900 5800
Wire Wire Line
	2200 5600 2400 5600
Connection ~ 2400 5600
Wire Wire Line
	2400 5600 2400 5800
Wire Wire Line
	2400 5000 2500 5000
Connection ~ 2400 5000
Wire Wire Line
	2400 5000 2400 5300
Wire Wire Line
	2400 5500 4400 5500
$Comp
L HR2:LM5050-1;TSOT-23-6 U20
U 1 1 5FCF6F49
P 3000 1200
F 0 "U20" H 3250 1350 50  0000 C CNN
F 1 "LM5050-1;TSOT-23-6" H 3500 750 50  0000 C CNN
F 2 "Connector_TO_SOT_SMD:TSOT-23-6" H 3200 1250 60  0001 L CNN
F 3 "https://www.ti.com/lit/ds/symlink/lm5050-1.pdf" H 3200 1050 60  0001 L CNN
F 4 "https://www.ti.com/lit/ds/symlink/lm5050-1.pdf" H 3200 1150 60  0001 L CNN "manf#"
F 5 "Idealf Diode w/external NFET" H 3200 950 60  0001 L CNN "desc"
	1    3000 1200
	1    0    0    -1  
$EndComp
$Comp
L HR2_MISC:Q_PWR_NMOS_DSG Q26
U 1 1 5FCF8312
P 4100 1000
F 0 "Q26" V 4050 1150 50  0000 C CNN
F 1 "3A_PWR_NMOS_DSG;SOT23" V 4300 1000 50  0000 C CNN
F 2 "" H 4300 1100 50  0001 C CNN
F 3 "~" H 4100 1000 50  0001 C CNN
	1    4100 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1700 900  2900 900 
Connection ~ 1700 900 
Wire Wire Line
	3000 1300 2900 1300
Wire Wire Line
	2900 1300 2900 1200
Connection ~ 2900 900 
Wire Wire Line
	2900 900  3900 900 
Wire Wire Line
	3000 1200 2900 1200
Connection ~ 2900 1200
Wire Wire Line
	2900 1200 2900 900 
Wire Wire Line
	3000 1400 2800 1400
Connection ~ 2400 2400
Wire Wire Line
	4000 1300 4100 1300
Wire Wire Line
	4100 1300 4100 1200
Wire Wire Line
	4000 1400 4400 1400
Wire Wire Line
	4400 1400 4400 900 
Wire Wire Line
	4400 900  4300 900 
$Comp
L HR2:LM5050-1;TSOT-23-6 U21
U 1 1 5FD748AE
P 3000 2200
F 0 "U21" H 3250 2350 50  0000 C CNN
F 1 "LM5050-1;TSOT-23-6" H 3500 1750 50  0000 C CNN
F 2 "Connector_TO_SOT_SMD:TSOT-23-6" H 3200 2250 60  0001 L CNN
F 3 "https://www.ti.com/lit/ds/symlink/lm5050-1.pdf" H 3200 2050 60  0001 L CNN
F 4 "https://www.ti.com/lit/ds/symlink/lm5050-1.pdf" H 3200 2150 60  0001 L CNN "manf#"
F 5 "Idealf Diode w/external NFET" H 3200 1950 60  0001 L CNN "desc"
	1    3000 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 2300 3000 2300
Connection ~ 1800 3800
Wire Wire Line
	2900 2300 2900 2200
Wire Wire Line
	2900 2200 3000 2200
Wire Wire Line
	2400 2500 3000 2500
Wire Wire Line
	2900 1900 3900 1900
Connection ~ 2900 2200
Wire Wire Line
	4000 2300 4100 2300
Wire Wire Line
	4100 2300 4100 2200
Wire Wire Line
	4000 2400 4400 2400
Wire Wire Line
	4400 2400 4400 1900
Wire Wire Line
	4400 1900 4300 1900
$Comp
L HR2_MISC:Q_PWR_NMOS_DSG Q27
U 1 1 5FD75C02
P 4100 2000
F 0 "Q27" V 4000 2150 50  0000 C CNN
F 1 "3A_NMOS_DSG;SOT23" V 4300 2200 50  0000 C CNN
F 2 "" H 4300 2100 50  0001 C CNN
F 3 "~" H 4100 2000 50  0001 C CNN
	1    4100 2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4400 1900 4400 1400
Connection ~ 4400 1900
Connection ~ 4400 1400
Wire Wire Line
	4400 2400 4400 3800
Connection ~ 4400 2400
Connection ~ 2900 2300
Wire Wire Line
	2900 2200 2900 1900
Wire Wire Line
	2900 3200 2900 2300
Wire Wire Line
	1800 3800 1800 3200
Wire Wire Line
	1800 3200 2900 3200
$Comp
L HR2_MISC:Q_PWR_NMOS_DSG Q28
U 1 1 603044B4
P 6100 1000
F 0 "Q28" V 6050 1150 50  0000 C CNN
F 1 "3A_PWR_NMOS_DSG;SOT23" V 6300 1000 50  0000 C CNN
F 2 "" H 6300 1100 50  0001 C CNN
F 3 "~" H 6100 1000 50  0001 C CNN
	1    6100 1000
	0    -1   -1   0   
$EndComp
$Comp
L HR2:LM5050-1;TSOT-23-6 U1
U 1 1 60304C6A
P 5000 1200
F 0 "U1" H 5250 1350 50  0000 C CNN
F 1 "LM5050-1;TSOT-23-6" H 5500 750 50  0000 C CNN
F 2 "Connector_TO_SOT_SMD:TSOT-23-6" H 5200 1250 60  0001 L CNN
F 3 "https://www.ti.com/lit/ds/symlink/lm5050-1.pdf" H 5200 1050 60  0001 L CNN
F 4 "https://www.ti.com/lit/ds/symlink/lm5050-1.pdf" H 5200 1150 60  0001 L CNN "manf#"
F 5 "Idealf Diode w/external NFET" H 5200 950 60  0001 L CNN "desc"
	1    5000 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 900  4900 900 
Connection ~ 4400 900 
Wire Wire Line
	6000 1300 6100 1300
Wire Wire Line
	6100 1300 6100 1200
Wire Wire Line
	5000 1200 4900 1200
Wire Wire Line
	4900 1200 4900 900 
Connection ~ 4900 900 
Wire Wire Line
	4900 900  5900 900 
Wire Wire Line
	5000 1300 4900 1300
Wire Wire Line
	4900 1300 4900 1200
Connection ~ 4900 1200
Wire Wire Line
	5000 1400 4900 1400
Wire Wire Line
	4900 1400 4900 1500
Wire Wire Line
	4900 1500 5000 1500
Wire Wire Line
	2400 1700 4900 1700
Wire Wire Line
	4900 1700 4900 1500
Connection ~ 2400 1700
Wire Wire Line
	2400 1700 2400 2400
Connection ~ 4900 1500
Text Label 2400 1700 0    50   ~ 0
GND
Wire Wire Line
	4400 1900 7900 1900
Connection ~ 7900 900 
Wire Wire Line
	6300 900  6400 900 
Wire Wire Line
	6400 900  6400 1400
Connection ~ 6400 900 
Wire Wire Line
	6400 900  7900 900 
Wire Wire Line
	6400 1400 6000 1400
Connection ~ 4900 1700
Wire Wire Line
	4900 1700 9200 1700
Wire Wire Line
	1700 2900 2400 2900
Connection ~ 1700 2900
Connection ~ 2400 2900
Wire Wire Line
	2400 2900 2400 2700
Wire Wire Line
	2400 2900 2400 5000
Wire Wire Line
	9200 2200 9400 2200
Wire Wire Line
	9200 2600 9200 2900
Wire Wire Line
	8900 2600 9200 2600
Wire Wire Line
	7400 3100 7400 2400
Wire Wire Line
	7400 2400 9200 2400
Connection ~ 7400 3100
Connection ~ 9850 1200
Wire Wire Line
	9850 1200 10000 1200
Wire Wire Line
	9400 2200 9400 5800
Connection ~ 9400 2200
Wire Wire Line
	9400 2200 9850 2200
Wire Wire Line
	2400 1500 2400 1700
Connection ~ 2400 2500
Wire Wire Line
	2400 2500 2400 2700
Wire Wire Line
	2400 1500 3000 1500
Wire Wire Line
	2800 1400 2800 2400
Wire Wire Line
	2800 2400 3000 2400
Wire Wire Line
	2800 2400 2800 4200
Wire Wire Line
	2800 4200 3200 4200
Connection ~ 2800 2400
Connection ~ 3200 4200
Wire Wire Line
	3200 4200 3200 4400
$EndSCHEMATC
