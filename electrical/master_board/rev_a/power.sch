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
	7200 2600 7300 2600
Wire Wire Line
	7600 2800 7200 2800
NoConn ~ 7200 3400
$Comp
L Device:R_US R1
U 1 1 5F4AAFD8
P 2550 4150
F 0 "R1" H 2400 4250 50  0000 L CNN
F 1 "100KΩ;1608" H 2600 4050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2590 4140 50  0001 C CNN
F 3 "~" H 2550 4150 50  0001 C CNN
	1    2550 4150
	1    0    0    -1  
$EndComp
Text Notes 2900 2700 0    50   ~ 0
Note: Power On/Off circuit is from David Jones EEVBlog #262
$Comp
L Device:Q_NPN_BEC Q1
U 1 1 5F4ABAA0
P 2650 4700
F 0 "Q1" H 2550 4850 50  0000 L CNN
F 1 "2N2222;SOT23" H 2100 4550 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2850 4800 50  0001 C CNN
F 3 "~" H 2650 4700 50  0001 C CNN
	1    2650 4700
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
	2550 3800 2550 4000
Wire Wire Line
	2550 4300 2550 4400
Wire Wire Line
	2550 4400 3200 4400
Wire Wire Line
	2550 4500 2550 4400
Connection ~ 2550 4400
Wire Wire Line
	3800 4100 3800 3800
Wire Wire Line
	2850 4700 3800 4700
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
Wire Wire Line
	2550 2800 5100 2800
Wire Wire Line
	5100 2800 5100 4100
Connection ~ 2550 3800
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
	2550 4900 2550 5500
Wire Wire Line
	2550 5500 4400 5500
Connection ~ 4400 5500
Connection ~ 4400 3800
Wire Wire Line
	4400 5100 4400 5000
Connection ~ 4400 5000
Wire Wire Line
	1900 3800 2550 3800
Connection ~ 2550 5500
Connection ~ 5100 5500
Connection ~ 7600 5500
$Comp
L HR2:U3V70xMATE;F1x4+F1x5 CN?
U 1 1 5F5303B1
P 6400 3400
AR Path="/5F5303B1" Ref="CN?"  Part="1" 
AR Path="/5F52F39E/5F5303B1" Ref="CN93"  Part="1" 
F 0 "CN93" H 6700 2450 50  0000 C CNN
F 1 "POLOLU_U3V70F9;F1x4+F1x5" H 6800 3550 50  0000 C CNN
F 2 "HR2:U3V70xMATE_F1x4+F1x5" H 6600 3450 60  0001 L CNN
F 3 "Pololu U3V70x Voltage Regulator Mating Connector" H 6600 3250 60  0001 L CNN
F 4 "Pololu U3V70x Voltage Regulator Mating Connector" H 6600 3350 60  0001 L CNN "manf#"
	1    6400 3400
	1    0    0    1   
$EndComp
Wire Wire Line
	7200 3300 7600 3300
Connection ~ 7600 3300
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
	7600 2900 7600 2800
Wire Wire Line
	7200 2900 7600 2900
Connection ~ 7600 3200
Wire Wire Line
	7200 3200 7600 3200
Wire Wire Line
	7600 3300 7600 3200
Connection ~ 7600 2900
Wire Wire Line
	7600 3200 7600 2900
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
Text Label 9900 5500 2    50   ~ 0
GND
Text Label 2200 3800 0    50   ~ 0
USB5V
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
Wire Wire Line
	3000 3800 2550 3800
Text Notes 2600 3400 0    50   ~ 0
Note: The PFET's in the KiCAD library have passive pins\n  for design rule checks.  A PFET with Power In on the\n  Source and Power Out on the Drain is needed.\n  It was easier to do an ugly box than a pretty PFET.\n  Deal with it.
Wire Wire Line
	1900 3200 2100 3200
Wire Wire Line
	2100 3200 2100 3300
Wire Wire Line
	1900 3300 2100 3300
Connection ~ 2100 3300
Wire Wire Line
	2100 3300 2100 4200
Wire Wire Line
	1900 2800 2550 2800
Connection ~ 2550 2800
Text Notes 1450 2350 0    50   ~ 0
Note: When the motors are running full bore\n  up a hill, the current drain becomes significant.\n  Use two USB connectors to maximize current\n  from the USB battery pack.
$Comp
L Device:CP1 C?
U 1 1 5F7AF36B
P 9300 2150
AR Path="/5FA4A874/5F7AF36B" Ref="C?"  Part="1" 
AR Path="/5F52F39E/5F7AF36B" Ref="C4"  Part="1" 
F 0 "C4" H 9350 2250 50  0000 L CNN
F 1 "1000µF@10Vmin;D10P5H13max" H 8050 2050 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 9300 2150 50  0001 C CNN
F 3 "~" H 9300 2150 50  0001 C CNN
	1    9300 2150
	1    0    0    -1  
$EndComp
$Comp
L HR2:LM66100;SOT363 U1
U 1 1 5F7B1947
P 6400 1400
F 0 "U1" H 6650 1550 50  0000 C CNN
F 1 "LM66100;SOT363" H 6750 750 60  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 6600 1450 60  0001 L CNN
F 3 "1.5A Ideal Diode" H 6600 1250 60  0001 L CNN
F 4 "1.5A Ideal Diod" H 6600 1150 60  0001 L CNN "Field5"
	1    6400 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1500 7600 1500
Connection ~ 7600 2800
Wire Wire Line
	7500 1400 7100 1400
Wire Wire Line
	7100 1900 7200 1900
Connection ~ 7600 2400
Wire Wire Line
	7600 2400 7600 2800
Wire Wire Line
	7100 1600 7200 1600
Wire Wire Line
	7200 1600 7200 1900
Connection ~ 7200 1900
Wire Wire Line
	7100 1800 7500 1800
Wire Wire Line
	7500 1800 7500 1400
Text Notes 3750 1800 0    50   ~ 0
Note:\n1. The capacitor is a radial through hole electrolytic with a diameter\n   of 10mm and lead pitch of 5mm.  The capacitance is 1000µF at 16V.\n2. The ideal diode should keep the 5V rail clean while the P5V rail\n   gets abused.\n
Text Notes 5150 3050 0    50   ~ 0
9V for Motors and WOWBus\ngenerated by Pololu Boost regulator.
Text Notes 8400 1350 0    50   ~ 0
P5V is a "dirty" 5 volt supply used\nfor lighting LED's etc.
Text Notes 6200 5000 0    50   ~ 0
3.3V for miscellanious\ndevices that need it.
Wire Wire Line
	1900 4200 2100 4200
Wire Wire Line
	2100 4300 2100 4200
Connection ~ 2100 4200
Wire Wire Line
	1900 4300 2100 4300
Wire Wire Line
	2100 4300 2100 5500
Connection ~ 2100 4300
Wire Wire Line
	2100 5500 2550 5500
$Comp
L HR2:USB_B_MICRO_POWER_SLAVE CN1
U 1 1 5F9D0CA6
P 1100 2800
F 0 "CN1" H 1350 2950 50  0000 C CNN
F 1 "USB3140-30-230-1-C;VERT,2.3FLANGE" H 1300 2150 50  0000 C CNN
F 2 "HR2:USB3140-30-230-1-C;VERT,2.3FLANGE" H 1300 2850 60  0001 L CNN
F 3 "2.3FLANGE" H 1300 2650 60  0001 L CNN
F 4 "USB_Mini Power (Slave)" H 1300 2550 60  0001 L CNN "Field5"
	1    1100 2800
	1    0    0    -1  
$EndComp
$Comp
L HR2:USB_B_MICRO_POWER_MASTER CN2
U 1 1 5F9D14A3
P 1100 3800
F 0 "CN2" H 1350 3950 50  0000 C CNN
F 1 "USB3140-30-230-1-C;VERT,2.3FLANGE" H 1300 3150 50  0000 C CNN
F 2 "HR2:USB3140-30-230-1-C;VERT,2.3FLANGE" H 1300 3850 60  0001 L CNN
F 3 "2.3FLANGE" H 1300 3650 60  0001 L CNN
F 4 "USB_Mini Power (Slave)" H 1300 3550 60  0001 L CNN "Field5"
	1    1100 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4600 6000 4600
Wire Wire Line
	6000 4600 6000 5500
Text Notes 750  1300 0    80   ~ 0
TO DO:\n1. Do power supply review with somebody.
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
P 2550 2700
F 0 "TP1" H 2608 2818 50  0000 L CNN
F 1 "USB5V" H 2608 2727 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 2750 2700 50  0001 C CNN
F 3 "~" H 2750 2700 50  0001 C CNN
	1    2550 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2800 2550 2700
$Comp
L Jumper:Jumper_3_Open JP2
U 1 1 5FB5A2C2
P 2100 7150
F 0 "JP2" H 2250 7050 50  0000 C CNN
F 1 "LED_EN;M1x3" H 2100 7300 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2100 7150 50  0001 C CNN
F 3 "~" H 2100 7150 50  0001 C CNN
	1    2100 7150
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
	2600 7150 2350 7150
Connection ~ 3400 7150
Wire Wire Line
	3400 7150 2600 7150
Connection ~ 4200 7150
Wire Wire Line
	4200 7150 3400 7150
Wire Wire Line
	2100 5500 2100 7000
Connection ~ 2100 5500
Wire Wire Line
	7600 5500 7400 5500
Wire Wire Line
	7400 4600 7100 4600
Wire Wire Line
	7600 5500 7700 5500
Connection ~ 9700 1900
Connection ~ 9300 1900
Connection ~ 7900 1900
Connection ~ 9650 2900
Connection ~ 9000 2900
Wire Wire Line
	8500 2500 7900 2500
Wire Wire Line
	9000 2900 8900 2900
Wire Wire Line
	9000 2500 9000 2900
Wire Wire Line
	8900 2500 9000 2500
$Comp
L Jumper:Jumper_2_Open JP6
U 1 1 5FBC51EE
P 8700 2500
F 0 "JP6" H 8500 2550 50  0000 C CNN
F 1 "SHUNT;M1x2" H 8700 2400 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8700 2500 50  0001 C CNN
F 3 "~" H 8700 2500 50  0001 C CNN
	1    8700 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 2900 7900 2900
Wire Wire Line
	7900 1500 7900 1900
Wire Wire Line
	8550 1500 7900 1500
Wire Wire Line
	9300 1900 8900 1900
Wire Wire Line
	9300 1500 9300 1900
Wire Wire Line
	8950 1500 9300 1500
$Comp
L Jumper:Jumper_2_Open JP5
U 1 1 5FB9F9CC
P 8750 1500
F 0 "JP5" H 8550 1550 50  0000 C CNN
F 1 "SHUNT;M1x2" H 8750 1400 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8750 1500 50  0001 C CNN
F 3 "~" H 8750 1500 50  0001 C CNN
	1    8750 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 1900 7900 1900
Wire Wire Line
	9650 2900 9950 2900
Wire Wire Line
	9700 1900 9950 1900
Wire Wire Line
	9700 1900 9700 1800
$Comp
L Connector:TestPoint TP8
U 1 1 5FB39289
P 9700 1800
F 0 "TP8" H 9758 1918 50  0000 L CNN
F 1 "P5V" H 9758 1827 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 9900 1800 50  0001 C CNN
F 3 "~" H 9900 1800 50  0001 C CNN
	1    9700 1800
	1    0    0    -1  
$EndComp
Text Notes 3500 7350 0    50   ~ 0
Power On LED's
Text Label 9950 1900 2    50   ~ 0
P5V
Text HLabel 9950 1900 2    50   Output ~ 0
P5V
Text Label 9950 2900 2    50   ~ 0
9V
Text HLabel 9900 5500 2    50   Output ~ 0
GND
Text HLabel 9950 2900 2    50   Output ~ 0
9V
Wire Wire Line
	5000 6000 9500 6000
Wire Wire Line
	9300 1900 9500 1900
Wire Wire Line
	4200 5900 9400 5900
Wire Wire Line
	3400 5800 9300 5800
Wire Wire Line
	2600 5700 9200 5700
Text HLabel 9900 4100 2    50   Output ~ 0
5V
Text Label 9900 4100 2    50   ~ 0
5V
Text HLabel 9900 4900 2    50   Output ~ 0
3.3V
Text Label 9900 4900 2    50   ~ 0
3.3V
$Comp
L Connector:TestPoint TP10
U 1 1 5FB4D981
P 9700 4500
F 0 "TP10" H 9758 4618 50  0000 L CNN
F 1 "3.3V" H 9758 4527 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 9900 4500 50  0001 C CNN
F 3 "~" H 9900 4500 50  0001 C CNN
	1    9700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 4500 9700 4900
Wire Wire Line
	8000 4100 7900 4100
$Comp
L Jumper:Jumper_2_Open JP3
U 1 1 5FBB6B5B
P 8700 3800
F 0 "JP3" H 8500 3850 50  0000 C CNN
F 1 "SHUNT;M1x2" H 8700 3700 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8700 3800 50  0001 C CNN
F 3 "~" H 8700 3800 50  0001 C CNN
	1    8700 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 3800 9000 3800
Wire Wire Line
	9000 3800 9000 4100
Wire Wire Line
	9000 4100 8900 4100
Wire Wire Line
	8500 3800 7900 3800
Connection ~ 9000 4100
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
Wire Wire Line
	9000 4100 9300 4100
$Comp
L Connector:TestPoint TP9
U 1 1 5FC189F6
P 9700 3900
F 0 "TP9" H 9758 4018 50  0000 L CNN
F 1 "5V" H 9758 3927 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 9900 3900 50  0001 C CNN
F 3 "~" H 9900 3900 50  0001 C CNN
	1    9700 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3900 9700 4100
Connection ~ 9700 4100
Wire Wire Line
	9700 4900 9900 4900
Wire Wire Line
	9000 4900 9200 4900
Wire Wire Line
	9700 4100 9900 4100
Wire Wire Line
	9500 1900 9500 6000
Wire Wire Line
	9300 4100 9300 5800
Connection ~ 9300 4100
Wire Wire Line
	9300 4100 9700 4100
Wire Wire Line
	9200 4900 9200 5700
Connection ~ 9200 4900
Wire Wire Line
	9200 4900 9700 4900
Connection ~ 9500 1900
Wire Wire Line
	9500 1900 9700 1900
Wire Wire Line
	9400 2900 9650 2900
Wire Wire Line
	7500 3800 7900 3800
Connection ~ 7500 3800
Connection ~ 7900 3800
Wire Wire Line
	7900 3800 7900 4100
Wire Wire Line
	7400 4600 7900 4600
Connection ~ 7400 4600
Connection ~ 7900 4600
Wire Wire Line
	7900 4600 7900 4900
Connection ~ 9700 4900
$Comp
L Connector:TestPoint TP11
U 1 1 5FE99A87
P 9700 5300
F 0 "TP11" H 9758 5418 50  0000 L CNN
F 1 "GND" H 9758 5327 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 9900 5300 50  0001 C CNN
F 3 "~" H 9900 5300 50  0001 C CNN
	1    9700 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 5300 9700 5500
Connection ~ 9700 5500
Wire Wire Line
	9700 5500 9900 5500
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
	8900 5500 9700 5500
Text Notes 7800 5650 0    50   ~ 0
Extra Ground Test Points
NoConn ~ 1850 7150
Text Notes 1600 7550 0    50   ~ 0
Enable/Disble Power LED's
NoConn ~ 8150 50  
Text Label 7900 1500 0    50   ~ 0
SHUNT_P5V
Text Notes 8300 900  0    50   ~ 0
To measure current, remove 0Ω resistor\nand place a current meter across jumper.
Wire Wire Line
	9300 2000 9300 1900
Wire Wire Line
	7200 1900 7900 1900
Wire Wire Line
	9300 2300 9300 2400
Wire Wire Line
	7600 2400 9300 2400
Wire Wire Line
	9000 3000 9000 2900
Wire Wire Line
	9000 3300 9000 3400
Wire Wire Line
	9000 3400 7600 3400
Connection ~ 7600 3400
Wire Wire Line
	7600 3400 7600 3300
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
Connection ~ 9400 2900
Wire Wire Line
	9400 2900 9400 5900
Wire Wire Line
	9200 3200 9200 2900
Connection ~ 9200 2900
Wire Wire Line
	9200 2900 9400 2900
Wire Wire Line
	9650 2900 9650 2700
$Comp
L Connector:TestPoint TP7
U 1 1 5FB40221
P 9650 2700
F 0 "TP7" H 9708 2818 50  0000 L CNN
F 1 "9V" H 9708 2727 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 9850 2700 50  0001 C CNN
F 3 "~" H 9850 2700 50  0001 C CNN
	1    9650 2700
	1    0    0    -1  
$EndComp
Text Label 7900 3800 0    50   ~ 0
SHUNT_5V
Text Label 7900 4500 0    50   ~ 0
SHUNT_3.3
$Comp
L Device:CP1 C5
U 1 1 5F50C5A8
P 9200 3350
F 0 "C5" H 9250 3450 50  0000 L CNN
F 1 "470µF,25Vmin;D10P5H13max" H 8050 3250 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 9200 3350 50  0001 C CNN
F 3 "~" H 9200 3350 50  0001 C CNN
	1    9200 3350
	1    0    0    -1  
$EndComp
Connection ~ 7500 1800
Wire Wire Line
	7500 1800 7500 3800
Wire Wire Line
	7200 3000 7400 3000
Wire Wire Line
	7400 3000 7400 3100
Wire Wire Line
	7400 3100 7200 3100
Wire Wire Line
	7400 4300 9000 4300
Wire Wire Line
	9000 4300 9000 4100
Connection ~ 7400 3100
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
P 8000 1900
F 0 "R22" H 8250 2050 50  0000 C CNN
F 1 "0Ω;1608" H 8450 1750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8200 1950 60  0001 L CNN
F 3 "0 Ohm Power Shunt" H 8200 1750 60  0001 L CNN
F 4 "0 Ohm Power Shunt" H 8200 1650 60  0001 L CNN "Field5"
	1    8000 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 1500 7600 2400
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
	7900 2500 7900 2600
Wire Wire Line
	7300 2600 7300 2700
Wire Wire Line
	7200 2700 7300 2700
Text Label 7900 2500 0    50   ~ 0
SHUNT_9V
$Comp
L HR2:SHUNT R20
U 1 1 5FC2832D
P 8000 4100
F 0 "R20" H 8250 4250 50  0000 C CNN
F 1 "0Ω;1608" H 8450 3950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8200 4150 60  0001 L CNN
F 3 "0 Ohm Power Shunt" H 8200 3950 60  0001 L CNN
F 4 "0 Ohm Power Shunt" H 8200 3850 60  0001 L CNN "Field5"
	1    8000 4100
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
	7300 2600 7900 2600
Connection ~ 7300 2600
Connection ~ 7900 2600
Wire Wire Line
	7900 2600 7900 2900
Wire Wire Line
	4400 3800 7500 3800
Wire Wire Line
	6000 4100 7400 4100
Wire Wire Line
	7400 3100 7400 4100
Connection ~ 6000 4100
Connection ~ 7400 4100
Wire Wire Line
	7400 4100 7400 4300
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
	2550 2800 2550 3800
Wire Wire Line
	3200 4100 3200 4400
$EndSCHEMATC
