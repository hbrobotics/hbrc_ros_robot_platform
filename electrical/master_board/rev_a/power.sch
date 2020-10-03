EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 3 8
Title "HR2 Master Board"
Date "2020-10-03"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 8750 3200 2    50   Output ~ 0
9V
Text HLabel 8750 4300 2    50   Output ~ 0
5V
Wire Wire Line
	7600 3100 8200 3100
Text HLabel 8750 6000 2    50   Output ~ 0
GND
Wire Wire Line
	8000 3300 7600 3300
NoConn ~ 7600 3900
Wire Wire Line
	8000 6000 8000 4200
$Comp
L Device:R_US R1
U 1 1 5F4AAFD8
P 2950 4650
F 0 "R1" H 2800 4750 50  0000 L CNN
F 1 "100KΩ;1608" H 3000 4550 50  0000 L CNN
F 2 "" V 2990 4640 50  0001 C CNN
F 3 "~" H 2950 4650 50  0001 C CNN
	1    2950 4650
	1    0    0    -1  
$EndComp
Text Notes 2000 6600 0    50   ~ 0
Note: Power On/Off circuit is from David  Jones EEVBlog #262
$Comp
L Device:Q_NPN_BEC Q1
U 1 1 5F4ABAA0
P 3050 5200
F 0 "Q1" H 2950 5350 50  0000 L CNN
F 1 "2N2222;SOT23" H 2500 5050 50  0000 L CNN
F 2 "" H 3250 5300 50  0001 C CNN
F 3 "~" H 3050 5200 50  0001 C CNN
	1    3050 5200
	-1   0    0    -1  
$EndComp
Text Notes 3050 5900 0    50   ~ 0
Note: The bipolar transtistors are generic\n  small signal NPN SOT23 in a BEC (i.e\n  pin1 = Base, pin 2 = Emitter,\n  pi 3 = Collector) configuration.\n  Pretty much any NPN will do, so\n  substitutions should be pretty safe.
$Comp
L Device:Q_NPN_BEC Q3
U 1 1 5F4AC33E
P 5200 5500
F 0 "Q3" H 5390 5546 50  0000 L CNN
F 1 "2N2222;SOT23" H 5390 5455 50  0000 L CNN
F 2 "" H 5400 5600 50  0001 C CNN
F 3 "~" H 5200 5500 50  0001 C CNN
	1    5200 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 5F4AC83B
P 4200 4750
F 0 "R2" H 4050 4850 50  0000 L CNN
F 1 "100KΩ;1608" H 4250 4650 50  0000 L CNN
F 2 "" V 4240 4740 50  0001 C CNN
F 3 "~" H 4200 4750 50  0001 C CNN
	1    4200 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R4
U 1 1 5F4ACD3F
P 5300 4750
F 0 "R4" H 5150 4850 50  0000 L CNN
F 1 "100KΩ;1608" H 5350 4650 50  0000 L CNN
F 2 "" V 5340 4740 50  0001 C CNN
F 3 "~" H 5300 4750 50  0001 C CNN
	1    5300 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R3
U 1 1 5F4ACF72
P 4800 4750
F 0 "R3" H 4650 4850 50  0000 L CNN
F 1 "1MΩ;1608" H 4850 4650 50  0000 L CNN
F 2 "" V 4840 4740 50  0001 C CNN
F 3 "~" H 4800 4750 50  0001 C CNN
	1    4800 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 4300 2950 4500
Wire Wire Line
	2950 4800 2950 4900
Wire Wire Line
	2950 4900 3600 4900
Wire Wire Line
	3600 4900 3600 4800
Wire Wire Line
	2950 5000 2950 4900
Connection ~ 2950 4900
Wire Wire Line
	4200 4600 4200 4300
Wire Wire Line
	3250 5200 4200 5200
Wire Wire Line
	4200 5200 4200 4900
$Comp
L Switch:SW_Push SW1
U 1 1 5F4B612A
P 4500 5200
F 0 "SW1" H 4350 5300 50  0000 C CNN
F 1 "PTS645SK50SMTR92 LFS;6x9.5" H 4150 5150 50  0000 C CNN
F 2 "" H 4500 5400 50  0001 C CNN
F 3 "~" H 4500 5400 50  0001 C CNN
	1    4500 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 5200 4300 5200
Connection ~ 4200 5200
Wire Wire Line
	2950 3300 5300 3300
Wire Wire Line
	5300 3300 5300 4600
Connection ~ 2950 4300
Wire Wire Line
	4200 4300 4800 4300
Wire Wire Line
	4800 4300 4800 4600
Connection ~ 4200 4300
Wire Wire Line
	4700 5200 5300 5200
Wire Wire Line
	5300 5200 5300 5300
Wire Wire Line
	5300 5200 5300 4900
Connection ~ 5300 5200
Wire Wire Line
	5000 5500 4800 5500
Wire Wire Line
	4800 5500 4800 4900
$Comp
L Device:C C1
U 1 1 5F4BF513
P 4800 5750
F 0 "C1" H 4650 5850 50  0000 L CNN
F 1 "22µF;1608" H 4850 5650 50  0000 L CNN
F 2 "" H 4838 5600 50  0001 C CNN
F 3 "~" H 4800 5750 50  0001 C CNN
	1    4800 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 5900 4800 6000
Wire Wire Line
	4800 6000 5300 6000
Wire Wire Line
	5300 6000 5300 5700
Wire Wire Line
	2950 5400 2950 6000
Wire Wire Line
	2950 6000 4800 6000
Connection ~ 4800 6000
Connection ~ 4800 4300
Wire Wire Line
	4800 5600 4800 5500
Connection ~ 4800 5500
$Comp
L Connector:USB_B_Mini CN1
U 1 1 5F4B0748
P 2000 4500
F 0 "CN1" H 1900 4850 50  0000 C CNN
F 1 "AMP10103594-0001LF;USB1x6H" H 1200 4150 50  0000 C CNN
F 2 "" H 2150 4450 50  0001 C CNN
F 3 "~" H 2150 4450 50  0001 C CNN
	1    2000 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 4300 2950 4300
NoConn ~ 2300 4500
NoConn ~ 2300 4600
NoConn ~ 2300 4700
Wire Wire Line
	1900 4900 1900 5000
Wire Wire Line
	1900 5000 2000 5000
Wire Wire Line
	2000 5000 2000 4900
Wire Wire Line
	1900 5000 1900 6000
Wire Wire Line
	1900 6000 2950 6000
Connection ~ 1900 5000
Connection ~ 2950 6000
Wire Wire Line
	8750 6000 8300 6000
Connection ~ 5300 6000
Wire Wire Line
	7800 3500 7600 3500
Connection ~ 8000 6000
Wire Wire Line
	8000 6000 7800 6000
$Comp
L HR2:U3V70xMATE;F1x4+F1x5 CN?
U 1 1 5F5303B1
P 6800 3900
AR Path="/5F5303B1" Ref="CN?"  Part="1" 
AR Path="/5F52F39E/5F5303B1" Ref="CN42"  Part="1" 
F 0 "CN42" H 7100 2950 50  0000 C CNN
F 1 "POLOLU_U3V70F9;F1x4+F1x5" H 7200 4050 50  0000 C CNN
F 2 "HR2:U3V70xMATE_F1x4+F1x5" H 7000 3950 60  0001 L CNN
F 3 "Pololu U3V70x Voltage Regulator Mating Connector" H 7000 3750 60  0001 L CNN
F 4 "Pololu U3V70x Voltage Regulator Mating Connector" H 7000 3850 60  0001 L CNN "manf#"
	1    6800 3900
	1    0    0    1   
$EndComp
Wire Wire Line
	7600 3800 8000 3800
Connection ~ 8000 3800
Wire Wire Line
	7600 3200 8200 3200
Wire Wire Line
	7600 3600 7800 3600
Connection ~ 7800 3600
Wire Wire Line
	7800 3600 7800 3500
Wire Wire Line
	4800 4300 6150 4300
Wire Wire Line
	7800 3600 7800 4300
$Comp
L HR2:AP2114HA-3.3TRG1_1A;SOT223 VR1
U 1 1 5F4C5F9D
P 6300 4750
F 0 "VR1" H 6550 4900 50  0000 C CNN
F 1 "AP2114HA-3.3TRG1_1A;SOT223" H 7350 4900 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT_223-3_TabPin2" H 6500 4800 60  0001 L CNN
F 3 "1A 3.3V LDO Voltage Linear Regulator" H 6500 4600 60  0001 L CNN
F 4 "1A 3.3V LDO Voltage Linear Regulator" H 6500 4500 60  0001 L CNN "Field5"
	1    6300 4750
	1    0    0    -1  
$EndComp
Connection ~ 7800 4300
Wire Wire Line
	8200 3100 8200 3200
Wire Wire Line
	8000 3400 8000 3300
Wire Wire Line
	7600 3400 8000 3400
Connection ~ 8000 3700
Wire Wire Line
	7600 3700 8000 3700
Wire Wire Line
	8000 3800 8000 3700
Connection ~ 8000 3400
Wire Wire Line
	8000 3700 8000 3400
$Comp
L Device:C C4
U 1 1 5F4D49AE
P 6150 5000
F 0 "C4" H 6000 5100 50  0000 L CNN
F 1 "4.7µF;1608" H 5700 4900 50  0000 L CNN
F 2 "" H 6188 4850 50  0001 C CNN
F 3 "~" H 6150 5000 50  0001 C CNN
	1    6150 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5F4D53A8
P 7800 5050
F 0 "C5" H 7650 5150 50  0000 L CNN
F 1 "4.7µF;1608" H 7350 4950 50  0000 L CNN
F 2 "" H 7838 4900 50  0001 C CNN
F 3 "~" H 7800 5050 50  0001 C CNN
	1    7800 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4850 6150 4750
Connection ~ 6150 4300
Wire Wire Line
	6150 4300 7800 4300
Wire Wire Line
	6300 4750 6150 4750
Connection ~ 6150 4750
Wire Wire Line
	6150 4750 6150 4300
Wire Wire Line
	7800 4900 7800 4750
Wire Wire Line
	7800 4750 7500 4750
Wire Wire Line
	7800 5200 7800 6000
Wire Wire Line
	7800 6000 6900 6000
Wire Wire Line
	6900 6000 6900 5350
Wire Wire Line
	6150 5150 6150 6000
Wire Wire Line
	6150 6000 6900 6000
Connection ~ 6900 6000
Connection ~ 7800 6000
Connection ~ 6150 6000
Wire Wire Line
	6150 6000 5300 6000
Text Label 8600 3200 0    50   ~ 0
9V
Text Label 8600 4300 0    50   ~ 0
5V
Text Label 8550 6000 0    50   ~ 0
GND
Text HLabel 8750 4750 2    50   Output ~ 0
3.3V
Wire Wire Line
	7800 4750 8300 4750
Connection ~ 7800 4750
Text Label 8550 4750 0    50   ~ 0
3.3V
Text Label 2600 4300 0    50   ~ 0
USB5V
$Comp
L Device:CP1 C7
U 1 1 5F4F1B8B
P 8400 3450
F 0 "C7" H 8515 3496 50  0000 L CNN
F 1 "470µF,25V;D8_P3.5" H 8450 3300 50  0000 L CNN
F 2 "" H 8400 3450 50  0001 C CNN
F 3 "~" H 8400 3450 50  0001 C CNN
	1    8400 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 3600 8400 3700
Wire Wire Line
	8400 3700 8000 3700
Wire Wire Line
	8400 3300 8400 3200
Wire Wire Line
	8400 3200 8750 3200
Wire Wire Line
	8200 4100 8200 4200
Wire Wire Line
	8200 4200 8000 4200
Connection ~ 8000 4200
Wire Wire Line
	8000 4200 8000 3800
Wire Wire Line
	8200 3800 8200 3200
Connection ~ 8200 3200
$Comp
L Device:CP1 C6
U 1 1 5F50C5A8
P 8200 3950
F 0 "C6" H 8315 3996 50  0000 L CNN
F 1 "470µF,25V;D8_P3.5" H 8250 3800 50  0000 L CNN
F 2 "" H 8200 3950 50  0001 C CNN
F 3 "~" H 8200 3950 50  0001 C CNN
	1    8200 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3200 8400 3200
Connection ~ 8400 3200
$Comp
L Device:R_US R13
U 1 1 5F4D39A5
P 8300 5100
F 0 "R13" H 8150 5200 50  0000 L CNN
F 1 "470Ω;1608" H 8350 5000 50  0000 L CNN
F 2 "" V 8340 5090 50  0001 C CNN
F 3 "~" H 8300 5100 50  0001 C CNN
	1    8300 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5F4D441C
P 8300 5600
F 0 "D1" V 8400 5700 50  0000 R CNN
F 1 "LEDGRN;1608" V 8300 5500 50  0000 R CNN
F 2 "" H 8300 5600 50  0001 C CNN
F 3 "~" H 8300 5600 50  0001 C CNN
	1    8300 5600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8300 4950 8300 4750
Connection ~ 8300 4750
Wire Wire Line
	8300 4750 8750 4750
Wire Wire Line
	8300 5450 8300 5250
Wire Wire Line
	8300 5750 8300 6000
Connection ~ 8300 6000
Wire Wire Line
	8300 6000 8000 6000
$Comp
L HR2:POWER_PFET_GSD Q2
U 1 1 5F56BDDA
P 3100 4300
F 0 "Q2" H 3350 4450 50  0000 C CNN
F 1 "PFET_6A_GSD;SOT23" H 4050 4450 50  0000 C CNN
F 2 "HR2:" H 3300 4350 60  0001 L CNN
F 3 "Power PFG (GSD pinout)" H 3300 4150 60  0001 L CNN
F 4 "Power PFG (GSD pinout)" H 3300 4250 60  0001 L CNN "manf#"
F 5 "Power PFET (GSD pinout)" H 3300 4050 60  0001 L CNN "Field5"
	1    3100 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 4300 2950 4300
Wire Wire Line
	2950 4300 2950 3300
Wire Wire Line
	4100 4300 4200 4300
Wire Wire Line
	7800 4300 8750 4300
Text Notes 3000 3900 0    50   ~ 0
Note: The PFET's in the KiCAD library have passive pins\n  for design rule checks.  A PFET with Power In on the\n  Source and Power Out on the Drain is needed.\n  It was easier to do an ugly box than a pretty PFET.\n  Deal with it.
$Comp
L HR2:USB_B_MINI_POWER_SLAVE CN2
U 1 1 5F56C48C
P 1600 3300
F 0 "CN2" H 1850 3450 50  0000 C CNN
F 1 "AMP10103594-0001LF;USB1x6H" H 1200 2650 50  0000 C CNN
F 2 "Connector:USB_B_Mini" H 1800 3350 60  0001 L CNN
F 3 "USB_Mini Power (Slave)" H 1800 3150 60  0001 L CNN
F 4 "USB_Mini Power (Slave)" H 1800 3050 60  0001 L CNN "Field5"
	1    1600 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 3700 2500 3700
Wire Wire Line
	2500 3700 2500 3800
Wire Wire Line
	2500 5000 2000 5000
Connection ~ 2000 5000
Wire Wire Line
	2400 3800 2500 3800
Connection ~ 2500 3800
Wire Wire Line
	2500 3800 2500 5000
Wire Wire Line
	2400 3300 2950 3300
Connection ~ 2950 3300
Text Notes 950  3000 0    50   ~ 0
Note: When the motors are running full bore\n  up a hill, the current drain becomes significant.\n  Use to two USB connectors to maximize current\n  from the USB battery pack.
$EndSCHEMATC
