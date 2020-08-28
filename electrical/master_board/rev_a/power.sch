EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 5 8
Title "HR2: Power"
Date "2020-08-27"
Rev "A"
Comp "Home Brew Robotics CLub"
Comment1 "Copyright © 2020 by Home Brew Robotics Club"
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 8000 3100 2    50   Output ~ 0
9V
Text HLabel 8000 3200 2    50   Output ~ 0
5V
Wire Wire Line
	7400 3100 8000 3100
Text HLabel 8200 6000 2    50   Output ~ 0
GND
Wire Wire Line
	7700 3300 7400 3300
Wire Wire Line
	7400 3400 7700 3400
Wire Wire Line
	7700 3400 7700 3300
NoConn ~ 7400 3900
Connection ~ 7700 3400
Wire Wire Line
	7700 6000 7700 3800
$Comp
L Device:Q_PMOS_GDS Q2
U 1 1 5F4A99AA
P 4100 4500
F 0 "Q2" V 4300 4600 50  0000 C CNN
F 1 "   FDN360P;SOT23" V 4000 4250 50  0000 C CNN
F 2 "" H 4300 4600 50  0001 C CNN
F 3 "~" H 4100 4500 50  0001 C CNN
	1    4100 4500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R1
U 1 1 5F4AAFD8
P 3450 4650
F 0 "R1" H 3300 4750 50  0000 L CNN
F 1 "100KΩ;1608" H 3500 4550 50  0000 L CNN
F 2 "" V 3490 4640 50  0001 C CNN
F 3 "~" H 3450 4650 50  0001 C CNN
	1    3450 4650
	1    0    0    -1  
$EndComp
Text Notes 2000 6600 0    50   ~ 0
Note: Power On/Off circuit is from David  Jones EEVBlog #262
$Comp
L Device:Q_NPN_BEC Q1
U 1 1 5F4ABAA0
P 3550 5200
F 0 "Q1" H 3450 5350 50  0000 L CNN
F 1 "2N2222;SOT23" H 3000 5050 50  0000 L CNN
F 2 "" H 3750 5300 50  0001 C CNN
F 3 "~" H 3550 5200 50  0001 C CNN
	1    3550 5200
	-1   0    0    -1  
$EndComp
Text Notes 2000 6900 0    50   ~ 0
Note: The bipolar transtistors are generic small signal NPN SOT23\nin a BEC (i.e. pin1 = Base, pin 2 = Emitter, pi 3 = Collector) configuration.\nPretty much any NPN will do, so substitutions should be pretty safe.
$Comp
L Device:Q_NPN_BEC Q3
U 1 1 5F4AC33E
P 5900 5500
F 0 "Q3" H 6090 5546 50  0000 L CNN
F 1 "2N2222;SOT23" H 6090 5455 50  0000 L CNN
F 2 "" H 6100 5600 50  0001 C CNN
F 3 "~" H 5900 5500 50  0001 C CNN
	1    5900 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 5F4AC83B
P 4900 4750
F 0 "R2" H 4750 4850 50  0000 L CNN
F 1 "100KΩ;1608" H 4950 4650 50  0000 L CNN
F 2 "" V 4940 4740 50  0001 C CNN
F 3 "~" H 4900 4750 50  0001 C CNN
	1    4900 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R4
U 1 1 5F4ACD3F
P 6000 4750
F 0 "R4" H 5850 4850 50  0000 L CNN
F 1 "100KΩ;1608" H 6050 4650 50  0000 L CNN
F 2 "" V 6040 4740 50  0001 C CNN
F 3 "~" H 6000 4750 50  0001 C CNN
	1    6000 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R3
U 1 1 5F4ACF72
P 5500 4750
F 0 "R3" H 5350 4850 50  0000 L CNN
F 1 "1MΩ;1608" H 5550 4650 50  0000 L CNN
F 2 "" V 5540 4740 50  0001 C CNN
F 3 "~" H 5500 4750 50  0001 C CNN
	1    5500 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4400 3800 4400
Wire Wire Line
	3450 4400 3450 4500
Wire Wire Line
	3450 4800 3450 4900
Wire Wire Line
	3450 4900 4100 4900
Wire Wire Line
	4100 4900 4100 4700
Wire Wire Line
	3450 5000 3450 4900
Connection ~ 3450 4900
Wire Wire Line
	4900 4600 4900 4400
Wire Wire Line
	4900 4400 4300 4400
Wire Wire Line
	3750 5200 4900 5200
Wire Wire Line
	4900 5200 4900 4900
$Comp
L Switch:SW_Push SW1
U 1 1 5F4B612A
P 5200 5200
F 0 "SW1" H 5050 5300 50  0000 C CNN
F 1 "PTS645SK50SMTR92 LFS;6x9.5" H 4850 5150 50  0000 C CNN
F 2 "" H 5200 5400 50  0001 C CNN
F 3 "~" H 5200 5400 50  0001 C CNN
	1    5200 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 5200 5000 5200
Connection ~ 4900 5200
Wire Wire Line
	3800 4400 3800 4200
Wire Wire Line
	3800 4200 6000 4200
Wire Wire Line
	6000 4200 6000 4600
Connection ~ 3800 4400
Wire Wire Line
	3800 4400 3450 4400
Wire Wire Line
	4900 4400 5500 4400
Wire Wire Line
	5500 4400 5500 4600
Connection ~ 4900 4400
Wire Wire Line
	5400 5200 6000 5200
Wire Wire Line
	6000 5200 6000 5300
Wire Wire Line
	6000 5200 6000 4900
Connection ~ 6000 5200
Wire Wire Line
	5700 5500 5500 5500
Wire Wire Line
	5500 5500 5500 4900
$Comp
L Device:C C1
U 1 1 5F4BF513
P 5500 5750
F 0 "C1" H 5350 5850 50  0000 L CNN
F 1 "22µF;1608" H 5550 5650 50  0000 L CNN
F 2 "" H 5538 5600 50  0001 C CNN
F 3 "~" H 5500 5750 50  0001 C CNN
	1    5500 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 5900 5500 6000
Wire Wire Line
	5500 6000 6000 6000
Wire Wire Line
	6000 6000 6000 5700
Wire Wire Line
	3450 5400 3450 6000
Wire Wire Line
	3450 6000 5500 6000
Connection ~ 5500 6000
Connection ~ 3450 4400
Connection ~ 5500 4400
Wire Wire Line
	5500 5600 5500 5500
Connection ~ 5500 5500
$Comp
L Connector:USB_B_Mini CN1
U 1 1 5F4B0748
P 2700 4600
F 0 "CN1" H 2757 5067 50  0000 C CNN
F 1 "AMP10103594-0001LF;USB1x6H" H 1900 4250 50  0000 C CNN
F 2 "" H 2850 4550 50  0001 C CNN
F 3 "~" H 2850 4550 50  0001 C CNN
	1    2700 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 4400 3450 4400
NoConn ~ 3000 4600
NoConn ~ 3000 4700
NoConn ~ 3000 4800
Wire Wire Line
	2600 5000 2600 5100
Wire Wire Line
	2600 5100 2700 5100
Wire Wire Line
	2700 5100 2700 5000
Wire Wire Line
	2600 5100 2600 6000
Wire Wire Line
	2600 6000 3450 6000
Connection ~ 2600 5100
Connection ~ 3450 6000
Wire Wire Line
	8200 6000 7700 6000
Connection ~ 6000 6000
Wire Wire Line
	7600 3500 7400 3500
Connection ~ 7700 6000
Wire Wire Line
	7700 6000 6000 6000
$Comp
L HR2:U3V70xMATE;F1x4+F1x5 CN?
U 1 1 5F5303B1
P 6600 3900
AR Path="/5F5303B1" Ref="CN?"  Part="1" 
AR Path="/5F52F39E/5F5303B1" Ref="CN42"  Part="1" 
F 0 "CN42" H 6900 2950 50  0000 C CNN
F 1 "POLOLU_U3V70F9;F1x4+F1x5" H 7000 4050 50  0000 C CNN
F 2 "HR2:U3V70xMATE_F1x4+F1x5" H 6800 3950 60  0001 L CNN
F 3 "Pololu U3V70x Voltage Regulator Mating Connector" H 6800 3750 60  0001 L CNN
F 4 "Pololu U3V70x Voltage Regulator Mating Connector" H 6800 3850 60  0001 L CNN "manf#"
	1    6600 3900
	1    0    0    1   
$EndComp
Wire Wire Line
	7400 3800 7700 3800
Connection ~ 7700 3800
Wire Wire Line
	7700 3800 7700 3700
Wire Wire Line
	7400 3700 7700 3700
Connection ~ 7700 3700
Wire Wire Line
	7700 3700 7700 3400
Wire Wire Line
	7400 3200 8000 3200
Wire Wire Line
	7400 3600 7600 3600
Connection ~ 7600 3600
Wire Wire Line
	7600 3600 7600 3500
Wire Wire Line
	5500 4400 7600 4400
Wire Wire Line
	7600 3600 7600 4400
$EndSCHEMATC
