EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "Encoder"
Date "2021-04-18"
Rev "F"
Comp "Homebrew Robotics Club"
Comment1 "License: MIT"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	5300 3500 4800 3500
$Comp
L Device:R_US R1
U 1 1 5F42263B
P 4200 3150
F 0 "R1" H 4100 3250 50  0000 L CNN
F 1 "47KΩ;1608" H 4200 3050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4240 3140 50  0001 C CNN
F 3 "~" H 4200 3150 50  0001 C CNN
	1    4200 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 5F422F39
P 4800 3150
F 0 "R2" H 4700 3250 50  0000 L CNN
F 1 "47KΩ;1608" H 4800 3050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4840 3140 50  0001 C CNN
F 3 "~" H 4800 3150 50  0001 C CNN
	1    4800 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3000 4200 2800
Wire Wire Line
	4800 3300 4800 3500
Text Label 3350 4500 0    50   ~ 0
QUADA
Text Label 3350 4400 0    50   ~ 0
GND
Text Notes 6000 4000 0    50   ~ 0
The DRV5013FAQBZR is a Hall Effect sensor that changes state each time\nthe sensor senses that magnetic field has changed.  Please note that only\na small number of the DRV5013 detect filed direction changes.  Most only\ndetect presense and non presense of a field.  The latter ones are not useful.\nThe center of each SOT23 package must be placed and the magnetic field\nflux near maximum strength.  In addition, the two sensors must be placed\n90 degrees apart.\n
Text Notes 1650 5300 0    50   ~ 0
CN1 is a hybrid connector for two male 1x3 right angle pin headers and\ntwo oval slots to be soldered to the motors.  It is represented as an\nintegrated connector so that the mechanical spacing of everything is\ncorrect.  There is also a motor shaft hole included.
Wire Wire Line
	5300 2900 5850 2900
Wire Wire Line
	5300 2900 5300 3500
Wire Wire Line
	4200 4500 5850 4500
Wire Wire Line
	4200 3300 4200 4500
Connection ~ 4200 4500
Wire Wire Line
	5300 4600 5300 3500
Text Label 3350 4600 0    50   ~ 0
QUADB
Text Label 3350 4300 0    50   ~ 0
VCC
Wire Wire Line
	4000 4300 5750 4300
Connection ~ 4000 4300
Wire Wire Line
	4000 4300 4000 2800
Text Label 3350 4000 0    50   ~ 0
MOTOR-
Text Label 3350 3900 0    50   ~ 0
MOTOR+
Wire Wire Line
	3700 4100 3700 4000
Wire Wire Line
	3800 3900 3800 4200
Wire Wire Line
	3250 4500 4200 4500
Wire Wire Line
	3250 4300 4000 4300
Wire Wire Line
	3800 4200 3250 4200
Wire Wire Line
	3250 4100 3700 4100
Wire Wire Line
	3700 4000 3250 4000
Wire Wire Line
	3250 3900 3800 3900
$Comp
L HR2:DRV5013FAQDBZR Q1
U 1 1 606117B3
P 6550 2800
F 0 "Q1" H 7050 2950 50  0000 R CNN
F 1 "DRV5013FAQDBZR;SOT23" H 7350 2450 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6750 2850 60  0001 L CNN
F 3 "https://www.ti.com/lit/ds/symlink/drv5013.pdf" H 6750 2650 60  0001 L CNN
F 4 "https://www.ti.com/lit/ds/symlink/drv5013.pdf" H 6750 2750 60  0001 L CNN "manf#"
F 5 "DRV5013 Digital-Latch Hall Effect Sensor" H 6750 2550 60  0001 L CNN "desc"
	1    6550 2800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5850 2800 5750 2800
Connection ~ 4800 2800
Wire Wire Line
	4800 2800 4800 3000
Wire Wire Line
	4800 2800 4200 2800
Connection ~ 4200 2800
Wire Wire Line
	4200 2800 4000 2800
Wire Wire Line
	5850 3000 5400 3000
Wire Wire Line
	5400 3000 5400 4150
Connection ~ 5300 3500
$Comp
L HR2:DRV5013FAQDBZR Q2
U 1 1 6062122A
P 6550 4400
F 0 "Q2" H 7050 4550 50  0000 R CNN
F 1 "DRV5013FAQDBZR;SOT23" H 7350 4050 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6750 4450 60  0001 L CNN
F 3 "https://www.ti.com/lit/ds/symlink/drv5013.pdf" H 6750 4250 60  0001 L CNN
F 4 "https://www.ti.com/lit/ds/symlink/drv5013.pdf" H 6750 4350 60  0001 L CNN "manf#"
F 5 "DRV5013 Digital-Latch Hall Effect Sensor" H 6750 4150 60  0001 L CNN "desc"
	1    6550 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5850 4400 5750 4400
Wire Wire Line
	5750 4400 5750 4300
Wire Wire Line
	5850 4600 5400 4600
$Comp
L Device:C_Small C1
U 1 1 60634648
P 5600 2650
F 0 "C1" V 5371 2650 50  0000 C CNN
F 1 "10nF;1608" V 5462 2650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5600 2650 50  0001 C CNN
F 3 "~" H 5600 2650 50  0001 C CNN
	1    5600 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	5700 2650 5750 2650
Wire Wire Line
	5750 2650 5750 2800
Connection ~ 5750 2800
Wire Wire Line
	5750 2800 4800 2800
Wire Wire Line
	5500 2650 5400 2650
Wire Wire Line
	5400 2650 5400 3000
Connection ~ 5400 3000
$Comp
L Device:C_Small C2
U 1 1 60637842
P 5600 4150
F 0 "C2" V 5371 4150 50  0000 C CNN
F 1 "10nF;1608" V 5462 4150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5600 4150 50  0001 C CNN
F 3 "~" H 5600 4150 50  0001 C CNN
	1    5600 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	5500 4150 5400 4150
Connection ~ 5400 4150
Wire Wire Line
	5400 4150 5400 4400
Wire Wire Line
	5700 4150 5750 4150
Wire Wire Line
	5750 4150 5750 4300
Connection ~ 5750 4300
$Comp
L HR2:ENCODER_FIX;2xM1x3RA CN1
U 1 1 607CDDAB
P 2350 4600
F 0 "CN1" H 2600 3750 50  0000 C CNN
F 1 "ENCODER_FIX;2xM1x3RA" H 2850 4750 50  0000 C CNN
F 2 "HR2:ENCODER_2xM1x3RA" H 2550 4650 60  0001 L CNN
F 3 "Fixed HR2 Encoder Connectors" H 2550 4450 60  0001 L CNN
F 4 "Fixed HR2 Encoder Connectors" H 2550 4350 60  0001 L CNN "desc"
	1    2350 4600
	1    0    0    1   
$EndComp
Connection ~ 5400 4400
Wire Wire Line
	5400 4400 5400 4600
Wire Wire Line
	3250 4600 5300 4600
Wire Wire Line
	3250 4400 5400 4400
$EndSCHEMATC
