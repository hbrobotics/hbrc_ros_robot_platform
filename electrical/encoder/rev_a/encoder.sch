EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "Encoder"
Date "2020-06-06"
Rev "A"
Comp "Homebrew Robotics Club"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L HR2:ENCODER;2xM1x3RA CN1
U 1 1 5F409D86
P 4150 4350
F 0 "CN1" H 4450 4500 50  0000 C CNN
F 1 "ENCODER;2xM1x3RA" H 4600 3500 50  0000 C CNN
F 2 "HR2:ENCODER_2xM1x3RA" H 4350 4400 60  0001 L CNN
F 3 "HR2 Encoder Connectors" H 4350 4200 60  0001 L CNN
F 4 "HR2 Encoder Connectors" H 4350 4100 60  0001 L CNN "Field5"
	1    4150 4350
	1    0    0    1   
$EndComp
$Comp
L HR2:AH1806;SC59 Q1
U 1 1 5F40AB04
P 8100 2700
F 0 "Q1" H 8700 2850 50  0000 R CNN
F 1 "AH1806;SC59" H 8650 2350 50  0000 R CNN
F 2 "HR2:SC59" H 8300 2750 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 8300 2550 60  0001 L CNN
F 4 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 8300 2650 60  0001 L CNN "manf#"
F 5 "AH1806 Omnipolar Hall Effect Switch" H 8300 2450 60  0001 L CNN "Field5"
	1    8100 2700
	-1   0    0    -1  
$EndComp
$Comp
L HR2:AH1806;SC59 Q2
U 1 1 5F41D920
P 8100 4050
F 0 "Q2" H 8700 4200 50  0000 R CNN
F 1 "AH1806;SC59" H 8650 3700 50  0000 R CNN
F 2 "HR2:SC59" H 8300 4100 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 8300 3900 60  0001 L CNN
F 4 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 8300 4000 60  0001 L CNN "manf#"
F 5 "AH1806 Omnipolar Hall Effect Switch" H 8300 3800 60  0001 L CNN "Field5"
	1    8100 4050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5050 3850 5500 3850
Wire Wire Line
	5500 3850 5500 3750
Wire Wire Line
	5500 3750 5050 3750
Wire Wire Line
	5050 3650 5600 3650
Wire Wire Line
	5600 3650 5600 3950
Wire Wire Line
	5600 3950 5050 3950
Wire Wire Line
	5050 4050 5900 4050
Wire Wire Line
	5900 4050 5900 2700
Wire Wire Line
	5900 2700 6400 2700
Wire Wire Line
	5900 4050 7300 4050
Connection ~ 5900 4050
Wire Wire Line
	7300 2800 7200 2800
Wire Wire Line
	7200 2800 7200 4150
Wire Wire Line
	7200 4350 5050 4350
Wire Wire Line
	7300 4150 7200 4150
Connection ~ 7200 4150
Wire Wire Line
	7200 4150 7200 4350
Wire Wire Line
	5050 4150 6200 4150
Wire Wire Line
	6200 4150 6200 3400
Wire Wire Line
	6200 3400 6600 3400
Wire Wire Line
	5050 4250 6400 4250
$Comp
L Device:R_US R1
U 1 1 5F42263B
P 6400 3750
F 0 "R1" H 6300 3850 50  0000 L CNN
F 1 "100Ω;1608" H 6400 3650 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6440 3740 50  0001 C CNN
F 3 "~" H 6400 3750 50  0001 C CNN
	1    6400 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 5F422F39
P 6600 3050
F 0 "R2" H 6500 3150 50  0000 L CNN
F 1 "100Ω;1608" H 6600 2950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6640 3040 50  0001 C CNN
F 3 "~" H 6600 3050 50  0001 C CNN
	1    6600 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 3600 6400 2700
Connection ~ 6400 2700
Connection ~ 6400 4250
Wire Wire Line
	6400 4250 7300 4250
Wire Wire Line
	6400 2700 6600 2700
Wire Wire Line
	6600 2700 6600 2900
Wire Wire Line
	6600 3200 6600 3400
Connection ~ 6600 2700
Text Label 5150 4050 0    50   ~ 0
VCC
Text Label 5150 4150 0    50   ~ 0
QUADB
Text Label 5150 4250 0    50   ~ 0
QUADA
Text Label 5150 4350 0    50   ~ 0
GND
Text Label 5150 3650 0    50   ~ 0
MOTOR+
Text Label 5150 3750 0    50   ~ 0
MOTOR-
Wire Wire Line
	6600 2700 7300 2700
Wire Wire Line
	7050 3400 7050 2900
Wire Wire Line
	7050 2900 7300 2900
Wire Wire Line
	6400 3900 6400 4250
Wire Wire Line
	6600 3400 7050 3400
Connection ~ 6600 3400
Text Notes 7350 3700 0    50   ~ 0
The AH1806 is a Unipolar Hall Effect sensor.  The center of each\nSC59 package must be placed 4mm from the motor shaft center\n(i.e. on a 4mm radius circle centered on the motor shaft.)\nIn addition, the two sensors must be placed 90 degrees apart\non this 4mm radius circle.\n
Text Notes 3450 5050 0    50   ~ 0
CN1 is a hybrid connector for two male 1x3 right angle pin headers and\ntwo oval slots to be soldered to the motors.  It is represented as an\nintegrated connector so that the mechanical spacing of everything is\ncorrect.  There is also a motor shaft hole included.
$EndSCHEMATC
