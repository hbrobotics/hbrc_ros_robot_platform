EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "Encoder"
Date "2021-01-20"
Rev "B"
Comp "Homebrew Robotics Club"
Comment1 "License: MIT"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Sensor_Magnetic:AH1806-W Q1
U 1 1 5F40AB04
P 7950 2650
F 0 "Q1" H 7850 3000 50  0000 R CNN
F 1 "AH1806;SC59" H 7900 2300 50  0000 R CNN
F 2 "HR2:SC59" H 8150 2700 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 8150 2500 60  0001 L CNN
F 4 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 8150 2600 60  0001 L CNN "manf#"
F 5 "AH1806 Omnipolar Hall Effect Switch" H 8150 2400 60  0001 L CNN "Field5"
	1    7950 2650
	-1   0    0    -1  
$EndComp
$Comp
L Sensor_Magnetic:AH1806-W Q2
U 1 1 5F41D920
P 7950 4250
F 0 "Q2" H 7850 4600 50  0000 R CNN
F 1 "AH1806;SC59" H 7900 3900 50  0000 R CNN
F 2 "HR2:SC59" H 8150 4300 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 8150 4100 60  0001 L CNN
F 4 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 8150 4200 60  0001 L CNN "manf#"
F 5 "AH1806 Omnipolar Hall Effect Switch" H 8150 4000 60  0001 L CNN "Field5"
	1    7950 4250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5900 2200 6400 2200
Wire Wire Line
	6200 3400 6600 3400
$Comp
L Device:R_US R1
U 1 1 5F42263B
P 6400 3750
F 0 "R1" H 6300 3850 50  0000 L CNN
F 1 "47KΩ;1005" H 6400 3650 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6440 3740 50  0001 C CNN
F 3 "~" H 6400 3750 50  0001 C CNN
	1    6400 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 5F422F39
P 6600 2850
F 0 "R2" H 6500 2950 50  0000 L CNN
F 1 "47KΩ;1005" H 6600 2750 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6640 2840 50  0001 C CNN
F 3 "~" H 6600 2850 50  0001 C CNN
	1    6600 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 3600 6400 2200
Connection ~ 6400 2200
Wire Wire Line
	6400 2200 6600 2200
Wire Wire Line
	6600 2200 6600 2700
Wire Wire Line
	6600 3000 6600 3400
Connection ~ 6600 2200
Text Label 5150 4250 0    50   ~ 0
QUADA
Text Label 5150 4350 0    50   ~ 0
GND
Wire Wire Line
	6600 3400 7100 3400
Connection ~ 6600 3400
Text Notes 7350 3700 0    50   ~ 0
The AH1806 is a Unipolar Hall Effect sensor.  The center of each\nSC59 package must be placed 4mm from the motor shaft center\n(i.e. on a 4mm radius circle centered on the motor shaft.)\nIn addition, the two sensors must be placed 90 degrees apart\non this 4mm radius circle.\n
Text Notes 3450 5050 0    50   ~ 0
CN1 is a hybrid connector for two male 1x3 right angle pin headers and\ntwo oval slots to be soldered to the motors.  It is represented as an\nintegrated connector so that the mechanical spacing of everything is\ncorrect.  There is also a motor shaft hole included.
Wire Wire Line
	7950 2200 7950 2250
Wire Wire Line
	6600 2200 7950 2200
Wire Wire Line
	7950 3100 7950 3050
Wire Wire Line
	7500 3800 7950 3800
Wire Wire Line
	7950 3800 7950 3850
Wire Wire Line
	7100 2650 7650 2650
Wire Wire Line
	7100 2650 7100 3400
Wire Wire Line
	7200 4350 7200 4750
Wire Wire Line
	7200 4750 7950 4750
Wire Wire Line
	7950 4750 7950 4650
Wire Wire Line
	7200 4350 7200 3100
Wire Wire Line
	7200 3100 7950 3100
Connection ~ 7200 4350
Wire Wire Line
	6400 4250 7650 4250
Wire Wire Line
	6400 3900 6400 4250
Connection ~ 6400 4250
Wire Wire Line
	6200 4150 6200 3400
Text Label 5150 4150 0    50   ~ 0
QUADB
Text Label 5150 4050 0    50   ~ 0
VCC
Wire Wire Line
	7500 3800 7500 4050
Wire Wire Line
	5900 4050 7500 4050
Connection ~ 5900 4050
Wire Wire Line
	5900 4050 5900 2200
Text Label 5150 3750 0    50   ~ 0
MOTOR-
Text Label 5150 3650 0    50   ~ 0
MOTOR+
Wire Wire Line
	5500 3850 5500 3750
Wire Wire Line
	5600 3650 5600 3950
Wire Wire Line
	7200 4350 5050 4350
Wire Wire Line
	5050 4250 6400 4250
Wire Wire Line
	5050 4150 6200 4150
Wire Wire Line
	5050 4050 5900 4050
Wire Wire Line
	5600 3950 5050 3950
Wire Wire Line
	5050 3850 5500 3850
Wire Wire Line
	5500 3750 5050 3750
Wire Wire Line
	5050 3650 5600 3650
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
$EndSCHEMATC
