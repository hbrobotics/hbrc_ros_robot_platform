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
NoConn ~ 5550 3450
NoConn ~ 5550 3550
NoConn ~ 5550 3650
NoConn ~ 5550 3750
NoConn ~ 5550 3850
NoConn ~ 5550 3950
$Comp
L HR2:ENCODER;2xM1x3RA CN1
U 1 1 5F409D86
P 4650 3450
F 0 "CN1" H 4950 3600 50  0000 C CNN
F 1 "ENCODER;2xM1x3RA" H 5100 2800 50  0000 C CNN
F 2 "HR2:ENCODER_2xM1x3RA" H 4850 3500 60  0001 L CNN
F 3 "HR2 Encoder Connectors" H 4850 3300 60  0001 L CNN
F 4 "HR2 Encoder Connectors" H 4850 3200 60  0001 L CNN "Field5"
	1    4650 3450
	1    0    0    -1  
$EndComp
NoConn ~ 6450 3950
NoConn ~ 6450 3850
NoConn ~ 6450 3750
$Comp
L HR2:AH1806;SC59 Q1
U 1 1 5F40AB04
P 7250 3750
F 0 "Q1" H 7850 3900 50  0000 R CNN
F 1 "AH1806;SC59" H 7800 3400 50  0000 R CNN
F 2 "HR2:SC59" H 7450 3800 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 7450 3600 60  0001 L CNN
F 4 "https://www.diodes.com/assets/Datasheets/AH1806.pdf" H 7450 3700 60  0001 L CNN "manf#"
F 5 "AH1806 Omnipolar Hall Effect Switch" H 7450 3500 60  0001 L CNN "Field5"
	1    7250 3750
	-1   0    0    -1  
$EndComp
$EndSCHEMATC
