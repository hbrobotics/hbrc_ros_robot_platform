EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 9 10
Title "HR2 Motors and Encoders"
Date "2020-10-17"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	3300 5100 3300 4700
Text HLabel 4200 4900 2    50   Input ~ 0
LMOTOR_CTL2
Text HLabel 4200 4400 2    50   Input ~ 0
RMOTOR_CTL2
Text HLabel 4200 4300 2    50   Input ~ 0
RMOTOR_CTL1
Text HLabel 4200 4600 2    50   Input ~ 0
9V
Wire Wire Line
	3200 4300 4200 4300
Wire Wire Line
	3200 4400 4200 4400
Wire Wire Line
	3200 4500 5000 4500
Wire Wire Line
	3200 4600 3900 4600
Wire Wire Line
	3300 4700 3200 4700
Wire Wire Line
	3200 4900 4200 4900
Wire Wire Line
	3200 5100 3300 5100
Text HLabel 4200 5000 2    50   Input ~ 0
LMOTOR_CTL1
$Comp
L HR2:DRV8833PWPR;16HTSSOP U1
U 1 1 5F4C0C8C
P 2300 3500
F 0 "U1" H 2550 3650 50  0000 C CNN
F 1 "DRV8833PWPR;16HTSSOP" H 2700 1750 50  0000 C CNN
F 2 "Package_SO:HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm_Mask3x3mm_ThermalVias" H 2500 3550 60  0001 L CNN
F 3 "Dual H-Bridge Motor Drive" H 2500 3350 60  0001 L CNN
F 4 "DualH-Bridge Motor Drive" H 2500 3250 60  0001 L CNN "Field5"
	1    2300 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 5000 4200 5000
Wire Wire Line
	5000 4500 5000 4600
$Comp
L Device:C C3
U 1 1 5F4C3DE1
P 5000 4750
F 0 "C3" H 4900 4850 50  0000 L CNN
F 1 "10nF,16V,X7R;1608" H 5050 4650 50  0000 L CNN
F 2 "" H 5038 4600 50  0001 C CNN
F 3 "~" H 5000 4750 50  0001 C CNN
	1    5000 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5F4CB112
P 4800 5250
F 0 "C2" H 4700 5350 50  0000 L CNN
F 1 "2.2uF,6.3V;1608" H 4150 5150 50  0000 L CNN
F 2 "" H 4838 5100 50  0001 C CNN
F 3 "~" H 4800 5250 50  0001 C CNN
	1    4800 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 5100 4800 4800
Wire Wire Line
	4800 4800 3200 4800
Wire Wire Line
	3300 5100 3300 5500
Wire Wire Line
	3300 5500 3900 5500
Wire Wire Line
	4800 5500 4800 5400
Connection ~ 3300 5100
Wire Wire Line
	4800 5500 5000 5500
Wire Wire Line
	5000 5500 5000 4900
Connection ~ 4800 5500
$Comp
L Device:R_US R12
U 1 1 5F4D52D3
P 6050 5300
F 0 "R12" V 5950 5250 50  0000 C CNN
F 1 "1KΩ.1W;1608" V 6150 5300 50  0000 C CNN
F 2 "" V 6090 5290 50  0001 C CNN
F 3 "~" H 6050 5300 50  0001 C CNN
	1    6050 5300
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R11
U 1 1 5F4D52DD
P 6050 5000
F 0 "R11" V 5950 4950 50  0000 C CNN
F 1 "1KΩ.1W;1608" V 6150 5000 50  0000 C CNN
F 2 "" V 6090 4990 50  0001 C CNN
F 3 "~" H 6050 5000 50  0001 C CNN
	1    6050 5000
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R10
U 1 1 5F4D52E7
P 6050 4700
F 0 "R10" V 5950 4650 50  0000 C CNN
F 1 "1KΩ.1W;1608" V 6150 4700 50  0000 C CNN
F 2 "" V 6090 4690 50  0001 C CNN
F 3 "~" H 6050 4700 50  0001 C CNN
	1    6050 4700
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R9
U 1 1 5F4D52F1
P 6050 4400
F 0 "R9" V 5950 4350 50  0000 C CNN
F 1 "1KΩ.1W;1608" V 6150 4400 50  0000 C CNN
F 2 "" V 6090 4390 50  0001 C CNN
F 3 "~" H 6050 4400 50  0001 C CNN
	1    6050 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 5300 6400 5300
Wire Wire Line
	6400 5300 6400 5000
Wire Wire Line
	6400 4400 6200 4400
Wire Wire Line
	6200 4700 6400 4700
Connection ~ 6400 4700
Wire Wire Line
	6400 4700 6400 4400
Wire Wire Line
	6200 5000 6400 5000
Connection ~ 6400 5000
Wire Wire Line
	6400 5000 6400 4700
Wire Wire Line
	5900 5000 5700 5000
Wire Wire Line
	5700 5000 5700 5300
Wire Wire Line
	5700 5300 5900 5300
Wire Wire Line
	5900 4700 5700 4700
Wire Wire Line
	5700 4700 5700 5000
Connection ~ 5700 5000
Wire Wire Line
	5900 4400 5700 4400
Wire Wire Line
	5700 4400 5700 4700
Connection ~ 5700 4700
Wire Wire Line
	6900 4000 6000 4000
Wire Wire Line
	6000 4000 6000 4100
Wire Wire Line
	6000 4100 3200 4100
Wire Wire Line
	4900 2000 6900 2000
Text Label 3250 3600 0    50   ~ 0
LMOTOR_OUT
Text Label 3250 3800 0    50   ~ 0
LMOTOR_OUT
Text Label 3250 4300 0    50   ~ 0
RMOTOR_CTL1
Text Label 3250 4400 0    50   ~ 0
RMOTOR_CTL2
Text Label 3350 5000 0    50   ~ 0
LMOTOR_CTL1
Text Label 3350 4900 0    50   ~ 0
LMOTOR_CTL2
Wire Wire Line
	6900 2100 5000 2100
Wire Wire Line
	4900 2000 4900 3600
Wire Wire Line
	3200 3800 5000 3800
Wire Wire Line
	5000 2100 5000 3800
Wire Wire Line
	3200 3500 4200 3500
Text HLabel 4200 3500 2    50   Input ~ 0
~MOTOR_SLEEP
Text Label 3250 3500 0    50   ~ 0
~MOTOR_SLEEP
Text HLabel 4200 4200 2    50   Output ~ 0
~MOTOR_FAULT
Wire Wire Line
	3200 4200 4200 4200
Text Label 3250 4200 0    50   ~ 0
~MOTOR_FAULT
Wire Wire Line
	3200 3600 4900 3600
Wire Wire Line
	3200 3700 5700 3700
Connection ~ 5700 3700
Connection ~ 5700 3100
Wire Wire Line
	5700 2800 5700 3100
Wire Wire Line
	5900 2800 5700 2800
Connection ~ 5700 3400
Wire Wire Line
	5700 3100 5700 3400
Wire Wire Line
	5900 3100 5700 3100
Wire Wire Line
	5700 3700 5900 3700
Wire Wire Line
	5700 3400 5700 3700
Wire Wire Line
	5900 3400 5700 3400
Wire Wire Line
	6400 3400 6400 3100
Connection ~ 6400 3400
Wire Wire Line
	6200 3400 6400 3400
Wire Wire Line
	6400 3100 6400 2800
Connection ~ 6400 3100
Wire Wire Line
	6200 3100 6400 3100
Wire Wire Line
	6400 2800 6200 2800
Wire Wire Line
	6400 3700 6400 3400
Wire Wire Line
	6200 3700 6400 3700
$Comp
L Device:R_US R5
U 1 1 5F4CD3C4
P 6050 2800
F 0 "R5" V 5950 2750 50  0000 C CNN
F 1 "1KΩ.1W;1608" V 6150 2800 50  0000 C CNN
F 2 "" V 6090 2790 50  0001 C CNN
F 3 "~" H 6050 2800 50  0001 C CNN
	1    6050 2800
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R6
U 1 1 5F4CCE9C
P 6050 3100
F 0 "R6" V 5950 3050 50  0000 C CNN
F 1 "1KΩ.1W;1608" V 6150 3100 50  0000 C CNN
F 2 "" V 6090 3090 50  0001 C CNN
F 3 "~" H 6050 3100 50  0001 C CNN
	1    6050 3100
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R7
U 1 1 5F4CC9D3
P 6050 3400
F 0 "R7" V 5950 3350 50  0000 C CNN
F 1 "1KΩ.1W;1608" V 6150 3400 50  0000 C CNN
F 2 "" V 6090 3390 50  0001 C CNN
F 3 "~" H 6050 3400 50  0001 C CNN
	1    6050 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R8
U 1 1 5F4CC3F2
P 6050 3700
F 0 "R8" V 5950 3650 50  0000 C CNN
F 1 "1KΩ.1W;1608" V 6150 3700 50  0000 C CNN
F 2 "" V 6090 3690 50  0001 C CNN
F 3 "~" H 6050 3700 50  0001 C CNN
	1    6050 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	3200 3900 6900 3900
Text HLabel 8300 3300 2    50   Input ~ 0
3.3V
Text HLabel 8300 2900 2    50   Output ~ 0
LQUAD_B
Text HLabel 8300 3000 2    50   Output ~ 0
LQUAD_A
Text HLabel 8300 5000 2    50   Output ~ 0
RQUAD_A
Text HLabel 8300 4900 2    50   Output ~ 0
RQUAD_B
Wire Wire Line
	5000 5500 6400 5500
Wire Wire Line
	6400 4200 6900 4200
Connection ~ 5000 5500
Wire Wire Line
	5700 4400 5700 4000
Wire Wire Line
	5700 4000 3200 4000
Connection ~ 5700 4400
Wire Wire Line
	6400 5500 6400 5300
Connection ~ 6400 5300
Wire Wire Line
	6400 4200 6400 4400
Connection ~ 6400 4400
Wire Wire Line
	6400 3700 6400 4200
Connection ~ 6400 3700
Connection ~ 6400 4200
Wire Wire Line
	6900 2300 6400 2300
Wire Wire Line
	6400 2300 6400 2800
Connection ~ 6400 2800
Text HLabel 8300 5500 2    50   Input ~ 0
GND
Wire Wire Line
	6400 5500 8300 5500
Connection ~ 6400 5500
Wire Wire Line
	6900 4100 6500 4100
Wire Wire Line
	6500 4100 6500 3300
Wire Wire Line
	6500 2200 6900 2200
Wire Wire Line
	8300 3300 6500 3300
Connection ~ 6500 3300
Wire Wire Line
	6500 3300 6500 2200
Wire Wire Line
	6900 2400 6700 2400
Wire Wire Line
	6700 2400 6700 3000
Wire Wire Line
	6700 3000 8300 3000
Wire Wire Line
	6900 2500 6800 2500
Wire Wire Line
	6800 2500 6800 2900
Wire Wire Line
	6800 2900 8300 2900
Wire Wire Line
	6900 4300 6700 4300
Wire Wire Line
	8300 5000 6700 5000
Wire Wire Line
	6700 4300 6700 5000
Wire Wire Line
	8300 4900 6800 4900
Wire Wire Line
	6800 4900 6800 4400
Wire Wire Line
	6800 4400 6900 4400
Text Label 7300 3300 2    50   ~ 0
3.3V
Text Label 8300 5000 2    50   ~ 0
RQUAD_A
Text Label 8300 4900 2    50   ~ 0
RQUAD_B
Text Label 7300 3000 2    50   ~ 0
LQUAD_A
Text Label 7300 2900 2    50   ~ 0
LQUAD_B
Text Label 3700 4600 0    50   ~ 0
9V
Text Label 8300 5500 2    50   ~ 0
GND
Text Notes 5350 2650 0    50   ~ 0
250Ω  .4W Current Sense
Text Notes 5450 5650 0    50   ~ 0
250Ω  .4W Current Sense
Text Notes 2400 3300 0    50   ~ 0
Motor Controller\n
Text Notes 6950 3700 0    50   ~ 0
Right Motor/Encoder\nConnectors
Text Notes 6900 1800 0    50   ~ 0
Left Motor/Encoder\nConnectors
$Comp
L Device:C C22
U 1 1 5FA19E5E
P 3900 5250
F 0 "C22" H 3750 5350 50  0000 L CNN
F 1 "10pF;1608" H 3500 5150 50  0000 L CNN
F 2 "" H 3938 5100 50  0001 C CNN
F 3 "~" H 3900 5250 50  0001 C CNN
	1    3900 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 5100 3900 4600
Connection ~ 3900 4600
Wire Wire Line
	3900 4600 4200 4600
Wire Wire Line
	3900 5400 3900 5500
Connection ~ 3900 5500
Wire Wire Line
	3900 5500 4800 5500
$Comp
L HR2:ENCODER;2xF1x3 CN55
U 1 1 5F967601
P 7800 2000
F 0 "CN55" H 8500 2150 50  0000 R CNN
F 1 "HR2_ENCODER;2xF1x3" H 8650 1350 50  0000 R CNN
F 2 "HR2:ENCODER_2xF1x3" H 8000 2050 60  0001 L CNN
F 3 "HR2 Encoder Connectors" H 8000 1850 60  0001 L CNN
F 4 "HR2 Encoder Connector" H 8000 1750 60  0001 L CNN "Field5"
	1    7800 2000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:ENCODER;2xF1x3 CN56
U 1 1 5F968E59
P 7800 3900
F 0 "CN56" H 8500 4050 50  0000 R CNN
F 1 "HR2_ENCODER;2xF1x3" H 8650 3250 50  0000 R CNN
F 2 "HR2:ENCODER_2xF1x3" H 8000 3950 60  0001 L CNN
F 3 "HR2 Encoder Connectors" H 8000 3750 60  0001 L CNN
F 4 "HR2 Encoder Connector" H 8000 3650 60  0001 L CNN "Field5"
	1    7800 3900
	-1   0    0    -1  
$EndComp
$EndSCHEMATC
