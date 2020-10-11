EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 6 9
Title "HR2 Servos and Sonars"
Date "2020-10-03"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 1000 1000 0    50   Input ~ 0
GND
Text HLabel 1000 700  0    50   Input ~ 0
5V
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F490633
P 7000 4400
AR Path="/5F490633" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490633" Ref="CN51"  Part="1" 
F 0 "CN51" H 7400 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 7450 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 7200 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 7200 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 7200 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 7200 4150 60  0001 L CNN "Field5"
	1    7000 4400
	-1   0    0    -1  
$EndComp
$Comp
L HR2:CD450B;TTSOP16 U6
U 1 1 5F56033D
P 2600 3000
F 0 "U6" H 2850 3150 50  0000 C CNN
F 1 "CD450B;TTSOP16" H 3150 2150 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 2800 3050 60  0001 L CNN
F 3 "Hex Level Shifter" H 2800 2850 60  0001 L CNN
F 4 "Hex Level Shifter" H 2800 2750 60  0001 L CNN "Field5"
	1    2600 3000
	1    0    0    -1  
$EndComp
Text HLabel 1000 900  0    50   Input ~ 0
3.3V
Wire Wire Line
	1000 700  1700 700 
Text Label 1650 700  2    50   ~ 0
5V
Text Label 1650 1000 2    50   ~ 0
GND
Text Label 1650 900  2    50   ~ 0
3.3V
Text Label 1950 6700 0    50   ~ 0
3.3V
Text Label 1950 7200 0    50   ~ 0
GND
Text Label 2600 3300 2    50   ~ 0
TRIG4
Text Label 2600 3200 2    50   ~ 0
TRIG5
Text Label 2600 3100 2    50   ~ 0
TRIG6
Text Label 3250 1900 0    50   ~ 0
TRIG7
Text Label 3700 3600 0    50   ~ 0
TRIG1_5V
Text Label 3700 3400 0    50   ~ 0
TRIG3_5V
Text Label 3700 3300 0    50   ~ 0
TRIG4_5V
Text Label 3700 3200 0    50   ~ 0
TRIG5_5V
Text Label 4700 1900 0    50   ~ 0
TRIG7_5V
$Comp
L Device:R_US R21
U 1 1 5FADB964
P 5900 4950
F 0 "R21" H 5950 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 5450 5050 50  0000 L CNN
F 2 "" V 5940 4940 50  0001 C CNN
F 3 "~" H 5900 4950 50  0001 C CNN
	1    5900 4950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R22
U 1 1 5FADC37B
P 5900 5450
F 0 "R22" H 5950 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 5450 5550 50  0000 L CNN
F 2 "" V 5940 5440 50  0001 C CNN
F 3 "~" H 5900 5450 50  0001 C CNN
	1    5900 5450
	-1   0    0    1   
$EndComp
Text Label 5900 4600 0    50   ~ 0
ECHO4_5V
Text Notes 2750 2800 0    50   ~ 0
3.3V to 5V Level Shifter
Text Notes 5500 2900 0    50   ~ 0
Notes:\n1. The Sonars are HC04's that operate off of 5V.\n2. The trigger signals are provided by a serial shift register.\n3. A level shifter is used to shift trigger signals from 3.3V to 5V.\n4. A resistor voltage divider is used to convert the echo from 5V to 3.3V.\n5. Most hobby servers operate off of approximately 5V - 6V.\n6. The 4.8V supply is a 5V supply that is used for servos and LED's.\n7. The servo PWM signals a level shifted from 3.3V to 5v.\n8. The servos are manually labeled CN50-CN56 to support mechanical\n    automatic positioned derived from the mechanical model.\n9. The echo signals are fed to external interrupt pins on the Nucleo PCB.
$Comp
L Device:C C10
U 1 1 5F7FE24F
P 2350 6950
F 0 "C10" H 2150 7050 50  0000 L CNN
F 1 "10pF;1608" H 2400 6850 50  0000 L CNN
F 2 "" H 2388 6800 50  0001 C CNN
F 3 "~" H 2350 6950 50  0001 C CNN
	1    2350 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 6800 2350 6700
Wire Wire Line
	2350 6700 1900 6700
Wire Wire Line
	2350 7100 2350 7200
Wire Wire Line
	2350 7200 1900 7200
Entry Wire Line
	1900 6700 1800 6800
Entry Wire Line
	1900 7200 1800 7100
$Comp
L Device:C C11
U 1 1 5F8B3EAF
P 2950 6950
F 0 "C11" H 2750 7050 50  0000 L CNN
F 1 "10pF;1608" H 3000 6850 50  0000 L CNN
F 2 "" H 2988 6800 50  0001 C CNN
F 3 "~" H 2950 6950 50  0001 C CNN
	1    2950 6950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5F8B4468
P 3550 6950
F 0 "C12" H 3350 7050 50  0000 L CNN
F 1 "10pF;1608" H 3600 6850 50  0000 L CNN
F 2 "" H 3588 6800 50  0001 C CNN
F 3 "~" H 3550 6950 50  0001 C CNN
	1    3550 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 6700 2950 6700
Connection ~ 2350 6700
Wire Wire Line
	2950 6800 2950 6700
Connection ~ 2950 6700
Wire Wire Line
	2950 6700 3550 6700
Wire Wire Line
	3550 6800 3550 6700
Wire Wire Line
	2350 7200 2950 7200
Connection ~ 2350 7200
Wire Wire Line
	3550 7100 3550 7200
Wire Wire Line
	2950 7100 2950 7200
Connection ~ 2950 7200
Wire Wire Line
	2950 7200 3550 7200
Text Notes 2500 6650 0    50   ~ 0
Filter Capacitors
Text Label 3300 2000 0    50   ~ 0
GND
Text Label 3700 3500 0    50   ~ 0
TRIG2_5V
Wire Wire Line
	4250 3600 3700 3600
Text Label 3700 3100 0    50   ~ 0
TRIG6_5V
Text Label 2600 3400 2    50   ~ 0
TRIG3
Text Label 2600 3500 2    50   ~ 0
TRIG2
Text Label 2600 3600 2    50   ~ 0
TRIG1
Text HLabel 3000 1600 0    50   Input ~ 0
SERVO2
Text HLabel 3000 1500 0    50   Input ~ 0
SERVO3
Text HLabel 3000 1400 0    50   Input ~ 0
SERVO4
Text Label 3000 1700 0    50   ~ 0
SERVO1
Text Label 3000 1600 0    50   ~ 0
SERVO2
Text Label 3000 1500 0    50   ~ 0
SERVO3
Text Label 3000 1400 0    50   ~ 0
SERVO4
Text Label 4700 1700 0    50   ~ 0
SERVO1_5V
Text Label 4700 1600 0    50   ~ 0
SERVO2_5V
Text Label 4700 1500 0    50   ~ 0
SERVO3_5V
Text Label 4700 1400 0    50   ~ 0
SERVO4_5V
Wire Wire Line
	5900 4600 5900 4800
Wire Wire Line
	5900 4600 6300 4600
Wire Wire Line
	5900 5300 5900 5200
Wire Wire Line
	6300 4700 6200 4700
Wire Wire Line
	6200 4700 6200 4900
Wire Wire Line
	6300 4400 6200 4400
Wire Wire Line
	6200 4400 6200 4200
Wire Wire Line
	5900 5600 5900 5700
Wire Wire Line
	5900 5700 6400 5700
Connection ~ 6200 4200
Wire Wire Line
	6300 4500 6000 4500
Connection ~ 5900 5700
Wire Wire Line
	6200 4900 6400 4900
Wire Wire Line
	6400 4900 6400 5700
Connection ~ 6400 5700
Wire Wire Line
	5900 5200 5600 5200
Connection ~ 5900 5200
Wire Wire Line
	5900 5200 5900 5100
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E2DBA
P 8200 4400
AR Path="/5F7E2DBA" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E2DBA" Ref="CN52"  Part="1" 
F 0 "CN52" H 8600 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 8650 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 8400 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8400 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8400 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 8400 4150 60  0001 L CNN "Field5"
	1    8200 4400
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_US R54
U 1 1 5F7E2DC4
P 7100 4950
F 0 "R54" H 7150 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 6650 5050 50  0000 L CNN
F 2 "" V 7140 4940 50  0001 C CNN
F 3 "~" H 7100 4950 50  0001 C CNN
	1    7100 4950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R55
U 1 1 5F7E2DCE
P 7100 5450
F 0 "R55" H 7150 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 6650 5550 50  0000 L CNN
F 2 "" V 7140 5440 50  0001 C CNN
F 3 "~" H 7100 5450 50  0001 C CNN
	1    7100 5450
	-1   0    0    1   
$EndComp
Text Label 7100 4600 0    50   ~ 0
ECHO5_5V
Wire Wire Line
	7100 4600 7100 4800
Wire Wire Line
	7100 4600 7500 4600
Wire Wire Line
	7100 5300 7100 5200
Wire Wire Line
	7500 4700 7400 4700
Wire Wire Line
	7400 4700 7400 4900
Wire Wire Line
	7500 4400 7400 4400
Wire Wire Line
	7400 4400 7400 4200
Wire Wire Line
	7100 5600 7100 5700
Wire Wire Line
	7100 5700 7600 5700
Connection ~ 7400 4200
Wire Wire Line
	7500 4500 7200 4500
Connection ~ 7100 5700
Wire Wire Line
	7400 4900 7600 4900
Wire Wire Line
	7600 4900 7600 5700
Connection ~ 7600 5700
Wire Wire Line
	7100 5200 6800 5200
Connection ~ 7100 5200
Wire Wire Line
	7100 5200 7100 5100
Wire Wire Line
	6400 5700 7100 5700
Wire Wire Line
	6200 4200 7400 4200
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E752A
P 9400 4400
AR Path="/5F7E752A" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E752A" Ref="CN53"  Part="1" 
F 0 "CN53" H 9800 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 9850 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 9600 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 9600 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 9600 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 9600 4150 60  0001 L CNN "Field5"
	1    9400 4400
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_US R56
U 1 1 5F7E7534
P 8300 4950
F 0 "R56" H 8350 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 7850 5050 50  0000 L CNN
F 2 "" V 8340 4940 50  0001 C CNN
F 3 "~" H 8300 4950 50  0001 C CNN
	1    8300 4950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R57
U 1 1 5F7E753E
P 8300 5450
F 0 "R57" H 8350 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 7850 5550 50  0000 L CNN
F 2 "" V 8340 5440 50  0001 C CNN
F 3 "~" H 8300 5450 50  0001 C CNN
	1    8300 5450
	-1   0    0    1   
$EndComp
Text Label 8300 4600 0    50   ~ 0
ECHO6_5V
Wire Wire Line
	8300 4600 8300 4800
Wire Wire Line
	8300 4600 8700 4600
Wire Wire Line
	8300 5300 8300 5200
Wire Wire Line
	8700 4700 8600 4700
Wire Wire Line
	8600 4700 8600 4900
Wire Wire Line
	8700 4400 8600 4400
Wire Wire Line
	8600 4400 8600 4200
Wire Wire Line
	8300 5600 8300 5700
Wire Wire Line
	8300 5700 8800 5700
Connection ~ 8600 4200
Wire Wire Line
	8700 4500 8400 4500
Connection ~ 8300 5700
Wire Wire Line
	8600 4900 8800 4900
Wire Wire Line
	8800 4900 8800 5700
Connection ~ 8800 5700
Wire Wire Line
	8300 5200 8000 5200
Connection ~ 8300 5200
Wire Wire Line
	8300 5200 8300 5100
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E7561
P 10600 4400
AR Path="/5F7E7561" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E7561" Ref="CN54"  Part="1" 
F 0 "CN54" H 11000 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 11050 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 10800 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 10800 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 10800 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 10800 4150 60  0001 L CNN "Field5"
	1    10600 4400
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_US R58
U 1 1 5F7E756B
P 9500 4950
F 0 "R58" H 9550 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 9050 5050 50  0000 L CNN
F 2 "" V 9540 4940 50  0001 C CNN
F 3 "~" H 9500 4950 50  0001 C CNN
	1    9500 4950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R59
U 1 1 5F7E7575
P 9500 5450
F 0 "R59" H 9550 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 9050 5550 50  0000 L CNN
F 2 "" V 9540 5440 50  0001 C CNN
F 3 "~" H 9500 5450 50  0001 C CNN
	1    9500 5450
	-1   0    0    1   
$EndComp
Text Label 9500 4600 0    50   ~ 0
ECHO7_5V
Wire Wire Line
	9500 4600 9500 4800
Wire Wire Line
	9500 4600 9900 4600
Wire Wire Line
	9500 5300 9500 5200
Wire Wire Line
	9900 4700 9800 4700
Wire Wire Line
	9800 4700 9800 4900
Wire Wire Line
	9900 4400 9800 4400
Wire Wire Line
	9800 4400 9800 4200
Wire Wire Line
	9500 5600 9500 5700
Wire Wire Line
	9500 5700 10050 5700
Wire Wire Line
	9900 4500 9500 4500
Connection ~ 9500 5700
Wire Wire Line
	9800 4900 10050 4900
Wire Wire Line
	10050 4900 10050 5700
Wire Wire Line
	9500 5200 9200 5200
Wire Wire Line
	9200 5200 9200 5900
Connection ~ 9500 5200
Wire Wire Line
	9500 5200 9500 5100
Wire Wire Line
	8800 5700 9500 5700
Wire Wire Line
	8600 4200 9800 4200
Wire Wire Line
	7600 5700 8300 5700
Wire Wire Line
	7400 4200 8600 4200
$Comp
L Device:R_US R48
U 1 1 5F7F0676
P 2300 4950
F 0 "R48" H 2350 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 1850 5050 50  0000 L CNN
F 2 "" V 2340 4940 50  0001 C CNN
F 3 "~" H 2300 4950 50  0001 C CNN
	1    2300 4950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R49
U 1 1 5F7F0680
P 2300 5450
F 0 "R49" H 2350 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 1850 5550 50  0000 L CNN
F 2 "" V 2340 5440 50  0001 C CNN
F 3 "~" H 2300 5450 50  0001 C CNN
	1    2300 5450
	-1   0    0    1   
$EndComp
Text Label 2300 4600 0    50   ~ 0
ECHO1_5V
Wire Wire Line
	2300 4600 2300 4800
Wire Wire Line
	2300 4600 2700 4600
Wire Wire Line
	2300 5300 2300 5200
Wire Wire Line
	2700 4700 2600 4700
Wire Wire Line
	2600 4700 2600 4900
Wire Wire Line
	2700 4400 2600 4400
Wire Wire Line
	2600 4400 2600 4200
Wire Wire Line
	2300 5600 2300 5700
Wire Wire Line
	2300 5700 2800 5700
Wire Wire Line
	2700 4500 2400 4500
Wire Wire Line
	2300 5700 1900 5700
Connection ~ 2300 5700
Wire Wire Line
	2600 4900 2800 4900
Wire Wire Line
	2800 4900 2800 5700
Connection ~ 2800 5700
Wire Wire Line
	2300 5200 2000 5200
Connection ~ 2300 5200
Wire Wire Line
	2300 5200 2300 5100
$Comp
L Device:R_US R50
U 1 1 5F7F06AD
P 3500 4950
F 0 "R50" H 3550 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 3050 5050 50  0000 L CNN
F 2 "" V 3540 4940 50  0001 C CNN
F 3 "~" H 3500 4950 50  0001 C CNN
	1    3500 4950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R51
U 1 1 5F7F06B7
P 3500 5450
F 0 "R51" H 3550 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 3050 5550 50  0000 L CNN
F 2 "" V 3540 5440 50  0001 C CNN
F 3 "~" H 3500 5450 50  0001 C CNN
	1    3500 5450
	-1   0    0    1   
$EndComp
Text Label 3500 4600 0    50   ~ 0
ECHO2_5V
Wire Wire Line
	3500 4600 3500 4800
Wire Wire Line
	3500 4600 3900 4600
Wire Wire Line
	3500 5300 3500 5200
Wire Wire Line
	3900 4700 3800 4700
Wire Wire Line
	3800 4700 3800 4900
Wire Wire Line
	3900 4400 3800 4400
Wire Wire Line
	3800 4400 3800 4200
Wire Wire Line
	3500 5600 3500 5700
Wire Wire Line
	3500 5700 4000 5700
Connection ~ 3800 4200
Wire Wire Line
	3900 4500 3600 4500
Connection ~ 3500 5700
Wire Wire Line
	3800 4900 4000 4900
Wire Wire Line
	4000 4900 4000 5700
Connection ~ 4000 5700
Wire Wire Line
	3500 5200 3200 5200
Connection ~ 3500 5200
Wire Wire Line
	3500 5200 3500 5100
Wire Wire Line
	2800 5700 3500 5700
Wire Wire Line
	2600 4200 3800 4200
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F06DA
P 5800 4400
AR Path="/5F7F06DA" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F06DA" Ref="CN50"  Part="1" 
F 0 "CN50" H 6200 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 6250 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 6000 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 6000 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 6000 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 6000 4150 60  0001 L CNN "Field5"
	1    5800 4400
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_US R52
U 1 1 5F7F06E4
P 4700 4950
F 0 "R52" H 4750 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 4250 5050 50  0000 L CNN
F 2 "" V 4740 4940 50  0001 C CNN
F 3 "~" H 4700 4950 50  0001 C CNN
	1    4700 4950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R53
U 1 1 5F7F06EE
P 4700 5450
F 0 "R53" H 4750 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 4250 5550 50  0000 L CNN
F 2 "" V 4740 5440 50  0001 C CNN
F 3 "~" H 4700 5450 50  0001 C CNN
	1    4700 5450
	-1   0    0    1   
$EndComp
Text Label 4700 4600 0    50   ~ 0
ECHO3_5V
Wire Wire Line
	4700 4600 4700 4800
Wire Wire Line
	4700 4600 5100 4600
Wire Wire Line
	4700 5300 4700 5200
Wire Wire Line
	5100 4700 5000 4700
Wire Wire Line
	5000 4700 5000 4900
Wire Wire Line
	5100 4400 5000 4400
Wire Wire Line
	5000 4400 5000 4200
Wire Wire Line
	4700 5600 4700 5700
Wire Wire Line
	4700 5700 5200 5700
Connection ~ 5000 4200
Wire Wire Line
	5100 4500 4800 4500
Connection ~ 4700 5700
Wire Wire Line
	5000 4900 5200 4900
Wire Wire Line
	5200 4900 5200 5700
Connection ~ 5200 5700
Wire Wire Line
	4700 5200 4400 5200
Connection ~ 4700 5200
Wire Wire Line
	4700 5200 4700 5100
Wire Wire Line
	4000 5700 4700 5700
Wire Wire Line
	3800 4200 5000 4200
Wire Wire Line
	5000 4200 6200 4200
Wire Wire Line
	5200 5700 5900 5700
Wire Wire Line
	9200 5900 9500 5900
Text Label 9500 4500 0    50   ~ 0
TRIG7_5V
Text HLabel 9500 5900 2    50   Output ~ 0
ECHO7
Text HLabel 9500 6000 2    50   Output ~ 0
ECHO6
Text HLabel 9500 6100 2    50   Output ~ 0
ECHO5
Text HLabel 9500 6200 2    50   Output ~ 0
ECHO4
Text HLabel 9500 6300 2    50   Output ~ 0
ECHO3
Text HLabel 9500 6400 2    50   Output ~ 0
ECHO2
Text HLabel 9500 6500 2    50   Output ~ 0
ECHO1
Wire Wire Line
	9500 6000 8000 6000
Wire Wire Line
	8000 5200 8000 6000
Text Label 9200 5200 0    50   ~ 0
ECHO7
Text Label 8000 5200 0    50   ~ 0
ECHO6
Text Label 6800 5200 0    50   ~ 0
ECHO5
Text Label 5600 5200 0    50   ~ 0
ECHO4
Text Label 4400 5200 0    50   ~ 0
ECHO3
Text Label 3200 5200 0    50   ~ 0
ECHO2
Text Label 2000 5200 0    50   ~ 0
ECHO1
Wire Wire Line
	9500 6100 6800 6100
Wire Wire Line
	6800 5200 6800 6100
Wire Wire Line
	9500 6200 5600 6200
Wire Wire Line
	5600 5200 5600 6200
Wire Wire Line
	9500 6300 4400 6300
Wire Wire Line
	4400 5200 4400 6300
Wire Wire Line
	9500 6400 3200 6400
Wire Wire Line
	3200 5200 3200 6400
Wire Wire Line
	9500 6500 2000 6500
Wire Wire Line
	2000 5200 2000 6500
$Comp
L HR2:SN74HCS595;TTSOP16 U9
U 1 1 5F9D42B3
P 400 3500
F 0 "U9" H 700 1850 60  0000 R CNN
F 1 "SN74HCS595;TTSOP16" H 1500 3700 60  0000 R CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 600 3550 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 600 3350 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 600 3250 60  0001 L CNN "Field5"
	1    400  3500
	1    0    0    1   
$EndComp
Wire Wire Line
	4250 3600 4250 4000
Wire Wire Line
	2400 4000 2400 4500
Wire Wire Line
	4450 4100 4450 3500
Wire Wire Line
	3700 3500 4450 3500
Wire Wire Line
	3600 4100 3600 4500
Wire Wire Line
	3700 3400 4800 3400
Wire Wire Line
	4800 3400 4800 4500
Wire Wire Line
	3700 3300 6000 3300
Wire Wire Line
	6000 3300 6000 4500
Wire Wire Line
	3700 3200 7200 3200
Wire Wire Line
	7200 3200 7200 4500
Wire Wire Line
	3700 3100 8400 3100
Wire Wire Line
	8400 3100 8400 4500
Wire Wire Line
	9500 1900 9500 4500
$Comp
L HR2:SERVO;F1x3 CN4
U 1 1 5FB31019
P 6300 1000
F 0 "CN4" H 6800 1150 60  0000 R CNN
F 1 "SERVO;F1x3" H 6950 650 60  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 1050 60  0001 L CNN
F 3 "Servo F1X3" H 6500 850 60  0001 L CNN
F 4 "Servo F1X3" H 6500 950 60  0001 L CNN "manf#"
	1    6300 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;F1x3 CN5
U 1 1 5FB9CF1F
P 7300 1000
F 0 "CN5" H 7800 1150 60  0000 R CNN
F 1 "SERVO;F1x3" H 7950 650 60  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 7500 1050 60  0001 L CNN
F 3 "Servo F1X3" H 7500 850 60  0001 L CNN
F 4 "Servo F1X3" H 7500 950 60  0001 L CNN "manf#"
	1    7300 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;F1x3 CN6
U 1 1 5FB9D556
P 8300 1000
F 0 "CN6" H 8800 1150 60  0000 R CNN
F 1 "SERVO;F1x3" H 8950 650 60  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 8500 1050 60  0001 L CNN
F 3 "Servo F1X3" H 8500 850 60  0001 L CNN
F 4 "Servo F1X3" H 8500 950 60  0001 L CNN "manf#"
	1    8300 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;F1x3 CN7
U 1 1 5FB9D857
P 9250 1000
F 0 "CN7" H 9750 1150 60  0000 R CNN
F 1 "SERVO;F1x3" H 9900 650 60  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 9450 1050 60  0001 L CNN
F 3 "Servo F1X3" H 9450 850 60  0001 L CNN
F 4 "Servo F1X3" H 9450 950 60  0001 L CNN "manf#"
	1    9250 1000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5600 1000 5400 1000
Wire Wire Line
	5600 1100 5400 1100
Wire Wire Line
	6400 1500 6400 1200
Wire Wire Line
	6400 1200 6600 1200
Wire Wire Line
	7400 1600 7400 1200
Wire Wire Line
	7400 1200 7600 1200
Wire Wire Line
	8350 1700 8350 1200
Wire Wire Line
	8350 1200 8550 1200
Wire Wire Line
	1200 3400 2600 3400
Wire Wire Line
	1200 3200 2600 3200
Wire Wire Line
	1200 3100 2600 3100
Text HLabel 1000 800  0    50   Input ~ 0
P5V
Wire Wire Line
	1700 800  1000 800 
Text Label 1650 800  2    50   ~ 0
P5V
Wire Wire Line
	1200 3000 2200 3000
Wire Wire Line
	1000 900  1700 900 
Wire Wire Line
	1200 3500 2600 3500
Text Label 5400 1000 0    50   ~ 0
GND
Text Label 5400 1100 0    50   ~ 0
P5V
Entry Wire Line
	5400 1100 5300 1000
Entry Wire Line
	5300 900  5400 1000
Wire Wire Line
	6600 1000 6400 1000
Wire Wire Line
	6600 1100 6400 1100
Text Label 6400 1000 0    50   ~ 0
GND
Text Label 6400 1100 0    50   ~ 0
P5V
Entry Wire Line
	6400 1100 6300 1000
Entry Wire Line
	6300 900  6400 1000
Wire Wire Line
	7600 1000 7400 1000
Wire Wire Line
	7600 1100 7400 1100
Text Label 7400 1000 0    50   ~ 0
GND
Text Label 7400 1100 0    50   ~ 0
P5V
Entry Wire Line
	7400 1100 7300 1000
Entry Wire Line
	7300 900  7400 1000
Wire Wire Line
	8550 1000 8350 1000
Wire Wire Line
	8550 1100 8350 1100
Text Label 8350 1000 0    50   ~ 0
GND
Text Label 8350 1100 0    50   ~ 0
P5V
Entry Wire Line
	8350 1100 8250 1000
Entry Wire Line
	8250 900  8350 1000
Wire Bus Line
	8250 650  7300 650 
Connection ~ 7300 650 
Wire Bus Line
	7300 650  6300 650 
Connection ~ 6300 650 
Wire Bus Line
	6300 650  5300 650 
Connection ~ 5300 650 
Text Label 1700 2300 2    50   ~ 0
GND
Text Label 1700 2000 2    50   ~ 0
3.3V
Wire Bus Line
	1800 650  3200 650 
Wire Wire Line
	2600 3000 2400 3000
Text Label 2400 3000 0    50   ~ 0
3.3V
Wire Wire Line
	2400 4000 4250 4000
Wire Wire Line
	3600 4100 4450 4100
Wire Wire Line
	1200 3300 2600 3300
Entry Wire Line
	1800 800  1700 700 
Entry Wire Line
	1800 900  1700 800 
Entry Wire Line
	1800 1000 1700 900 
Entry Wire Line
	1900 4200 1800 4100
Text Label 1900 4200 0    50   ~ 0
5V
Wire Wire Line
	5400 1200 5600 1200
Wire Wire Line
	5400 1400 5400 1200
Wire Wire Line
	4600 1500 6400 1500
Wire Wire Line
	4600 1600 7400 1600
Wire Wire Line
	4600 1700 8350 1700
Wire Wire Line
	4600 1900 9500 1900
NoConn ~ 3500 1800
Wire Wire Line
	3000 1700 3500 1700
Wire Wire Line
	3000 1600 3500 1600
Wire Wire Line
	3000 1500 3500 1500
Wire Wire Line
	3000 1400 3500 1400
$Comp
L HR2:CD450B;TTSOP16 U7
U 1 1 5F67B87E
P 3500 1300
F 0 "U7" H 3750 1450 50  0000 C CNN
F 1 "CD450B;TTSOP16" H 4050 450 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 3700 1350 60  0001 L CNN
F 3 "Hex Level Shifter" H 3700 1150 60  0001 L CNN
F 4 "Hex Level Shifter" H 3700 1050 60  0001 L CNN "Field5"
	1    3500 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2000 1700 2000
Connection ~ 5000 650 
Wire Bus Line
	5000 650  5300 650 
Entry Wire Line
	4900 1300 5000 1200
Wire Wire Line
	4600 1400 5400 1400
Text Label 4900 1300 2    50   ~ 0
5V
Wire Wire Line
	4600 1300 4900 1300
Text Label 3300 1300 0    50   ~ 0
3.3V
Wire Wire Line
	3500 1300 3300 1300
Entry Wire Line
	3300 1300 3200 1200
Connection ~ 3200 650 
Wire Bus Line
	3200 650  5000 650 
Entry Wire Line
	1700 2000 1800 1900
Entry Wire Line
	1700 2300 1800 2200
Wire Wire Line
	1200 2300 1700 2300
Text HLabel 3000 1700 0    50   Input ~ 0
SERVO1
NoConn ~ 4600 1800
Wire Wire Line
	1200 2100 2000 2100
Wire Wire Line
	2000 2100 2000 3600
Wire Wire Line
	2000 3600 2600 3600
Wire Wire Line
	1000 1000 1700 1000
Entry Wire Line
	1800 1100 1700 1000
Entry Wire Line
	1900 5700 1800 5600
Text Label 2050 5700 0    50   ~ 0
GND
Text HLabel 1000 1400 0    50   Input ~ 0
MISC_MOSI
Text HLabel 1000 4000 0    50   Output ~ 0
SER_OUT
Text HLabel 1000 1200 0    50   Input ~ 0
MISC_SCK
Text Label 1700 2800 2    50   ~ 0
GND
Entry Wire Line
	1700 2800 1800 2700
Wire Wire Line
	1200 2900 1600 2900
Wire Wire Line
	1600 2900 1600 4000
Wire Wire Line
	1600 4000 1000 4000
Text Label 1700 2600 2    50   ~ 0
3.3V
Wire Wire Line
	1200 2600 1700 2600
Entry Wire Line
	1700 2600 1800 2500
Wire Wire Line
	1200 2200 1300 2200
Wire Wire Line
	1300 2200 1300 1400
Wire Wire Line
	1300 1400 1000 1400
Wire Wire Line
	1000 1200 1500 1200
Wire Wire Line
	1200 2500 1500 2500
Wire Wire Line
	1500 1200 1500 2500
Wire Wire Line
	1200 2800 1700 2800
Text HLabel 1000 1300 0    50   Input ~ 0
MISC_NSS
Wire Wire Line
	1200 2400 1400 2400
Wire Wire Line
	1400 2400 1400 1300
Wire Wire Line
	1400 1300 1000 1300
Text Notes 3650 1100 0    50   ~ 0
3.3V to 5V Level Shifter
Wire Wire Line
	1900 4200 2600 4200
Connection ~ 2600 4200
Entry Wire Line
	2400 3000 2300 2900
Connection ~ 2300 2350
Entry Wire Line
	4900 2000 5000 2100
Wire Bus Line
	2300 2350 3200 2350
Wire Bus Line
	2300 2350 2300 1600
Wire Bus Line
	2300 1600 1800 1600
Connection ~ 1800 1600
Entry Wire Line
	4000 3000 4100 2900
Text Label 4000 3000 2    50   ~ 0
5V
Wire Wire Line
	3700 3000 4000 3000
Connection ~ 4100 2350
Wire Bus Line
	4100 2350 5000 2350
Text Notes 6300 4100 0    50   ~ 0
Sonar Connectors
Text Notes 6700 600  0    50   ~ 0
Servos
Text Notes 2150 600  0    50   ~ 0
Power Bus
Text Notes 550  4200 0    50   ~ 0
3.3V to 5V Level Shifter
Text Notes 600  1750 0    50   ~ 0
8-Bit Serial to\nParallel Shift\nRegister.
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F06A3
P 4600 4400
AR Path="/5F7F06A3" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F06A3" Ref="CN70"  Part="1" 
F 0 "CN70" H 5000 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 5050 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 4800 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 4800 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 4800 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 4800 4150 60  0001 L CNN "Field5"
	1    4600 4400
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F066C
P 3400 4400
AR Path="/5F7F066C" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F066C" Ref="CN60"  Part="1" 
F 0 "CN60" H 3800 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 3850 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 3600 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 3600 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 3600 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 3600 4150 60  0001 L CNN "Field5"
	1    3400 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3500 1900 2200 1900
Wire Wire Line
	2200 1900 2200 3000
Entry Wire Line
	4000 3700 4100 3800
Text Label 4000 3700 2    50   ~ 0
GND
Wire Wire Line
	4600 2000 4900 2000
Entry Wire Line
	3200 2100 3300 2000
Wire Wire Line
	3300 2000 3500 2000
Connection ~ 3200 2350
Wire Bus Line
	3200 2350 4100 2350
Text Label 4900 2000 2    50   ~ 0
GND
Wire Wire Line
	3700 3700 4000 3700
Entry Wire Line
	1900 3700 1800 3600
Wire Wire Line
	1900 3700 2600 3700
Text Label 1900 3700 0    50   ~ 0
GND
Text Notes 500  3900 0    50   ~ 0
Note:\nQH is temp. No Connect.
NoConn ~ 1200 2700
Wire Bus Line
	5000 2000 5000 2350
Wire Bus Line
	2300 2350 2300 3000
Wire Bus Line
	3200 650  3200 1300
Wire Bus Line
	5000 650  5000 1300
Wire Bus Line
	3200 2000 3200 2350
Wire Bus Line
	5300 650  5300 1100
Wire Bus Line
	6300 650  6300 1100
Wire Bus Line
	7300 650  7300 1100
Wire Bus Line
	8250 650  8250 1100
Wire Bus Line
	4100 2350 4100 3900
Wire Bus Line
	1800 650  1800 1600
Wire Bus Line
	1800 1600 1800 7200
$EndSCHEMATC
