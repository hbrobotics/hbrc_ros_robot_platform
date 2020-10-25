EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 8 10
Title "HR2 Servos and Sonars"
Date "2020-10-17"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L HR2:CD450B;TTSOP16 U10
U 1 1 5F56033D
P 4750 2600
F 0 "U10" H 5000 2750 50  0000 C CNN
F 1 "CD450B;TTSOP16" H 5300 1750 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 4950 2650 60  0001 L CNN
F 3 "Hex Level Shifter" H 4950 2450 60  0001 L CNN
F 4 "Hex Level Shifter" H 4950 2350 60  0001 L CNN "Field5"
	1    4750 2600
	1    0    0    -1  
$EndComp
Text Label 1950 800  0    50   ~ 0
3.3V
Text Label 1950 1300 0    50   ~ 0
GND
Text Label 4750 3000 2    50   ~ 0
TRIG4
Text Label 4750 3100 2    50   ~ 0
TRIG5
Text Label 4750 3200 2    50   ~ 0
TRIG6
Text Label 4750 1900 2    50   ~ 0
TRIG7
Text Label 6400 2700 2    50   ~ 0
TRIG1_5V
Text Label 6400 2900 2    50   ~ 0
TRIG3_5V
Text Label 6400 3000 2    50   ~ 0
TRIG4_5V
Text Label 6400 3100 2    50   ~ 0
TRIG5_5V
Text Label 6400 1900 2    50   ~ 0
TRIG7_5V
Text Notes 4900 2400 0    50   ~ 0
3.3V to 5V Level Shifter
Text Notes 7050 3250 0    50   ~ 0
Notes:\n1. The Sonars are HC04's that operate off of 5V.\n2. The trigger signals are provided by a serial shift register.\n3. A level shifter is used to shift trigger signals from 3.3V to 5V.\n4. A resistor voltage divider is used to convert the echo from 5V to 3.3V.\n5. Most hobby servers operate off of approximately 5V - 6V.\n6. The P5V supply is a 5V supply that is used for servos and LED's.\n   This supply can be noisy due to changing current draw.\n7. The servo PWM signals are level shifted from 3.3V to 5v.\n8. The servos are manually labeled to support mechanical\n   automatic positioned derived from the mechanical model.\n9. The echo signals are fed to external interrupt pins on the Nucleo PCB.
$Comp
L Device:C C17
U 1 1 5F7FE24F
P 2350 1050
F 0 "C17" H 2150 1150 50  0000 L CNN
F 1 "10pF;1608" H 2400 950 50  0000 L CNN
F 2 "" H 2388 900 50  0001 C CNN
F 3 "~" H 2350 1050 50  0001 C CNN
	1    2350 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 900  2350 800 
Wire Wire Line
	2350 800  1900 800 
Wire Wire Line
	2350 1200 2350 1300
Wire Wire Line
	2350 1300 1900 1300
Entry Wire Line
	1900 800  1800 900 
Entry Wire Line
	1900 1300 1800 1200
$Comp
L Device:C C18
U 1 1 5F8B3EAF
P 2950 1050
F 0 "C18" H 2750 1150 50  0000 L CNN
F 1 "10pF;1608" H 3000 950 50  0000 L CNN
F 2 "" H 2988 900 50  0001 C CNN
F 3 "~" H 2950 1050 50  0001 C CNN
	1    2950 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 5F8B4468
P 3550 1050
F 0 "C19" H 3350 1150 50  0000 L CNN
F 1 "10pF;1608" H 3600 950 50  0000 L CNN
F 2 "" H 3588 900 50  0001 C CNN
F 3 "~" H 3550 1050 50  0001 C CNN
	1    3550 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 800  2950 800 
Connection ~ 2350 800 
Wire Wire Line
	2950 900  2950 800 
Connection ~ 2950 800 
Wire Wire Line
	2950 800  3550 800 
Wire Wire Line
	3550 900  3550 800 
Wire Wire Line
	2350 1300 2950 1300
Connection ~ 2350 1300
Wire Wire Line
	3550 1200 3550 1300
Wire Wire Line
	2950 1200 2950 1300
Connection ~ 2950 1300
Wire Wire Line
	2950 1300 3550 1300
Text Notes 2500 750  0    50   ~ 0
Filter Capacitors
Text Label 4450 2000 0    50   ~ 0
GND
Text Label 6400 2800 2    50   ~ 0
TRIG2_5V
Text Label 6400 3200 2    50   ~ 0
TRIG6_5V
Text Label 4750 2900 2    50   ~ 0
TRIG3
Text Label 4750 2800 2    50   ~ 0
TRIG2
Text Label 4750 2700 2    50   ~ 0
TRIG1
Text HLabel 1300 1600 0    50   Input ~ 0
SERVO2
Text HLabel 1300 1500 0    50   Input ~ 0
SERVO3
Text HLabel 1300 1400 0    50   Input ~ 0
SERVO4
Text Label 1300 1700 0    50   ~ 0
SERVO1
Text Label 1300 1600 0    50   ~ 0
SERVO2
Text Label 1300 1500 0    50   ~ 0
SERVO3
Text Label 1300 1400 0    50   ~ 0
SERVO4
Text Label 5850 1700 0    50   ~ 0
SERVO1_5V
Text Label 5850 1600 0    50   ~ 0
SERVO2_5V
Text Label 5850 1500 0    50   ~ 0
SERVO3_5V
Text Label 5850 1400 0    50   ~ 0
SERVO4_5V
$Comp
L HR2:SERVO;F1x3 CN6
U 1 1 5FB31019
P 7550 1000
F 0 "CN6" H 8050 1150 60  0000 R CNN
F 1 "SERVO;F1x3" H 8200 650 60  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 7750 1050 60  0001 L CNN
F 3 "Servo F1X3" H 7750 850 60  0001 L CNN
F 4 "Servo F1X3" H 7750 950 60  0001 L CNN "manf#"
	1    7550 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;F1x3 CN7
U 1 1 5FB9CF1F
P 8550 1000
F 0 "CN7" H 9050 1150 60  0000 R CNN
F 1 "SERVO;F1x3" H 9200 650 60  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 8750 1050 60  0001 L CNN
F 3 "Servo F1X3" H 8750 850 60  0001 L CNN
F 4 "Servo F1X3" H 8750 950 60  0001 L CNN "manf#"
	1    8550 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;F1x3 CN8
U 1 1 5FB9D556
P 9550 1000
F 0 "CN8" H 10050 1150 60  0000 R CNN
F 1 "SERVO;F1x3" H 10200 650 60  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 9750 1050 60  0001 L CNN
F 3 "Servo F1X3" H 9750 850 60  0001 L CNN
F 4 "Servo F1X3" H 9750 950 60  0001 L CNN "manf#"
	1    9550 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;F1x3 CN9
U 1 1 5FB9D857
P 10500 1000
F 0 "CN9" H 11000 1150 60  0000 R CNN
F 1 "SERVO;F1x3" H 11150 650 60  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10700 1050 60  0001 L CNN
F 3 "Servo F1X3" H 10700 850 60  0001 L CNN
F 4 "Servo F1X3" H 10700 950 60  0001 L CNN "manf#"
	1    10500 1000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6850 1000 6650 1000
Wire Wire Line
	6850 1100 6650 1100
Wire Wire Line
	7650 1500 7650 1200
Wire Wire Line
	7650 1200 7850 1200
Wire Wire Line
	8650 1600 8650 1200
Wire Wire Line
	8650 1200 8850 1200
Wire Wire Line
	9600 1700 9600 1200
Wire Wire Line
	9600 1200 9800 1200
Wire Wire Line
	3800 3000 4750 3000
Text Label 6650 1000 0    50   ~ 0
GND
Text Label 6650 1100 0    50   ~ 0
P5V
Entry Wire Line
	6650 1100 6550 1000
Entry Wire Line
	6550 900  6650 1000
Wire Wire Line
	7850 1000 7650 1000
Wire Wire Line
	7850 1100 7650 1100
Text Label 7650 1000 0    50   ~ 0
GND
Text Label 7650 1100 0    50   ~ 0
P5V
Entry Wire Line
	7650 1100 7550 1000
Entry Wire Line
	7550 900  7650 1000
Wire Wire Line
	8850 1000 8650 1000
Wire Wire Line
	8850 1100 8650 1100
Text Label 8650 1000 0    50   ~ 0
GND
Text Label 8650 1100 0    50   ~ 0
P5V
Entry Wire Line
	8650 1100 8550 1000
Entry Wire Line
	8550 900  8650 1000
Wire Wire Line
	9800 1000 9600 1000
Wire Wire Line
	9800 1100 9600 1100
Text Label 9600 1000 0    50   ~ 0
GND
Text Label 9600 1100 0    50   ~ 0
P5V
Entry Wire Line
	9600 1100 9500 1000
Entry Wire Line
	9500 900  9600 1000
Wire Bus Line
	9500 650  8550 650 
Connection ~ 8550 650 
Wire Bus Line
	8550 650  7550 650 
Connection ~ 7550 650 
Wire Bus Line
	7550 650  6550 650 
Connection ~ 6550 650 
Wire Wire Line
	4750 2600 4450 2600
Text Label 4450 2600 0    50   ~ 0
3.3V
Wire Wire Line
	6650 1200 6850 1200
Wire Wire Line
	6650 1400 6650 1200
Wire Wire Line
	5850 1500 7650 1500
Wire Wire Line
	5850 1600 8650 1600
Wire Wire Line
	5850 1700 9600 1700
NoConn ~ 4750 1800
Wire Wire Line
	1300 1700 4750 1700
Wire Wire Line
	1300 1600 4750 1600
Wire Wire Line
	1300 1500 4750 1500
Wire Wire Line
	1300 1400 4750 1400
$Comp
L HR2:CD450B;TTSOP16 U9
U 1 1 5F67B87E
P 4750 1300
F 0 "U9" H 5000 1450 50  0000 C CNN
F 1 "CD450B;TTSOP16" H 5300 450 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 4950 1350 60  0001 L CNN
F 3 "Hex Level Shifter" H 4950 1150 60  0001 L CNN
F 4 "Hex Level Shifter" H 4950 1050 60  0001 L CNN "Field5"
	1    4750 1300
	1    0    0    -1  
$EndComp
Entry Wire Line
	6450 1300 6550 1200
Wire Wire Line
	5850 1400 6650 1400
Text Label 6150 1300 2    50   ~ 0
5V
Wire Wire Line
	5850 1300 6450 1300
Text Label 4550 1300 0    50   ~ 0
3.3V
Wire Wire Line
	4750 1300 4450 1300
Entry Wire Line
	4450 1300 4350 1200
Wire Bus Line
	4350 650  6550 650 
Text HLabel 1300 1700 0    50   Input ~ 0
SERVO1
NoConn ~ 5850 1800
Wire Wire Line
	3800 3200 4750 3200
Text HLabel 1300 2900 0    50   Input ~ 0
DIO_MOSI
Text HLabel 1300 3900 0    50   Output ~ 0
SER_OUT
Text HLabel 1300 3000 0    50   Input ~ 0
DIO_SCK
Text HLabel 1300 3100 0    50   Input ~ 0
~DIO_NSS
Text Notes 4900 1100 0    50   ~ 0
3.3V to 5V Level Shifter
Entry Wire Line
	5950 2150 5850 2250
Entry Wire Line
	6200 2600 6300 2500
Text Label 6100 2600 2    50   ~ 0
5V
Text Notes 7950 600  0    50   ~ 0
Servos
Text Notes 2150 600  0    50   ~ 0
Power Bus
Text Label 4450 3600 0    50   ~ 0
GND
Entry Wire Line
	4350 2100 4450 2000
Wire Wire Line
	4450 2000 4750 2000
Connection ~ 4350 2250
Text Label 5950 2150 2    50   ~ 0
GND
Entry Wire Line
	1900 3300 1800 3400
Text Label 1900 3300 0    50   ~ 0
GND
Wire Wire Line
	3800 3100 4750 3100
Wire Wire Line
	5850 2600 6200 2600
Wire Wire Line
	6000 3300 5850 3300
Wire Wire Line
	5950 2150 5950 2000
Wire Wire Line
	5950 2000 5850 2000
Wire Wire Line
	3800 3300 3900 3300
Wire Wire Line
	3900 3300 3900 1900
Wire Wire Line
	3900 1900 4750 1900
Entry Wire Line
	4350 2500 4450 2600
Entry Wire Line
	4350 2500 4250 2600
Text Label 4250 2600 2    50   ~ 0
3.3V
Entry Wire Line
	4250 3600 4350 3500
Wire Wire Line
	3800 3600 4250 3600
Text Label 4250 3600 2    50   ~ 0
GND
Wire Wire Line
	3800 3400 3900 3400
Wire Wire Line
	3900 3400 3900 3900
Wire Wire Line
	1300 3900 3900 3900
Text Label 1300 2900 0    50   ~ 0
DIO_MOSI
Text Label 1300 3000 0    50   ~ 0
DIO_SCK
Text Label 1300 3100 0    50   ~ 0
~DIO_NSS
Text Label 1300 3900 0    50   ~ 0
SER_IO
Text Label 1900 3200 0    50   ~ 0
~NRST
Entry Wire Line
	1800 3300 1900 3200
Wire Bus Line
	4350 650  1800 650 
Connection ~ 4350 650 
Wire Bus Line
	1800 2250 4350 2250
Connection ~ 1800 2250
Text Notes 5600 3950 0    50   ~ 0
5 Volt Trigger Bus
Text Label 1700 5000 2    50   ~ 0
~NRST
Wire Wire Line
	1300 5000 1700 5000
Entry Wire Line
	1700 5000 1800 4900
Text HLabel 1300 5000 0    50   Input ~ 0
~NRST
Text Label 9500 4500 0    50   ~ 0
TRIG7_5V
Text Label 8300 4500 0    50   ~ 0
TRIG6_5V
Text Label 7100 4500 0    50   ~ 0
TRIG5_5V
Text Label 5900 4500 0    50   ~ 0
TRIG4_5V
Text Label 4700 4500 0    50   ~ 0
TRIG3_5V
Text Label 3500 4500 0    50   ~ 0
TRIG2_5V
Text Label 2300 4500 0    50   ~ 0
TRIG1_5V
Wire Wire Line
	4700 4500 4700 4100
Wire Wire Line
	3500 4500 3500 4100
Wire Wire Line
	2300 4500 2300 4100
Text Label 2300 4100 2    50   ~ 0
TRIG1_5V
Text Label 3500 4100 2    50   ~ 0
TRIG2_5V
Text Label 4700 4100 2    50   ~ 0
TRIG3_5V
Wire Wire Line
	5900 4500 5900 4100
Text Label 5900 4100 2    50   ~ 0
TRIG4_5V
Text Label 7100 4100 0    50   ~ 0
TRIG5_5V
Wire Wire Line
	7100 4500 7100 4100
Wire Wire Line
	9500 4500 9500 4100
Wire Wire Line
	8300 4500 8300 4100
Text Label 8300 4100 0    50   ~ 0
TRIG6_5V
Entry Wire Line
	2300 4100 2400 4000
Entry Wire Line
	3500 4100 3600 4000
Entry Wire Line
	4800 4000 4700 4100
Entry Wire Line
	6000 4000 5900 4100
Entry Wire Line
	8200 4000 8300 4100
Entry Wire Line
	7000 4000 7100 4100
Entry Wire Line
	9400 4000 9500 4100
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F066C
P 3400 4400
AR Path="/5F7F066C" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F066C" Ref="CN81"  Part="1" 
F 0 "CN81" H 3800 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 3850 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 3600 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 3600 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 3600 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 3600 4150 60  0001 L CNN "Field5"
	1    3400 4400
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F06A3
P 4600 4400
AR Path="/5F7F06A3" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F06A3" Ref="CN82"  Part="1" 
F 0 "CN82" H 5000 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 5050 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 4800 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 4800 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 4800 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 4800 4150 60  0001 L CNN "Field5"
	1    4600 4400
	-1   0    0    -1  
$EndComp
Text Notes 6200 4100 0    50   ~ 0
Sonar Connectors
Connection ~ 2600 4200
Wire Wire Line
	1900 4200 2600 4200
Text Label 2050 5700 0    50   ~ 0
GND
Entry Wire Line
	1900 5700 1800 5600
Entry Wire Line
	1800 4800 1700 4900
Wire Wire Line
	1300 4900 1700 4900
Text Label 1900 4200 0    50   ~ 0
5V
Entry Wire Line
	1900 4200 1800 4300
Entry Wire Line
	1800 4700 1700 4800
Entry Wire Line
	1800 4600 1700 4700
Entry Wire Line
	1800 4500 1700 4600
Wire Wire Line
	1300 4800 1700 4800
Text Label 1700 4700 2    50   ~ 0
P5V
Wire Wire Line
	1700 4700 1300 4700
Text HLabel 1300 4700 0    50   Input ~ 0
P5V
Wire Wire Line
	2000 5200 2000 5900
Text Label 1300 5900 0    50   ~ 0
ECHO1
Text Label 3200 5200 0    50   ~ 0
ECHO2
Text Label 4400 5200 0    50   ~ 0
ECHO3
Text Label 5600 5200 0    50   ~ 0
ECHO4
Text Label 6800 5200 0    50   ~ 0
ECHO5
Text Label 8000 5200 0    50   ~ 0
ECHO6
Text Label 9200 5200 0    50   ~ 0
ECHO7
Text HLabel 1300 5900 0    50   Output ~ 0
ECHO1
Text HLabel 1300 6000 0    50   Output ~ 0
ECHO2
Text HLabel 1300 6100 0    50   Output ~ 0
ECHO3
Text HLabel 1300 6200 0    50   Output ~ 0
ECHO4
Text HLabel 1300 6300 0    50   Output ~ 0
ECHO5
Text HLabel 1300 6400 0    50   Output ~ 0
ECHO6
Text HLabel 1300 6500 0    50   Output ~ 0
ECHO7
Text Label 9500 4100 0    50   ~ 0
TRIG7_5V
Wire Wire Line
	5200 5700 5900 5700
Wire Wire Line
	5000 4200 6200 4200
Wire Wire Line
	3800 4200 5000 4200
Wire Wire Line
	4000 5700 4700 5700
Wire Wire Line
	4700 5200 4700 5100
Connection ~ 4700 5200
Wire Wire Line
	4700 5200 4400 5200
Connection ~ 5200 5700
Wire Wire Line
	5200 4900 5200 5700
Wire Wire Line
	5000 4900 5200 4900
Connection ~ 4700 5700
Wire Wire Line
	5100 4500 4700 4500
Connection ~ 5000 4200
Wire Wire Line
	4700 5700 5200 5700
Wire Wire Line
	4700 5600 4700 5700
Wire Wire Line
	5000 4400 5000 4200
Wire Wire Line
	5100 4400 5000 4400
Wire Wire Line
	5000 4700 5000 4900
Wire Wire Line
	5100 4700 5000 4700
Wire Wire Line
	4700 5300 4700 5200
Wire Wire Line
	4700 4600 5100 4600
Wire Wire Line
	4700 4600 4700 4800
Text Label 4700 4600 0    50   ~ 0
ECHO3_5V
$Comp
L Device:R_US R28
U 1 1 5F7F06EE
P 4700 5450
F 0 "R28" H 4750 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 4250 5550 50  0000 L CNN
F 2 "" V 4740 5440 50  0001 C CNN
F 3 "~" H 4700 5450 50  0001 C CNN
	1    4700 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R27
U 1 1 5F7F06E4
P 4700 4950
F 0 "R27" H 4750 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 4250 5050 50  0000 L CNN
F 2 "" V 4740 4940 50  0001 C CNN
F 3 "~" H 4700 4950 50  0001 C CNN
	1    4700 4950
	-1   0    0    1   
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F06DA
P 5800 4400
AR Path="/5F7F06DA" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F06DA" Ref="CN83"  Part="1" 
F 0 "CN83" H 6200 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 6250 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 6000 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 6000 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 6000 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 6000 4150 60  0001 L CNN "Field5"
	1    5800 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2600 4200 3800 4200
Wire Wire Line
	2800 5700 3500 5700
Wire Wire Line
	3500 5200 3500 5100
Connection ~ 3500 5200
Wire Wire Line
	3500 5200 3200 5200
Connection ~ 4000 5700
Wire Wire Line
	4000 4900 4000 5700
Wire Wire Line
	3800 4900 4000 4900
Connection ~ 3500 5700
Wire Wire Line
	3900 4500 3500 4500
Connection ~ 3800 4200
Wire Wire Line
	3500 5700 4000 5700
Wire Wire Line
	3500 5600 3500 5700
Wire Wire Line
	3800 4400 3800 4200
Wire Wire Line
	3900 4400 3800 4400
Wire Wire Line
	3800 4700 3800 4900
Wire Wire Line
	3900 4700 3800 4700
Wire Wire Line
	3500 5300 3500 5200
Wire Wire Line
	3500 4600 3900 4600
Wire Wire Line
	3500 4600 3500 4800
Text Label 3500 4600 0    50   ~ 0
ECHO2_5V
$Comp
L Device:R_US R26
U 1 1 5F7F06B7
P 3500 5450
F 0 "R26" H 3550 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 3050 5550 50  0000 L CNN
F 2 "" V 3540 5440 50  0001 C CNN
F 3 "~" H 3500 5450 50  0001 C CNN
	1    3500 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R25
U 1 1 5F7F06AD
P 3500 4950
F 0 "R25" H 3550 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 3050 5050 50  0000 L CNN
F 2 "" V 3540 4940 50  0001 C CNN
F 3 "~" H 3500 4950 50  0001 C CNN
	1    3500 4950
	-1   0    0    1   
$EndComp
Wire Wire Line
	2300 5200 2300 5100
Connection ~ 2300 5200
Wire Wire Line
	2300 5200 2000 5200
Connection ~ 2800 5700
Wire Wire Line
	2800 4900 2800 5700
Wire Wire Line
	2600 4900 2800 4900
Connection ~ 2300 5700
Wire Wire Line
	2300 5700 1900 5700
Wire Wire Line
	2700 4500 2300 4500
Wire Wire Line
	2300 5700 2800 5700
Wire Wire Line
	2300 5600 2300 5700
Wire Wire Line
	2600 4400 2600 4200
Wire Wire Line
	2700 4400 2600 4400
Wire Wire Line
	2600 4700 2600 4900
Wire Wire Line
	2700 4700 2600 4700
Wire Wire Line
	2300 5300 2300 5200
Wire Wire Line
	2300 4600 2700 4600
Wire Wire Line
	2300 4600 2300 4800
Text Label 2300 4600 0    50   ~ 0
ECHO1_5V
$Comp
L Device:R_US R24
U 1 1 5F7F0680
P 2300 5450
F 0 "R24" H 2350 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 1850 5550 50  0000 L CNN
F 2 "" V 2340 5440 50  0001 C CNN
F 3 "~" H 2300 5450 50  0001 C CNN
	1    2300 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R23
U 1 1 5F7F0676
P 2300 4950
F 0 "R23" H 2350 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 1850 5050 50  0000 L CNN
F 2 "" V 2340 4940 50  0001 C CNN
F 3 "~" H 2300 4950 50  0001 C CNN
	1    2300 4950
	-1   0    0    1   
$EndComp
Wire Wire Line
	7400 4200 8600 4200
Wire Wire Line
	7600 5700 8300 5700
Wire Wire Line
	8600 4200 9800 4200
Wire Wire Line
	8800 5700 9500 5700
Wire Wire Line
	9500 5200 9500 5100
Connection ~ 9500 5200
Wire Wire Line
	9500 5200 9200 5200
Wire Wire Line
	10050 4900 10050 5700
Wire Wire Line
	9800 4900 10050 4900
Connection ~ 9500 5700
Wire Wire Line
	9900 4500 9500 4500
Wire Wire Line
	9500 5700 10050 5700
Wire Wire Line
	9500 5600 9500 5700
Wire Wire Line
	9800 4400 9800 4200
Wire Wire Line
	9900 4400 9800 4400
Wire Wire Line
	9800 4700 9800 4900
Wire Wire Line
	9900 4700 9800 4700
Wire Wire Line
	9500 5300 9500 5200
Wire Wire Line
	9500 4600 9900 4600
Wire Wire Line
	9500 4600 9500 4800
Text Label 9500 4600 0    50   ~ 0
ECHO7_5V
$Comp
L Device:R_US R36
U 1 1 5F7E7575
P 9500 5450
F 0 "R36" H 9550 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 9050 5550 50  0000 L CNN
F 2 "" V 9540 5440 50  0001 C CNN
F 3 "~" H 9500 5450 50  0001 C CNN
	1    9500 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R35
U 1 1 5F7E756B
P 9500 4950
F 0 "R35" H 9550 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 9050 5050 50  0000 L CNN
F 2 "" V 9540 4940 50  0001 C CNN
F 3 "~" H 9500 4950 50  0001 C CNN
	1    9500 4950
	-1   0    0    1   
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E7561
P 10600 4400
AR Path="/5F7E7561" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E7561" Ref="CN87"  Part="1" 
F 0 "CN87" H 11000 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 11050 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 10800 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 10800 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 10800 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 10800 4150 60  0001 L CNN "Field5"
	1    10600 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8300 5200 8300 5100
Connection ~ 8300 5200
Wire Wire Line
	8300 5200 8000 5200
Connection ~ 8800 5700
Wire Wire Line
	8800 4900 8800 5700
Wire Wire Line
	8600 4900 8800 4900
Connection ~ 8300 5700
Wire Wire Line
	8700 4500 8300 4500
Connection ~ 8600 4200
Wire Wire Line
	8300 5700 8800 5700
Wire Wire Line
	8300 5600 8300 5700
Wire Wire Line
	8600 4400 8600 4200
Wire Wire Line
	8700 4400 8600 4400
Wire Wire Line
	8600 4700 8600 4900
Wire Wire Line
	8700 4700 8600 4700
Wire Wire Line
	8300 5300 8300 5200
Wire Wire Line
	8300 4600 8700 4600
Wire Wire Line
	8300 4600 8300 4800
Text Label 8300 4600 0    50   ~ 0
ECHO6_5V
$Comp
L Device:R_US R34
U 1 1 5F7E753E
P 8300 5450
F 0 "R34" H 8350 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 7850 5550 50  0000 L CNN
F 2 "" V 8340 5440 50  0001 C CNN
F 3 "~" H 8300 5450 50  0001 C CNN
	1    8300 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R33
U 1 1 5F7E7534
P 8300 4950
F 0 "R33" H 8350 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 7850 5050 50  0000 L CNN
F 2 "" V 8340 4940 50  0001 C CNN
F 3 "~" H 8300 4950 50  0001 C CNN
	1    8300 4950
	-1   0    0    1   
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E752A
P 9400 4400
AR Path="/5F7E752A" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E752A" Ref="CN86"  Part="1" 
F 0 "CN86" H 9800 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 9850 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 9600 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 9600 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 9600 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 9600 4150 60  0001 L CNN "Field5"
	1    9400 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6200 4200 7400 4200
Wire Wire Line
	6400 5700 7100 5700
Wire Wire Line
	7100 5200 7100 5100
Connection ~ 7100 5200
Wire Wire Line
	7100 5200 6800 5200
Connection ~ 7600 5700
Wire Wire Line
	7600 4900 7600 5700
Wire Wire Line
	7400 4900 7600 4900
Connection ~ 7100 5700
Wire Wire Line
	7500 4500 7100 4500
Connection ~ 7400 4200
Wire Wire Line
	7100 5700 7600 5700
Wire Wire Line
	7100 5600 7100 5700
Wire Wire Line
	7400 4400 7400 4200
Wire Wire Line
	7500 4400 7400 4400
Wire Wire Line
	7400 4700 7400 4900
Wire Wire Line
	7500 4700 7400 4700
Wire Wire Line
	7100 5300 7100 5200
Wire Wire Line
	7100 4600 7500 4600
Wire Wire Line
	7100 4600 7100 4800
Text Label 7100 4600 0    50   ~ 0
ECHO5_5V
$Comp
L Device:R_US R32
U 1 1 5F7E2DCE
P 7100 5450
F 0 "R32" H 7150 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 6650 5550 50  0000 L CNN
F 2 "" V 7140 5440 50  0001 C CNN
F 3 "~" H 7100 5450 50  0001 C CNN
	1    7100 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R31
U 1 1 5F7E2DC4
P 7100 4950
F 0 "R31" H 7150 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 6650 5050 50  0000 L CNN
F 2 "" V 7140 4940 50  0001 C CNN
F 3 "~" H 7100 4950 50  0001 C CNN
	1    7100 4950
	-1   0    0    1   
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E2DBA
P 8200 4400
AR Path="/5F7E2DBA" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E2DBA" Ref="CN85"  Part="1" 
F 0 "CN85" H 8600 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 8650 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 8400 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8400 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8400 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 8400 4150 60  0001 L CNN "Field5"
	1    8200 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5900 5200 5900 5100
Connection ~ 5900 5200
Wire Wire Line
	5900 5200 5600 5200
Connection ~ 6400 5700
Wire Wire Line
	6400 4900 6400 5700
Wire Wire Line
	6200 4900 6400 4900
Connection ~ 5900 5700
Wire Wire Line
	6300 4500 5900 4500
Connection ~ 6200 4200
Wire Wire Line
	5900 5700 6400 5700
Wire Wire Line
	5900 5600 5900 5700
Wire Wire Line
	6200 4400 6200 4200
Wire Wire Line
	6300 4400 6200 4400
Wire Wire Line
	6200 4700 6200 4900
Wire Wire Line
	6300 4700 6200 4700
Wire Wire Line
	5900 5300 5900 5200
Wire Wire Line
	5900 4600 6300 4600
Wire Wire Line
	5900 4600 5900 4800
Text Label 5900 4600 0    50   ~ 0
ECHO4_5V
$Comp
L Device:R_US R30
U 1 1 5FADC37B
P 5900 5450
F 0 "R30" H 5950 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 5450 5550 50  0000 L CNN
F 2 "" V 5940 5440 50  0001 C CNN
F 3 "~" H 5900 5450 50  0001 C CNN
	1    5900 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R29
U 1 1 5FADB964
P 5900 4950
F 0 "R29" H 5950 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 5450 5050 50  0000 L CNN
F 2 "" V 5940 4940 50  0001 C CNN
F 3 "~" H 5900 4950 50  0001 C CNN
	1    5900 4950
	-1   0    0    1   
$EndComp
Text Label 1700 4800 2    50   ~ 0
3.3V
Text Label 1700 4900 2    50   ~ 0
GND
Text Label 1700 4600 2    50   ~ 0
5V
Wire Wire Line
	1300 4600 1700 4600
Text HLabel 1300 4800 0    50   Input ~ 0
3.3V
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F490633
P 7000 4400
AR Path="/5F490633" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490633" Ref="CN84"  Part="1" 
F 0 "CN84" H 7400 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 7450 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 7200 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 7200 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 7200 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 7200 4150 60  0001 L CNN "Field5"
	1    7000 4400
	-1   0    0    -1  
$EndComp
Text HLabel 1300 4600 0    50   Input ~ 0
5V
Text HLabel 1300 4900 0    50   Input ~ 0
GND
Wire Wire Line
	1300 5900 2000 5900
Wire Wire Line
	1300 6000 3200 6000
Wire Wire Line
	3200 5200 3200 6000
Wire Wire Line
	1300 6100 4400 6100
Wire Wire Line
	4400 5200 4400 6100
Wire Wire Line
	1300 6200 5600 6200
Wire Wire Line
	5600 5200 5600 6200
Wire Wire Line
	1300 6300 6800 6300
Wire Wire Line
	6800 5200 6800 6300
Wire Wire Line
	1300 6400 8000 6400
Wire Wire Line
	8000 5200 8000 6400
Wire Wire Line
	1300 6500 9200 6500
Wire Wire Line
	9200 5200 9200 6500
Entry Wire Line
	4350 3500 4450 3600
Wire Wire Line
	3800 2900 4750 2900
Wire Wire Line
	3800 2800 4750 2800
Wire Wire Line
	3800 2700 4750 2700
Wire Wire Line
	3800 2600 4250 2600
Text Notes 2300 2800 0    50   ~ 0
8-Bit Serial to\nParallel Shift\nRegister.
Wire Wire Line
	1900 3300 2700 3300
Wire Wire Line
	1900 3200 2700 3200
Wire Wire Line
	2700 3100 1300 3100
Wire Wire Line
	1300 3000 2700 3000
Wire Wire Line
	1300 2900 2700 2900
$Comp
L HR2:SN74HCS595;TTSOP16 U8
U 1 1 5FF3477F
P 2700 2600
F 0 "U8" H 2950 2750 60  0000 C CNN
F 1 "SN74HCS595;TTSOP16" H 3300 1450 60  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 2900 2650 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 2900 2450 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 2900 2350 60  0001 L CNN "Field5"
	1    2700 2600
	1    0    0    -1  
$EndComp
NoConn ~ 3800 3500
Wire Wire Line
	4450 3600 4650 3600
Wire Wire Line
	4650 3600 4650 3300
Wire Wire Line
	4650 3300 4750 3300
Wire Wire Line
	4650 3600 6000 3600
Connection ~ 4650 3600
Wire Wire Line
	6000 3300 6000 3600
Wire Wire Line
	5850 2700 6400 2700
Wire Wire Line
	5850 2800 6400 2800
Wire Wire Line
	5850 2900 6400 2900
Wire Wire Line
	5850 3000 6400 3000
Wire Wire Line
	5850 3100 6400 3100
Wire Wire Line
	5850 3200 6400 3200
Connection ~ 6500 4000
Entry Wire Line
	6400 2700 6500 2800
Entry Wire Line
	6400 2800 6500 2900
Entry Wire Line
	6400 2900 6500 3000
Entry Wire Line
	6400 3000 6500 3100
Entry Wire Line
	6400 3100 6500 3200
Entry Wire Line
	6400 3200 6500 3300
Entry Wire Line
	6400 1900 6500 2000
Wire Wire Line
	5850 1900 6400 1900
Text Label 2000 5200 0    50   ~ 0
ECHO1
Text Label 1300 6000 0    50   ~ 0
ECHO2
Text Label 1300 6500 0    50   ~ 0
ECHO7
Text Label 1300 6400 0    50   ~ 0
ECHO6
Text Label 1300 6300 0    50   ~ 0
ECHO5
Text Label 1300 6200 0    50   ~ 0
ECHO4
Text Label 1300 6100 0    50   ~ 0
ECHO3
Wire Bus Line
	4350 2250 6300 2250
Wire Bus Line
	4350 2000 4350 2250
Wire Bus Line
	6300 2250 6300 2600
Wire Bus Line
	4350 650  4350 1300
Wire Bus Line
	1800 650  1800 2250
Wire Bus Line
	7550 650  7550 1100
Wire Bus Line
	8550 650  8550 1100
Wire Bus Line
	9500 650  9500 1100
Wire Bus Line
	4350 2250 4350 3600
Wire Bus Line
	6550 650  6550 1300
Wire Bus Line
	2300 4000 6500 4000
Wire Bus Line
	6500 4000 9500 4000
Wire Bus Line
	6500 1900 6500 4000
Wire Bus Line
	1800 2250 1800 5700
$EndSCHEMATC
