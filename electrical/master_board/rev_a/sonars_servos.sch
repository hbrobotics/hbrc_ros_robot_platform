EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 9 10
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
P 4450 2600
F 0 "U10" H 4700 2750 50  0000 C CNN
F 1 "CD450B;TTSOP16" H 5000 1750 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 4650 2650 60  0001 L CNN
F 3 "Hex Level Shifter" H 4650 2450 60  0001 L CNN
F 4 "Hex Level Shifter" H 4650 2350 60  0001 L CNN "Field5"
	1    4450 2600
	1    0    0    -1  
$EndComp
Text Label 1650 800  0    50   ~ 0
3.3V
Text Label 1650 1300 0    50   ~ 0
GND
Text Label 4450 3000 2    50   ~ 0
TRIG4
Text Label 4450 3100 2    50   ~ 0
TRIG5
Text Label 4450 3200 2    50   ~ 0
TRIG6
Text Label 4450 1900 2    50   ~ 0
TRIG7
Text Label 6100 2700 2    50   ~ 0
TRIG1_5V
Text Label 6100 2900 2    50   ~ 0
TRIG3_5V
Text Label 6100 3000 2    50   ~ 0
TRIG4_5V
Text Label 6100 3100 2    50   ~ 0
TRIG5_5V
Text Label 6100 1900 2    50   ~ 0
TRIG7_5V
Text Notes 4600 2400 0    50   ~ 0
3.3V to 5V Level Shifter
Text Notes 6300 3800 0    50   ~ 0
Notes:\n1. The Sonars are HC04's that operate off of 5V.\n2. The trigger signals are provided by a serial shift register.\n3. A level shifter is used to shift trigger signals from 3.3V to 5V.\n4. A resistor voltage divider is used to convert the echo from 5V to 3.3V.\n5. Most hobby servers operate off of approximately 5V - 6V.\n6. The P5V supply is a 5V supply that is used for servos and LED's.\n   This supply can be noisy due to changing current draw.\n7. The servo PWM signals are level shifted from 3.3V to 5v.\n8. The servos are manually labeled to support mechanical\n   automatic positioned derived from the mechanical model.\n9. The echo signals are fed to external interrupt pins on the Nucleo PCB.
$Comp
L Device:C C17
U 1 1 5F7FE24F
P 2050 1050
F 0 "C17" H 1850 1150 50  0000 L CNN
F 1 "10pF;1608" H 2100 950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2088 900 50  0001 C CNN
F 3 "~" H 2050 1050 50  0001 C CNN
	1    2050 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 900  2050 800 
Wire Wire Line
	2050 800  1600 800 
Wire Wire Line
	2050 1200 2050 1300
Wire Wire Line
	2050 1300 1600 1300
Entry Wire Line
	1600 800  1500 900 
Entry Wire Line
	1600 1300 1500 1200
$Comp
L Device:C C18
U 1 1 5F8B3EAF
P 2650 1050
F 0 "C18" H 2450 1150 50  0000 L CNN
F 1 "10pF;1608" H 2700 950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2688 900 50  0001 C CNN
F 3 "~" H 2650 1050 50  0001 C CNN
	1    2650 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 5F8B4468
P 3250 1050
F 0 "C19" H 3050 1150 50  0000 L CNN
F 1 "10pF;1608" H 3300 950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3288 900 50  0001 C CNN
F 3 "~" H 3250 1050 50  0001 C CNN
	1    3250 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 800  2650 800 
Connection ~ 2050 800 
Wire Wire Line
	2650 900  2650 800 
Connection ~ 2650 800 
Wire Wire Line
	2650 800  3250 800 
Wire Wire Line
	3250 900  3250 800 
Wire Wire Line
	2050 1300 2650 1300
Connection ~ 2050 1300
Wire Wire Line
	3250 1200 3250 1300
Wire Wire Line
	2650 1200 2650 1300
Connection ~ 2650 1300
Wire Wire Line
	2650 1300 3250 1300
Text Notes 2200 750  0    50   ~ 0
Filter Capacitors
Text Label 4150 2000 0    50   ~ 0
GND
Text Label 6100 2800 2    50   ~ 0
TRIG2_5V
Text Label 6100 3200 2    50   ~ 0
TRIG6_5V
Text Label 4450 2900 2    50   ~ 0
TRIG3
Text Label 4450 2800 2    50   ~ 0
TRIG2
Text Label 4450 2700 2    50   ~ 0
TRIG1
Text HLabel 1000 1600 0    50   Input ~ 0
SERVO2
Text HLabel 1000 1500 0    50   Input ~ 0
SERVO3
Text HLabel 1000 1400 0    50   Input ~ 0
SERVO4
Text Label 1000 1700 0    50   ~ 0
SERVO1
Text Label 1000 1600 0    50   ~ 0
SERVO2
Text Label 1000 1500 0    50   ~ 0
SERVO3
Text Label 1000 1400 0    50   ~ 0
SERVO4
Text Label 5550 1700 0    50   ~ 0
SERVO1_5V
Text Label 5550 1600 0    50   ~ 0
SERVO2_5V
Text Label 5550 1500 0    50   ~ 0
SERVO3_5V
Text Label 5550 1400 0    50   ~ 0
SERVO4_5V
Wire Wire Line
	6500 1000 6300 1000
Wire Wire Line
	6500 1100 6300 1100
Wire Wire Line
	7350 1500 7350 1200
Wire Wire Line
	7350 1200 7500 1200
Wire Wire Line
	8350 1600 8350 1200
Wire Wire Line
	8350 1200 8500 1200
Wire Wire Line
	9300 1700 9300 1200
Wire Wire Line
	9300 1200 9500 1200
Wire Wire Line
	3500 3000 4450 3000
Text Label 6300 1000 0    50   ~ 0
SGND
Text Label 6300 1100 0    50   ~ 0
P5V
Entry Wire Line
	6300 1100 6200 1000
Entry Wire Line
	6200 900  6300 1000
Wire Wire Line
	7500 1000 7300 1000
Wire Wire Line
	7500 1100 7300 1100
Text Label 7300 1000 0    50   ~ 0
SGND
Text Label 7300 1100 0    50   ~ 0
P5V
Entry Wire Line
	7300 1100 7200 1000
Entry Wire Line
	7200 900  7300 1000
Wire Wire Line
	8500 1000 8300 1000
Wire Wire Line
	8500 1100 8300 1100
Text Label 8300 1000 0    50   ~ 0
SGND
Text Label 8300 1100 0    50   ~ 0
P5V
Entry Wire Line
	8300 1100 8200 1000
Entry Wire Line
	8200 900  8300 1000
Wire Wire Line
	9500 1000 9300 1000
Wire Wire Line
	9500 1100 9300 1100
Text Label 9300 1000 0    50   ~ 0
SGND
Text Label 9300 1100 0    50   ~ 0
P5V
Entry Wire Line
	9300 1100 9200 1000
Entry Wire Line
	9200 900  9300 1000
Wire Bus Line
	9200 650  8200 650 
Connection ~ 8200 650 
Wire Bus Line
	8200 650  7200 650 
Connection ~ 7200 650 
Wire Bus Line
	7200 650  6200 650 
Connection ~ 6200 650 
Wire Wire Line
	4450 2600 4100 2600
Text Label 4100 2600 0    50   ~ 0
3.3V
Wire Wire Line
	6300 1200 6500 1200
Wire Wire Line
	6300 1400 6300 1200
Wire Wire Line
	5550 1500 7350 1500
Wire Wire Line
	5550 1600 8350 1600
Wire Wire Line
	5550 1700 9300 1700
NoConn ~ 4450 1800
Wire Wire Line
	1000 1700 4450 1700
Wire Wire Line
	1000 1600 4450 1600
Wire Wire Line
	1000 1500 4450 1500
Wire Wire Line
	1000 1400 4450 1400
$Comp
L HR2:CD450B;TTSOP16 U9
U 1 1 5F67B87E
P 4450 1300
F 0 "U9" H 4700 1450 50  0000 C CNN
F 1 "CD450B;TTSOP16" H 5000 450 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 4650 1350 60  0001 L CNN
F 3 "Hex Level Shifter" H 4650 1150 60  0001 L CNN
F 4 "Hex Level Shifter" H 4650 1050 60  0001 L CNN "Field5"
	1    4450 1300
	1    0    0    -1  
$EndComp
Entry Wire Line
	6100 1300 6200 1200
Wire Wire Line
	5550 1400 6300 1400
Text Label 5850 1300 2    50   ~ 0
5V
Wire Wire Line
	5550 1300 6100 1300
Text Label 4150 1300 0    50   ~ 0
3.3V
Wire Wire Line
	4450 1300 4100 1300
Entry Wire Line
	4100 1300 4000 1200
Wire Bus Line
	4000 650  6200 650 
Text HLabel 1000 1700 0    50   Input ~ 0
SERVO1
NoConn ~ 5550 1800
Wire Wire Line
	3500 3200 4450 3200
Text HLabel 1000 2900 0    50   Input ~ 0
DIO_MOSI
Text HLabel 1000 3900 0    50   Output ~ 0
SER_OUT
Text HLabel 1000 3000 0    50   Input ~ 0
DIO_SCK
Text HLabel 1000 3100 0    50   Input ~ 0
~DIO_NSS
Text Notes 4600 1100 0    50   ~ 0
3.3V to 5V Level Shifter
Entry Wire Line
	5650 2150 5550 2250
Entry Wire Line
	5900 2600 6000 2500
Text Label 5800 2600 2    50   ~ 0
5V
Text Notes 7650 600  0    50   ~ 0
Servos
Text Notes 1850 600  0    50   ~ 0
Power Bus
Text Label 4100 3600 0    50   ~ 0
GND
Entry Wire Line
	4000 2100 4100 2000
Wire Wire Line
	4100 2000 4450 2000
Connection ~ 4000 2250
Text Label 5650 2150 2    50   ~ 0
GND
Entry Wire Line
	1600 3300 1500 3400
Text Label 1600 3300 0    50   ~ 0
GND
Wire Wire Line
	3500 3100 4450 3100
Wire Wire Line
	5550 2600 5900 2600
Wire Wire Line
	5700 3300 5550 3300
Wire Wire Line
	5650 2150 5650 2000
Wire Wire Line
	5650 2000 5550 2000
Wire Wire Line
	3500 3300 3600 3300
Wire Wire Line
	3600 3300 3600 1900
Wire Wire Line
	3600 1900 4450 1900
Entry Wire Line
	4000 2500 4100 2600
Entry Wire Line
	4000 2500 3900 2600
Text Label 3900 2600 2    50   ~ 0
3.3V
Entry Wire Line
	3900 3600 4000 3500
Wire Wire Line
	3500 3600 3900 3600
Text Label 3900 3600 2    50   ~ 0
GND
Wire Wire Line
	3500 3400 3600 3400
Wire Wire Line
	3600 3400 3600 3900
Wire Wire Line
	1000 3900 3600 3900
Text Label 1000 2900 0    50   ~ 0
DIO_MOSI
Text Label 1000 3000 0    50   ~ 0
DIO_SCK
Text Label 1000 3100 0    50   ~ 0
~DIO_NSS
Text Label 1000 3900 0    50   ~ 0
SER_IO
Text Label 1600 3200 0    50   ~ 0
~NRST
Entry Wire Line
	1500 3300 1600 3200
Wire Bus Line
	4000 650  1500 650 
Connection ~ 4000 650 
Wire Bus Line
	1500 2250 4000 2250
Connection ~ 1500 2250
Text Notes 5300 3950 0    50   ~ 0
5 Volt Trigger Bus
Text Label 1400 5000 2    50   ~ 0
~NRST
Wire Wire Line
	1000 5000 1400 5000
Entry Wire Line
	1400 5000 1500 4900
Text HLabel 1000 5000 0    50   Input ~ 0
~NRST
Text Label 9200 4500 0    50   ~ 0
TRIG7_5V
Text Label 8000 4500 0    50   ~ 0
TRIG6_5V
Text Label 6800 4500 0    50   ~ 0
TRIG5_5V
Text Label 5600 4500 0    50   ~ 0
TRIG4_5V
Text Label 4400 4500 0    50   ~ 0
TRIG3_5V
Text Label 3200 4500 0    50   ~ 0
TRIG2_5V
Text Label 2000 4500 0    50   ~ 0
TRIG1_5V
Wire Wire Line
	4400 4500 4400 4100
Wire Wire Line
	3200 4500 3200 4100
Wire Wire Line
	2000 4500 2000 4100
Text Label 2000 4100 2    50   ~ 0
TRIG1_5V
Text Label 3200 4100 2    50   ~ 0
TRIG2_5V
Text Label 4400 4100 2    50   ~ 0
TRIG3_5V
Wire Wire Line
	5600 4500 5600 4100
Text Label 5600 4100 2    50   ~ 0
TRIG4_5V
Text Label 6800 4100 0    50   ~ 0
TRIG5_5V
Wire Wire Line
	6800 4500 6800 4100
Wire Wire Line
	9200 4500 9200 4100
Wire Wire Line
	8000 4500 8000 4100
Text Label 8000 4100 0    50   ~ 0
TRIG6_5V
Entry Wire Line
	2000 4100 2100 4000
Entry Wire Line
	3200 4100 3300 4000
Entry Wire Line
	4500 4000 4400 4100
Entry Wire Line
	5700 4000 5600 4100
Entry Wire Line
	7900 4000 8000 4100
Entry Wire Line
	6700 4000 6800 4100
Entry Wire Line
	9100 4000 9200 4100
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F066C
P 3100 4400
AR Path="/5F7F066C" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F066C" Ref="CN81"  Part="1" 
F 0 "CN81" H 3500 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 3550 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4H" H 3300 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 3300 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 3300 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 3300 4150 60  0001 L CNN "Field5"
	1    3100 4400
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F06A3
P 4300 4400
AR Path="/5F7F06A3" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F06A3" Ref="CN82"  Part="1" 
F 0 "CN82" H 4700 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 4750 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4H" H 4500 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 4500 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 4500 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 4500 4150 60  0001 L CNN "Field5"
	1    4300 4400
	-1   0    0    -1  
$EndComp
Text Notes 5900 4100 0    50   ~ 0
Sonar Connectors
Connection ~ 2300 4200
Wire Wire Line
	1600 4200 2300 4200
Text Label 1750 5700 0    50   ~ 0
GND
Entry Wire Line
	1600 5700 1500 5600
Entry Wire Line
	1500 4800 1400 4900
Wire Wire Line
	1000 4900 1400 4900
Text Label 1600 4200 0    50   ~ 0
5V
Entry Wire Line
	1600 4200 1500 4300
Entry Wire Line
	1500 4700 1400 4800
Entry Wire Line
	1500 4600 1400 4700
Entry Wire Line
	1500 4500 1400 4600
Wire Wire Line
	1000 4800 1400 4800
Text Label 1400 4700 2    50   ~ 0
P5V
Wire Wire Line
	1400 4700 1000 4700
Text HLabel 1000 4700 0    50   Input ~ 0
P5V
Wire Wire Line
	1700 5200 1700 5900
Text Label 1000 5900 0    50   ~ 0
ECHO1
Text Label 2900 5200 0    50   ~ 0
ECHO2
Text Label 4100 5200 0    50   ~ 0
ECHO3
Text Label 5300 5200 0    50   ~ 0
ECHO4
Text Label 6500 5200 0    50   ~ 0
ECHO5
Text Label 7700 5200 0    50   ~ 0
ECHO6
Text Label 8900 5200 0    50   ~ 0
ECHO7
Text HLabel 1000 5900 0    50   Output ~ 0
ECHO1
Text HLabel 1000 6000 0    50   Output ~ 0
ECHO2
Text HLabel 1000 6100 0    50   Output ~ 0
ECHO3
Text HLabel 1000 6200 0    50   Output ~ 0
ECHO4
Text HLabel 1000 6300 0    50   Output ~ 0
ECHO5
Text HLabel 1000 6400 0    50   Output ~ 0
ECHO6
Text HLabel 1000 6500 0    50   Output ~ 0
ECHO7
Text Label 9200 4100 0    50   ~ 0
TRIG7_5V
Wire Wire Line
	4900 5700 5600 5700
Wire Wire Line
	4700 4200 5900 4200
Wire Wire Line
	3500 4200 4700 4200
Wire Wire Line
	3700 5700 4400 5700
Wire Wire Line
	4400 5200 4400 5100
Connection ~ 4400 5200
Wire Wire Line
	4400 5200 4100 5200
Connection ~ 4900 5700
Wire Wire Line
	4900 4900 4900 5700
Wire Wire Line
	4700 4900 4900 4900
Connection ~ 4400 5700
Wire Wire Line
	4800 4500 4400 4500
Connection ~ 4700 4200
Wire Wire Line
	4400 5700 4900 5700
Wire Wire Line
	4400 5600 4400 5700
Wire Wire Line
	4700 4400 4700 4200
Wire Wire Line
	4800 4400 4700 4400
Wire Wire Line
	4700 4700 4700 4900
Wire Wire Line
	4800 4700 4700 4700
Wire Wire Line
	4400 5300 4400 5200
Wire Wire Line
	4400 4600 4800 4600
Wire Wire Line
	4400 4600 4400 4800
Text Label 4400 4600 0    50   ~ 0
ECHO3_5V
$Comp
L Device:R_US R28
U 1 1 5F7F06EE
P 4400 5450
F 0 "R28" H 4450 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 3950 5550 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4440 5440 50  0001 C CNN
F 3 "~" H 4400 5450 50  0001 C CNN
	1    4400 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R27
U 1 1 5F7F06E4
P 4400 4950
F 0 "R27" H 4450 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 3950 5050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4440 4940 50  0001 C CNN
F 3 "~" H 4400 4950 50  0001 C CNN
	1    4400 4950
	-1   0    0    1   
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7F06DA
P 5500 4400
AR Path="/5F7F06DA" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7F06DA" Ref="CN83"  Part="1" 
F 0 "CN83" H 5900 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 5950 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4" H 5700 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5700 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5700 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 5700 4150 60  0001 L CNN "Field5"
	1    5500 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2300 4200 3500 4200
Wire Wire Line
	2500 5700 3200 5700
Wire Wire Line
	3200 5200 3200 5100
Connection ~ 3200 5200
Wire Wire Line
	3200 5200 2900 5200
Connection ~ 3700 5700
Wire Wire Line
	3700 4900 3700 5700
Wire Wire Line
	3500 4900 3700 4900
Connection ~ 3200 5700
Wire Wire Line
	3600 4500 3200 4500
Connection ~ 3500 4200
Wire Wire Line
	3200 5700 3700 5700
Wire Wire Line
	3200 5600 3200 5700
Wire Wire Line
	3500 4400 3500 4200
Wire Wire Line
	3600 4400 3500 4400
Wire Wire Line
	3500 4700 3500 4900
Wire Wire Line
	3600 4700 3500 4700
Wire Wire Line
	3200 5300 3200 5200
Wire Wire Line
	3200 4600 3600 4600
Wire Wire Line
	3200 4600 3200 4800
Text Label 3200 4600 0    50   ~ 0
ECHO2_5V
$Comp
L Device:R_US R26
U 1 1 5F7F06B7
P 3200 5450
F 0 "R26" H 3250 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 2750 5550 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3240 5440 50  0001 C CNN
F 3 "~" H 3200 5450 50  0001 C CNN
	1    3200 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R25
U 1 1 5F7F06AD
P 3200 4950
F 0 "R25" H 3250 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 2750 5050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3240 4940 50  0001 C CNN
F 3 "~" H 3200 4950 50  0001 C CNN
	1    3200 4950
	-1   0    0    1   
$EndComp
Wire Wire Line
	2000 5200 2000 5100
Connection ~ 2000 5200
Wire Wire Line
	2000 5200 1700 5200
Connection ~ 2500 5700
Wire Wire Line
	2500 4900 2500 5700
Wire Wire Line
	2300 4900 2500 4900
Connection ~ 2000 5700
Wire Wire Line
	2000 5700 1600 5700
Wire Wire Line
	2400 4500 2000 4500
Wire Wire Line
	2000 5700 2500 5700
Wire Wire Line
	2000 5600 2000 5700
Wire Wire Line
	2300 4400 2300 4200
Wire Wire Line
	2400 4400 2300 4400
Wire Wire Line
	2300 4700 2300 4900
Wire Wire Line
	2400 4700 2300 4700
Wire Wire Line
	2000 5300 2000 5200
Wire Wire Line
	2000 4600 2400 4600
Wire Wire Line
	2000 4600 2000 4800
Text Label 2000 4600 0    50   ~ 0
ECHO1_5V
$Comp
L Device:R_US R24
U 1 1 5F7F0680
P 2000 5450
F 0 "R24" H 2050 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 1550 5550 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2040 5440 50  0001 C CNN
F 3 "~" H 2000 5450 50  0001 C CNN
	1    2000 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R23
U 1 1 5F7F0676
P 2000 4950
F 0 "R23" H 2050 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 1550 5050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2040 4940 50  0001 C CNN
F 3 "~" H 2000 4950 50  0001 C CNN
	1    2000 4950
	-1   0    0    1   
$EndComp
Wire Wire Line
	7100 4200 8300 4200
Wire Wire Line
	7300 5700 8000 5700
Wire Wire Line
	8300 4200 9500 4200
Wire Wire Line
	8500 5700 9200 5700
Wire Wire Line
	9200 5200 9200 5100
Connection ~ 9200 5200
Wire Wire Line
	9200 5200 8900 5200
Wire Wire Line
	9750 4900 9750 5700
Wire Wire Line
	9500 4900 9750 4900
Connection ~ 9200 5700
Wire Wire Line
	9600 4500 9200 4500
Wire Wire Line
	9200 5700 9750 5700
Wire Wire Line
	9200 5600 9200 5700
Wire Wire Line
	9500 4400 9500 4200
Wire Wire Line
	9600 4400 9500 4400
Wire Wire Line
	9500 4700 9500 4900
Wire Wire Line
	9600 4700 9500 4700
Wire Wire Line
	9200 5300 9200 5200
Wire Wire Line
	9200 4600 9600 4600
Wire Wire Line
	9200 4600 9200 4800
Text Label 9200 4600 0    50   ~ 0
ECHO7_5V
$Comp
L Device:R_US R36
U 1 1 5F7E7575
P 9200 5450
F 0 "R36" H 9250 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 8750 5550 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9240 5440 50  0001 C CNN
F 3 "~" H 9200 5450 50  0001 C CNN
	1    9200 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R35
U 1 1 5F7E756B
P 9200 4950
F 0 "R35" H 9250 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 8750 5050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9240 4940 50  0001 C CNN
F 3 "~" H 9200 4950 50  0001 C CNN
	1    9200 4950
	-1   0    0    1   
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E7561
P 10300 4400
AR Path="/5F7E7561" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E7561" Ref="CN87"  Part="1" 
F 0 "CN87" H 10700 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 10750 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4" H 10500 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 10500 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 10500 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 10500 4150 60  0001 L CNN "Field5"
	1    10300 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8000 5200 8000 5100
Connection ~ 8000 5200
Wire Wire Line
	8000 5200 7700 5200
Connection ~ 8500 5700
Wire Wire Line
	8500 4900 8500 5700
Wire Wire Line
	8300 4900 8500 4900
Connection ~ 8000 5700
Wire Wire Line
	8400 4500 8000 4500
Connection ~ 8300 4200
Wire Wire Line
	8000 5700 8500 5700
Wire Wire Line
	8000 5600 8000 5700
Wire Wire Line
	8300 4400 8300 4200
Wire Wire Line
	8400 4400 8300 4400
Wire Wire Line
	8300 4700 8300 4900
Wire Wire Line
	8400 4700 8300 4700
Wire Wire Line
	8000 5300 8000 5200
Wire Wire Line
	8000 4600 8400 4600
Wire Wire Line
	8000 4600 8000 4800
Text Label 8000 4600 0    50   ~ 0
ECHO6_5V
$Comp
L Device:R_US R34
U 1 1 5F7E753E
P 8000 5450
F 0 "R34" H 8050 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 7550 5550 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8040 5440 50  0001 C CNN
F 3 "~" H 8000 5450 50  0001 C CNN
	1    8000 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R33
U 1 1 5F7E7534
P 8000 4950
F 0 "R33" H 8050 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 7550 5050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8040 4940 50  0001 C CNN
F 3 "~" H 8000 4950 50  0001 C CNN
	1    8000 4950
	-1   0    0    1   
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E752A
P 9100 4400
AR Path="/5F7E752A" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E752A" Ref="CN86"  Part="1" 
F 0 "CN86" H 9500 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 9550 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 9300 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 9300 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 9300 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 9300 4150 60  0001 L CNN "Field5"
	1    9100 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5900 4200 7100 4200
Wire Wire Line
	6100 5700 6800 5700
Wire Wire Line
	6800 5200 6800 5100
Connection ~ 6800 5200
Wire Wire Line
	6800 5200 6500 5200
Connection ~ 7300 5700
Wire Wire Line
	7300 4900 7300 5700
Wire Wire Line
	7100 4900 7300 4900
Connection ~ 6800 5700
Wire Wire Line
	7200 4500 6800 4500
Connection ~ 7100 4200
Wire Wire Line
	6800 5700 7300 5700
Wire Wire Line
	6800 5600 6800 5700
Wire Wire Line
	7100 4400 7100 4200
Wire Wire Line
	7200 4400 7100 4400
Wire Wire Line
	7100 4700 7100 4900
Wire Wire Line
	7200 4700 7100 4700
Wire Wire Line
	6800 5300 6800 5200
Wire Wire Line
	6800 4600 7200 4600
Wire Wire Line
	6800 4600 6800 4800
Text Label 6800 4600 0    50   ~ 0
ECHO5_5V
$Comp
L Device:R_US R32
U 1 1 5F7E2DCE
P 6800 5450
F 0 "R32" H 6850 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 6350 5550 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6840 5440 50  0001 C CNN
F 3 "~" H 6800 5450 50  0001 C CNN
	1    6800 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R31
U 1 1 5F7E2DC4
P 6800 4950
F 0 "R31" H 6850 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 6350 5050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6840 4940 50  0001 C CNN
F 3 "~" H 6800 4950 50  0001 C CNN
	1    6800 4950
	-1   0    0    1   
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F7E2DBA
P 7900 4400
AR Path="/5F7E2DBA" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F7E2DBA" Ref="CN85"  Part="1" 
F 0 "CN85" H 8300 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 8350 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 8100 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8100 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8100 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 8100 4150 60  0001 L CNN "Field5"
	1    7900 4400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5600 5200 5600 5100
Connection ~ 5600 5200
Wire Wire Line
	5600 5200 5300 5200
Connection ~ 6100 5700
Wire Wire Line
	6100 4900 6100 5700
Wire Wire Line
	5900 4900 6100 4900
Connection ~ 5600 5700
Wire Wire Line
	6000 4500 5600 4500
Connection ~ 5900 4200
Wire Wire Line
	5600 5700 6100 5700
Wire Wire Line
	5600 5600 5600 5700
Wire Wire Line
	5900 4400 5900 4200
Wire Wire Line
	6000 4400 5900 4400
Wire Wire Line
	5900 4700 5900 4900
Wire Wire Line
	6000 4700 5900 4700
Wire Wire Line
	5600 5300 5600 5200
Wire Wire Line
	5600 4600 6000 4600
Wire Wire Line
	5600 4600 5600 4800
Text Label 5600 4600 0    50   ~ 0
ECHO4_5V
$Comp
L Device:R_US R30
U 1 1 5FADC37B
P 5600 5450
F 0 "R30" H 5650 5350 50  0000 L CNN
F 1 "33KΩ;1608" H 5150 5550 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5640 5440 50  0001 C CNN
F 3 "~" H 5600 5450 50  0001 C CNN
	1    5600 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R29
U 1 1 5FADB964
P 5600 4950
F 0 "R29" H 5650 4850 50  0000 L CNN
F 1 "22KΩ;1608" H 5150 5050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5640 4940 50  0001 C CNN
F 3 "~" H 5600 4950 50  0001 C CNN
	1    5600 4950
	-1   0    0    1   
$EndComp
Text Label 1400 4800 2    50   ~ 0
3.3V
Text Label 1400 4900 2    50   ~ 0
GND
Text Label 1400 4600 2    50   ~ 0
5V
Wire Wire Line
	1000 4600 1400 4600
Text HLabel 1000 4800 0    50   Input ~ 0
3.3V
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F490633
P 6700 4400
AR Path="/5F490633" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490633" Ref="CN84"  Part="1" 
F 0 "CN84" H 7100 4550 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 7150 3950 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 6900 4450 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 6900 4250 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 6900 4350 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 6900 4150 60  0001 L CNN "Field5"
	1    6700 4400
	-1   0    0    -1  
$EndComp
Text HLabel 1000 4600 0    50   Input ~ 0
5V
Text HLabel 1000 4900 0    50   Input ~ 0
GND
Wire Wire Line
	1000 5900 1700 5900
Wire Wire Line
	1000 6000 2900 6000
Wire Wire Line
	2900 5200 2900 6000
Wire Wire Line
	1000 6100 4100 6100
Wire Wire Line
	4100 5200 4100 6100
Wire Wire Line
	1000 6200 5300 6200
Wire Wire Line
	5300 5200 5300 6200
Wire Wire Line
	1000 6300 6500 6300
Wire Wire Line
	6500 5200 6500 6300
Wire Wire Line
	1000 6400 7700 6400
Wire Wire Line
	7700 5200 7700 6400
Wire Wire Line
	1000 6500 8900 6500
Wire Wire Line
	8900 5200 8900 6500
Entry Wire Line
	4000 3500 4100 3600
Wire Wire Line
	3500 2900 4450 2900
Wire Wire Line
	3500 2800 4450 2800
Wire Wire Line
	3500 2700 4450 2700
Wire Wire Line
	3500 2600 3900 2600
Text Notes 2000 2800 0    50   ~ 0
8-Bit Serial to\nParallel Shift\nRegister.
Wire Wire Line
	1600 3300 2400 3300
Wire Wire Line
	1600 3200 2400 3200
Wire Wire Line
	2400 3100 1000 3100
Wire Wire Line
	1000 3000 2400 3000
Wire Wire Line
	1000 2900 2400 2900
$Comp
L HR2:SN74HCS595;TTSOP16 U8
U 1 1 5FF3477F
P 2400 2600
F 0 "U8" H 2650 2750 60  0000 C CNN
F 1 "SN74HCS595;TTSOP16" H 3000 1450 60  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 2600 2650 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 2600 2450 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 2600 2350 60  0001 L CNN "Field5"
	1    2400 2600
	1    0    0    -1  
$EndComp
NoConn ~ 3500 3500
Wire Wire Line
	4100 3600 4350 3600
Wire Wire Line
	4350 3600 4350 3300
Wire Wire Line
	4350 3300 4450 3300
Wire Wire Line
	4350 3600 5700 3600
Connection ~ 4350 3600
Wire Wire Line
	5700 3300 5700 3600
Wire Wire Line
	5550 2700 6100 2700
Wire Wire Line
	5550 2800 6100 2800
Wire Wire Line
	5550 2900 6100 2900
Wire Wire Line
	5550 3000 6100 3000
Wire Wire Line
	5550 3100 6100 3100
Wire Wire Line
	5550 3200 6100 3200
Connection ~ 6200 4000
Entry Wire Line
	6100 2700 6200 2800
Entry Wire Line
	6100 2800 6200 2900
Entry Wire Line
	6100 2900 6200 3000
Entry Wire Line
	6100 3000 6200 3100
Entry Wire Line
	6100 3100 6200 3200
Entry Wire Line
	6100 3200 6200 3300
Entry Wire Line
	6100 1900 6200 2000
Wire Wire Line
	5550 1900 6100 1900
Text Label 1700 5200 0    50   ~ 0
ECHO1
Text Label 1000 6000 0    50   ~ 0
ECHO2
Text Label 1000 6500 0    50   ~ 0
ECHO7
Text Label 1000 6400 0    50   ~ 0
ECHO6
Text Label 1000 6300 0    50   ~ 0
ECHO5
Text Label 1000 6200 0    50   ~ 0
ECHO4
Text Label 1000 6100 0    50   ~ 0
ECHO3
$Comp
L HR2:ACS711;SOIC8 U19
U 1 1 5FC1C86E
P 7600 2100
F 0 "U19" H 7850 2250 50  0000 C CNN
F 1 "ACS711;SOIC8" H 8100 1650 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7800 2150 60  0001 L CNN
F 3 "Hall Effect Current Sensor" H 7800 1950 60  0001 L CNN
F 4 "Hall Effect Current Sensor" H 7800 1850 60  0001 L CNN "Field5"
	1    7600 2100
	1    0    0    -1  
$EndComp
$Comp
L HR2:SERVO;M1x3 CN6
U 1 1 5FC1D721
P 7200 1000
F 0 "CN6" H 7700 1150 50  0000 R CNN
F 1 "SERVO;M1x3" H 7800 650 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 7400 1050 60  0001 L CNN
F 3 "Servo M1X3" H 7400 850 60  0001 L CNN
F 4 "Servo F1X3" H 7400 750 60  0001 L CNN "Field5"
	1    7200 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;M1x4 CN9
U 1 1 5FC1E3A4
P 10200 1000
F 0 "CN9" H 10650 1150 50  0000 C CNN
F 1 "SERVO;M1x4" H 10550 550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10400 1050 60  0001 L CNN
F 3 "Servo M1X4" H 10400 850 60  0001 L CNN
F 4 "Servo M1X4" H 10400 750 60  0001 L CNN "Field5"
	1    10200 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;M1x3 CN7
U 1 1 5FC20D64
P 8200 1000
F 0 "CN7" H 8700 1150 50  0000 R CNN
F 1 "SERVO;M1x3" H 8800 650 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 8400 1050 60  0001 L CNN
F 3 "Servo M1X3" H 8400 850 60  0001 L CNN
F 4 "Servo F1X3" H 8400 750 60  0001 L CNN "Field5"
	1    8200 1000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;M1x3 CN8
U 1 1 5FC214E7
P 9200 1000
F 0 "CN8" H 9700 1150 50  0000 R CNN
F 1 "SERVO;M1x3" H 9800 650 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 9400 1050 60  0001 L CNN
F 3 "Servo M1X3" H 9400 850 60  0001 L CNN
F 4 "Servo F1X3" H 9400 750 60  0001 L CNN "Field5"
	1    9200 1000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7600 2100 7500 2100
Wire Wire Line
	7600 2400 7500 2400
Wire Wire Line
	7600 2200 7500 2200
Wire Wire Line
	7500 2200 7500 2100
Connection ~ 7500 2100
Wire Wire Line
	7500 2100 7300 2100
Wire Wire Line
	7600 2300 7500 2300
Wire Wire Line
	7500 2300 7500 2400
Connection ~ 7500 2400
Wire Wire Line
	7500 2400 7300 2400
Entry Wire Line
	7200 2000 7300 2100
Entry Wire Line
	7200 2300 7300 2400
Text Label 7300 2100 0    50   ~ 0
SGND
Text Label 7300 2400 0    50   ~ 0
GND
Entry Wire Line
	9200 2000 9100 2100
Entry Wire Line
	9200 2300 9100 2400
Wire Wire Line
	8700 2100 9100 2100
Wire Wire Line
	8700 2400 9100 2400
NoConn ~ 8700 2300
Text Label 9100 2400 2    50   ~ 0
GND
Text Label 9100 2100 2    50   ~ 0
3.3V
Wire Wire Line
	9500 1300 9400 1300
Wire Wire Line
	9400 1300 9400 1700
Wire Wire Line
	9400 1700 10400 1700
Wire Wire Line
	10400 1700 10400 6700
Wire Wire Line
	10400 6700 1000 6700
Wire Wire Line
	8700 2200 10300 2200
Wire Wire Line
	10300 2200 10300 6600
Wire Wire Line
	10300 6600 1000 6600
Text HLabel 1000 6600 0    50   Output ~ 0
SERVO_CUR
Text HLabel 1000 6700 0    50   Output ~ 0
SERVO_POS
Wire Bus Line
	4000 2250 6000 2250
Wire Bus Line
	4000 2000 4000 2250
Wire Bus Line
	6000 2250 6000 2600
Wire Bus Line
	4000 650  4000 1300
Wire Bus Line
	1500 650  1500 2250
Wire Bus Line
	8200 650  8200 1100
Wire Bus Line
	4000 2250 4000 3600
Wire Bus Line
	7200 650  7200 2400
Wire Bus Line
	6200 650  6200 1300
Wire Bus Line
	2000 4000 6200 4000
Wire Bus Line
	6200 4000 9200 4000
Wire Bus Line
	9200 650  9200 2400
Wire Bus Line
	6200 1900 6200 4000
Wire Bus Line
	1500 2250 1500 5700
$EndSCHEMATC
