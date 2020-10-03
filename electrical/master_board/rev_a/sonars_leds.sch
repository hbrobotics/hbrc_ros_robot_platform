EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 8 8
Title "HR2 Master Board"
Date "2020-10-03"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L HR2:SN74HCS595;TTSOP16 U4
U 1 1 5F559713
P 700 1400
F 0 "U4" H 950 1550 50  0000 C CNN
F 1 "SN74HCS595;TTSOP16" H 1100 -250 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 900 1450 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 900 1250 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 900 1150 60  0001 L CNN "Field5"
	1    700  1400
	1    0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NE_INNER;3xM1x1+M1x5 CN101
U 3 1 5F49BE0B
P 10500 2500
F 0 "CN101" H 11000 2650 50  0000 R CNN
F 1 "BRIDGE_NE_INNER;3xM1x1+M1x5" H 11800 1950 50  0000 R CNN
F 2 "HR2:BRIDGE_NE_INNER_3xM1x1+M1x5" H 10700 2550 60  0001 L CNN
F 3 "" H 10700 2350 60  0001 L CNN
F 4 "NE Inner Bridge Connectors" H 10700 2250 60  0001 L CNN "Field5"
	3    10500 2500
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NE_INNER;3xM1x1+M1x5 CN101
U 2 1 5F49BA1D
P 9050 1300
F 0 "CN101" H 9450 1450 50  0000 R CNN
F 1 "BRIDGE_NE_INNER;3xM1x1+M1x5" H 9950 1150 50  0000 R CNN
F 2 "HR2:BRIDGE_NE_INNER_3xM1x1+M1x5" H 9250 1350 60  0001 L CNN
F 3 "" H 9250 1150 60  0001 L CNN
F 4 "NE Inner Bridge Connectors" H 9250 1050 60  0001 L CNN "Field5"
	2    9050 1300
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NE_INNER;3xM1x1+M1x5 CN101
U 4 1 5F48CE60
P 9050 800
F 0 "CN101" H 9450 950 50  0000 R CNN
F 1 "BRIDGE_NE_INNER;3xM1x1+M1x5" H 10000 650 50  0000 R CNN
F 2 "HR2:BRIDGE_NE_INNER_3xM1x1+M1x5" H 9250 850 60  0001 L CNN
F 3 "" H 9250 650 60  0001 L CNN
F 4 "NE Inner Bridge Connectors" H 9250 550 60  0001 L CNN "Field5"
	4    9050 800 
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4200 4900 3400 4900
Wire Wire Line
	9800 5600 8100 5600
Wire Wire Line
	3400 1700 4950 1700
Wire Wire Line
	4200 4800 3400 4800
NoConn ~ 9800 6200
Wire Wire Line
	9800 5500 8100 5500
NoConn ~ 4950 2500
NoConn ~ 4950 2400
NoConn ~ 4950 2300
NoConn ~ 4950 2200
NoConn ~ 4950 2100
Text Label 4500 2000 0    50   ~ 0
ECHO2_5V
Text Label 4200 1800 0    50   ~ 0
TRIG2_5V
Text Label 4600 1600 0    50   ~ 0
LED6
Text Label 4600 1200 0    50   ~ 0
LED5
Text Label 4600 1100 0    50   ~ 0
LED4
Wire Wire Line
	4200 1800 4950 1800
NoConn ~ 4200 5500
Text Label 3900 5400 0    50   ~ 0
LED11
Text Label 3900 5300 0    50   ~ 0
LED10
Text Label 9300 6100 0    50   ~ 0
LED16
Text Label 9300 6000 0    50   ~ 0
LED15
Text Label 9300 5900 0    50   ~ 0
LED14
Text Label 9300 5800 0    50   ~ 0
LED13
Text Label 9300 2900 0    50   ~ 0
LED3
Text Label 9300 2800 0    50   ~ 0
LED2
Text Label 9300 2700 0    50   ~ 0
LED1
Text Label 9400 2500 0    50   ~ 0
ECHO1_5V
Text Label 8500 2600 0    50   ~ 0
TRIG1_5V
Text HLabel 9350 2100 2    50   Output ~ 0
ECHO1
Text Label 6900 1300 0    50   ~ 0
ECHO3_5V
Text Label 6000 1200 0    50   ~ 0
TRIG3_5V
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F4A9A9E
P 8000 5200
AR Path="/5F4A9A9E" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F4A9A9E" Ref="CN54"  Part="1" 
F 0 "CN54" H 8400 5350 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 8350 4750 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 8200 5250 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 5050 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 5150 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 8200 4950 60  0001 L CNN "Field5"
	1    8000 5200
	-1   0    0    -1  
$EndComp
Text HLabel 1000 1100 0    50   Input ~ 0
GND
Text HLabel 1000 900  0    50   Input ~ 0
5V
Text HLabel 1000 800  0    50   Input ~ 0
9V
Text HLabel 6700 1500 2    50   Output ~ 0
ECHO3
Text HLabel 4950 2850 2    50   Output ~ 0
ECHO2
$Comp
L HR2:BRIDGE_SW_INNER;M1x8 CN?
U 1 1 5F49069F
P 5000 4800
AR Path="/5F49069F" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F49069F" Ref="CN104"  Part="1" 
F 0 "CN104" H 5450 4950 50  0000 C CNN
F 1 "BRIDGE_SW_INNER;M1x8" H 5300 3950 50  0000 C CNN
F 2 "HR2:BRIDGE_SW_INNER_M1x8" H 5200 4850 60  0001 L CNN
F 3 "" H 5200 4650 60  0001 L CNN
F 4 "SW Inner Bridge Connector" H 5200 4550 60  0001 L CNN "Field5"
	1    5000 4800
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_SE_INNER;M1x8 CN?
U 1 1 5F490690
P 10600 5500
AR Path="/5F490690" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490690" Ref="CN103"  Part="1" 
F 0 "CN103" H 11050 5650 50  0000 C CNN
F 1 "BRIDGE_SE_INNER;M1x8" H 11250 4650 50  0000 C CNN
F 2 "HR2:BRIDGE_SE_INNER_M1x8" H 10800 5550 60  0001 L CNN
F 3 "" H 10800 5350 60  0001 L CNN
F 4 "SE Inner Bridge Connector" H 10800 5250 60  0001 L CNN "Field5"
	1    10600 5500
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NW_INNER;M1x2+M2x5 CN?
U 2 1 5F49067D
P 5650 1600
AR Path="/5F49067D" Ref="CN?"  Part="2" 
AR Path="/5F48CAAD/5F49067D" Ref="CN102"  Part="2" 
F 0 "CN102" H 6000 1750 50  0000 C CNN
F 1 "BRIDGE_NW_INNER;M1x2+M2x5" H 6300 550 50  0000 C CNN
F 2 "HR2:BRIDGE_NW_INNER_M1x2+M2x5" H 5850 1650 60  0001 L CNN
F 3 "" H 5850 1450 60  0001 L CNN
F 4 "NW Inner Bridge Connectors" H 5850 1350 60  0001 L CNN "Field5"
	2    5650 1600
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NW_INNER;M1x2+M2x5 CN?
U 1 1 5F490676
P 5650 1100
AR Path="/5F490676" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490676" Ref="CN102"  Part="1" 
F 0 "CN102" H 6000 1250 50  0000 C CNN
F 1 "BRIDGE_NW_INNER;M1x2+M2x5" H 6150 850 50  0000 C CNN
F 2 "HR2:BRIDGE_NW_INNER_M1x2+M2x5" H 5850 1150 60  0001 L CNN
F 3 "" H 5850 950 60  0001 L CNN
F 4 "NW Inner Bridge Connectors" H 5850 850 60  0001 L CNN "Field5"
	1    5650 1100
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F490643
P 8000 4100
AR Path="/5F490643" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490643" Ref="CN53"  Part="1" 
F 0 "CN53" H 8400 4250 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 8350 3650 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 8200 4150 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 3950 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 4050 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 8200 3850 60  0001 L CNN "Field5"
	1    8000 4100
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F49063B
P 8000 3100
AR Path="/5F49063B" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F49063B" Ref="CN52"  Part="1" 
F 0 "CN52" H 8400 3250 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 8350 2650 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 8200 3150 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 2950 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 3050 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 8200 2850 60  0001 L CNN "Field5"
	1    8000 3100
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F490633
P 8000 2100
AR Path="/5F490633" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490633" Ref="CN51"  Part="1" 
F 0 "CN51" H 8400 2250 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 8350 1650 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 8200 2150 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 1950 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 2050 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 8200 1850 60  0001 L CNN "Field5"
	1    8000 2100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9300 5700 9800 5700
Wire Wire Line
	9300 5800 9800 5800
Wire Wire Line
	9300 5900 9800 5900
Wire Wire Line
	9300 6000 9800 6000
Wire Wire Line
	9300 6100 9800 6100
NoConn ~ 1500 2200
$Comp
L HR2:CD450B;TTSOP16 U6
U 1 1 5F56033D
P 1200 5700
F 0 "U6" H 1450 5850 50  0000 C CNN
F 1 "CD450B;TTSOP16" H 1750 4850 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 1400 5750 60  0001 L CNN
F 3 "Hex Level Shifter" H 1400 5550 60  0001 L CNN
F 4 "Hex Level Shifter" H 1400 5450 60  0001 L CNN "Field5"
	1    1200 5700
	1    0    0    -1  
$EndComp
Text HLabel 1000 5100 0    50   Input ~ 0
LED_MOSI
Text HLabel 1000 5300 0    50   Input ~ 0
LED_SCLK
Text HLabel 1000 5200 0    50   Input ~ 0
LED_NSS
Text Label 9300 5700 0    50   ~ 0
LED12
Entry Wire Line
	9200 5900 9300 5800
Entry Wire Line
	9200 6000 9300 5900
Entry Wire Line
	9200 6100 9300 6000
Entry Wire Line
	9200 6200 9300 6100
Text HLabel 1000 1000 0    50   Input ~ 0
3.3V
Wire Wire Line
	1000 1100 1500 1100
Wire Wire Line
	1000 1000 1500 1000
Wire Wire Line
	1000 900  1500 900 
Wire Wire Line
	1000 800  1500 800 
Text Label 1500 800  2    50   ~ 0
9V
Text Label 1500 900  2    50   ~ 0
5V
Text Label 1500 1100 2    50   ~ 0
GND
Text Label 1500 1000 2    50   ~ 0
3.3V
Entry Wire Line
	1500 800  1600 700 
Entry Wire Line
	1500 900  1600 800 
Entry Wire Line
	1500 1000 1600 900 
Entry Wire Line
	1500 1100 1600 1000
Wire Bus Line
	1600 600  3300 600 
Text Label 8100 1300 0    50   ~ 0
5V
Entry Wire Line
	8100 1300 8000 1200
Entry Wire Line
	8100 800  8000 700 
Wire Wire Line
	8100 800  8450 800 
Wire Wire Line
	8100 1300 8450 1300
Text Label 8100 800  0    50   ~ 0
9V
$Comp
L HR2:SN74HCS595;TTSOP16 U5
U 1 1 5F5A832F
P 700 3300
F 0 "U5" H 950 3450 50  0000 C CNN
F 1 "SN74HCS595;TTSOP16" H 1100 1650 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 900 3350 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 900 3150 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 900 3050 60  0001 L CNN "Field5"
	1    700  3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 5100 1600 5100
Wire Wire Line
	1600 5100 1600 4600
Wire Wire Line
	1600 4600 1500 4600
Wire Wire Line
	1500 3900 1600 3900
Wire Wire Line
	1600 3900 1600 2700
Wire Wire Line
	1600 2700 1500 2700
NoConn ~ 1500 4100
Wire Wire Line
	1500 2500 1700 2500
Wire Wire Line
	1700 2500 1700 4400
Wire Wire Line
	1700 4400 1500 4400
Wire Wire Line
	1500 2400 1800 2400
Wire Wire Line
	1800 2400 1800 4300
Wire Wire Line
	1800 4300 1500 4300
$Comp
L HR2:CD450B;TTSOP16 U7
U 1 1 5F67B87E
P 1200 7000
F 0 "U7" H 1450 7150 50  0000 C CNN
F 1 "CD450B;TTSOP16" H 1750 6150 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 1400 7050 60  0001 L CNN
F 3 "Hex Level Shifter" H 1400 6850 60  0001 L CNN
F 4 "Hex Level Shifter" H 1400 6750 60  0001 L CNN "Field5"
	1    1200 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 4400 1700 5200
Wire Wire Line
	1700 5200 1000 5200
Connection ~ 1700 4400
Wire Wire Line
	1000 5300 1800 5300
Wire Wire Line
	1800 5300 1800 4300
Connection ~ 1800 4300
Entry Wire Line
	3200 6800 3300 6700
Text Label 3450 7400 0    50   ~ 0
3.3V
Entry Wire Line
	3200 5400 3300 5300
Text Label 2450 5400 0    50   ~ 0
3.3V
Wire Wire Line
	2300 7700 2400 7700
Entry Wire Line
	3200 7900 3300 7800
Text Label 3450 7900 0    50   ~ 0
GND
Text Label 3200 6900 2    50   ~ 0
5V
Wire Wire Line
	2500 6600 2500 6400
Wire Wire Line
	2500 6400 2300 6400
Entry Wire Line
	3200 6600 3300 6500
Text Label 2450 5700 0    50   ~ 0
5V
Wire Wire Line
	1200 7700 1000 7700
Wire Wire Line
	1000 7700 1000 7900
Wire Wire Line
	1000 7900 2400 7900
Wire Wire Line
	2400 7900 2400 7700
Text Label 950  6100 0    50   ~ 0
TRIG4
Text Label 950  6200 0    50   ~ 0
TRIG5
Text Label 950  6300 0    50   ~ 0
TRIG6
Text Label 950  7100 0    50   ~ 0
TRIG7
NoConn ~ 1200 7600
NoConn ~ 2300 7600
Text Label 3200 6600 2    50   ~ 0
GND
Text Label 2800 5800 2    50   ~ 0
TRIG1_5V
Text Label 2800 6000 2    50   ~ 0
TRIG3_5V
Text Label 2800 6100 2    50   ~ 0
TRIG4_5V
Text Label 2800 6200 2    50   ~ 0
TRIG5_5V
Text Label 2800 7100 2    50   ~ 0
TRIG7_5V
Wire Wire Line
	2300 5800 2800 5800
Wire Wire Line
	2800 5900 2300 5900
Wire Wire Line
	2800 6000 2300 6000
Wire Wire Line
	2800 6100 2300 6100
Wire Wire Line
	2800 6200 2300 6200
Entry Wire Line
	2800 6300 2900 6400
Entry Wire Line
	2800 6200 2900 6300
Entry Wire Line
	2800 6100 2900 6200
Entry Wire Line
	2800 6000 2900 6100
Entry Wire Line
	2800 5900 2900 6000
Entry Wire Line
	2800 5800 2900 5900
Entry Wire Line
	2800 7100 2900 7000
Connection ~ 3300 600 
Wire Bus Line
	3300 600  8000 600 
Entry Wire Line
	3200 2100 3300 2000
Entry Wire Line
	3200 4000 3300 3900
Entry Wire Line
	3200 4800 3300 4700
Text Label 3050 4800 0    50   ~ 0
3.3V
Text Label 3000 2900 0    50   ~ 0
3.3V
Text Label 3000 2100 0    50   ~ 0
GND
Wire Wire Line
	2500 6600 3200 6600
Connection ~ 2500 6600
Wire Wire Line
	2400 7900 3200 7900
Connection ~ 2400 7900
$Comp
L Device:R_US R23
U 1 1 5F5BC340
P 6450 1700
F 0 "R23" V 6400 1500 50  0000 L CNN
F 1 "33KΩ;1608" V 6550 1500 50  0000 L CNN
F 2 "" V 6490 1690 50  0001 C CNN
F 3 "~" H 6450 1700 50  0001 C CNN
	1    6450 1700
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R28
U 1 1 5F5BD60D
P 6650 1300
F 0 "R28" V 6600 1050 50  0000 L CNN
F 1 "22KΩ;1608" V 6750 1100 50  0000 L CNN
F 2 "" V 6690 1290 50  0001 C CNN
F 3 "~" H 6650 1300 50  0001 C CNN
	1    6650 1300
	0    1    1    0   
$EndComp
Text Label 7900 1700 2    50   ~ 0
GND
Text Label 6150 1500 0    50   ~ 0
ECHO3
Wire Wire Line
	6100 1300 6500 1300
Wire Wire Line
	6300 1700 6100 1700
Wire Wire Line
	6600 1700 7100 1700
Wire Wire Line
	7100 1700 7100 1400
Wire Wire Line
	6100 1300 6100 1500
Wire Wire Line
	6100 1700 6100 1500
Wire Wire Line
	6100 1500 6700 1500
Wire Wire Line
	7100 900  7900 900 
Wire Wire Line
	7100 1700 7900 1700
Connection ~ 7100 1700
Entry Wire Line
	7900 1700 8000 1600
Entry Wire Line
	7900 900  8000 800 
Wire Wire Line
	6000 1200 7300 1200
Wire Wire Line
	6800 1300 7300 1300
Wire Wire Line
	7300 1400 7100 1400
Wire Wire Line
	7100 1100 7100 900 
$Comp
L HR2:HCSR04;F1X4 CN?
U 1 1 5F49064B
P 8000 1100
AR Path="/5F49064B" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F49064B" Ref="CN50"  Part="1" 
F 0 "CN50" H 8400 1250 50  0000 C CNN
F 1 "HCSR04;F1X4" H 8350 650 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4" H 8200 1150 60  0001 L CNN
F 3 "HC-SR04" H 8200 950 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 8200 1050 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Normal Profile Receptacle)" H 8200 850 60  0001 L CNN "Field5"
	1    8000 1100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7100 1100 7300 1100
Text Label 7900 900  2    50   ~ 0
5V
Entry Wire Line
	6000 1200 5900 1300
Connection ~ 6100 1500
Text Label 6900 2300 0    50   ~ 0
ECHO4_5V
Text Label 6000 2200 0    50   ~ 0
TRIG4_5V
Text HLabel 6700 2500 2    50   Output ~ 0
ECHO4
$Comp
L Device:R_US R24
U 1 1 5F82494B
P 6450 2700
F 0 "R24" V 6400 2500 50  0000 L CNN
F 1 "33KΩ;1608" V 6550 2500 50  0000 L CNN
F 2 "" V 6490 2690 50  0001 C CNN
F 3 "~" H 6450 2700 50  0001 C CNN
	1    6450 2700
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R29
U 1 1 5F824955
P 6650 2300
F 0 "R29" V 6600 2050 50  0000 L CNN
F 1 "22KΩ;1608" V 6750 2100 50  0000 L CNN
F 2 "" V 6690 2290 50  0001 C CNN
F 3 "~" H 6650 2300 50  0001 C CNN
	1    6650 2300
	0    1    1    0   
$EndComp
Text Label 7900 2700 2    50   ~ 0
GND
Text Label 6150 2500 0    50   ~ 0
ECHO4
Wire Wire Line
	6100 2300 6500 2300
Wire Wire Line
	6300 2700 6100 2700
Wire Wire Line
	6600 2700 7100 2700
Wire Wire Line
	7100 2700 7100 2400
Wire Wire Line
	6100 2300 6100 2500
Wire Wire Line
	6100 2700 6100 2500
Wire Wire Line
	6100 2500 6700 2500
Wire Wire Line
	7100 2700 7900 2700
Connection ~ 7100 2700
Entry Wire Line
	7900 2700 8000 2600
Entry Wire Line
	7900 1900 8000 1800
Wire Wire Line
	6000 2200 7300 2200
Wire Wire Line
	6800 2300 7300 2300
Wire Wire Line
	7300 2400 7100 2400
Wire Wire Line
	7100 2100 7100 1900
Wire Wire Line
	7100 2100 7300 2100
Text Label 7900 1900 2    50   ~ 0
5V
Entry Wire Line
	6000 2200 5900 2300
Connection ~ 6100 2500
Text Label 6900 3300 0    50   ~ 0
ECHO5_5V
Text Label 6000 3200 0    50   ~ 0
TRIG5_5V
Text HLabel 6700 3500 2    50   Output ~ 0
ECHO5
$Comp
L Device:R_US R25
U 1 1 5F82B8DE
P 6450 3700
F 0 "R25" V 6400 3450 50  0000 L CNN
F 1 "33KΩ;1608" V 6550 3500 50  0000 L CNN
F 2 "" V 6490 3690 50  0001 C CNN
F 3 "~" H 6450 3700 50  0001 C CNN
	1    6450 3700
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R30
U 1 1 5F82B8E8
P 6650 3300
F 0 "R30" V 6600 3050 50  0000 L CNN
F 1 "22KΩ;1608" V 6750 3100 50  0000 L CNN
F 2 "" V 6690 3290 50  0001 C CNN
F 3 "~" H 6650 3300 50  0001 C CNN
	1    6650 3300
	0    1    1    0   
$EndComp
Text Label 7900 3700 2    50   ~ 0
GND
Text Label 6150 3500 0    50   ~ 0
ECHO5
Wire Wire Line
	6100 3300 6500 3300
Wire Wire Line
	6300 3700 6100 3700
Wire Wire Line
	6600 3700 7100 3700
Wire Wire Line
	7100 3700 7100 3400
Wire Wire Line
	6100 3300 6100 3500
Wire Wire Line
	6100 3700 6100 3500
Wire Wire Line
	6100 3500 6700 3500
Wire Wire Line
	7100 2900 7900 2900
Wire Wire Line
	7100 3700 7900 3700
Connection ~ 7100 3700
Entry Wire Line
	7900 3700 8000 3600
Entry Wire Line
	7900 2900 8000 2800
Wire Wire Line
	6000 3200 7300 3200
Wire Wire Line
	6800 3300 7300 3300
Wire Wire Line
	7300 3400 7100 3400
Wire Wire Line
	7100 3100 7100 2900
Wire Wire Line
	7100 3100 7300 3100
Text Label 7900 2900 2    50   ~ 0
5V
Entry Wire Line
	6000 3200 5900 3300
Connection ~ 6100 3500
Text Label 6900 4300 0    50   ~ 0
ECHO6_5V
Text Label 6000 4200 0    50   ~ 0
TRIG6_5V
Text HLabel 6700 4500 2    50   Output ~ 0
ECHO6
$Comp
L Device:R_US R26
U 1 1 5F82B917
P 6450 4700
F 0 "R26" V 6400 4450 50  0000 L CNN
F 1 "33KΩ;1608" V 6550 4500 50  0000 L CNN
F 2 "" V 6490 4690 50  0001 C CNN
F 3 "~" H 6450 4700 50  0001 C CNN
	1    6450 4700
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R31
U 1 1 5F82B921
P 6650 4300
F 0 "R31" V 6600 4050 50  0000 L CNN
F 1 "22KΩ;1608" V 6750 4100 50  0000 L CNN
F 2 "" V 6690 4290 50  0001 C CNN
F 3 "~" H 6650 4300 50  0001 C CNN
	1    6650 4300
	0    1    1    0   
$EndComp
Text Label 7900 4700 2    50   ~ 0
GND
Text Label 6150 4500 0    50   ~ 0
ECHO6
Wire Wire Line
	6100 4300 6500 4300
Wire Wire Line
	6300 4700 6100 4700
Wire Wire Line
	6600 4700 7100 4700
Wire Wire Line
	7100 4700 7100 4400
Wire Wire Line
	6100 4300 6100 4500
Wire Wire Line
	6100 4700 6100 4500
Wire Wire Line
	6100 4500 6700 4500
Wire Wire Line
	7100 3900 7900 3900
Wire Wire Line
	7100 4700 7900 4700
Connection ~ 7100 4700
Entry Wire Line
	7900 4700 8000 4600
Entry Wire Line
	7900 3900 8000 3800
Wire Wire Line
	6000 4200 7300 4200
Wire Wire Line
	6800 4300 7300 4300
Wire Wire Line
	7300 4400 7100 4400
Wire Wire Line
	7100 4100 7100 3900
Wire Wire Line
	7100 4100 7300 4100
Text Label 7900 3900 2    50   ~ 0
5V
Entry Wire Line
	6000 4200 5900 4300
Connection ~ 6100 4500
Text Label 6900 5400 0    50   ~ 0
ECHO7_5V
Text Label 6000 5300 0    50   ~ 0
TRIG7_5V
Text HLabel 6700 5600 2    50   Output ~ 0
ECHO7
$Comp
L Device:R_US R27
U 1 1 5F83AE46
P 6450 5800
F 0 "R27" V 6400 5550 50  0000 L CNN
F 1 "33KΩ;1608" V 6550 5600 50  0000 L CNN
F 2 "" V 6490 5790 50  0001 C CNN
F 3 "~" H 6450 5800 50  0001 C CNN
	1    6450 5800
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R32
U 1 1 5F83AE50
P 6650 5400
F 0 "R32" V 6600 5150 50  0000 L CNN
F 1 "22KΩ;1608" V 6750 5200 50  0000 L CNN
F 2 "" V 6690 5390 50  0001 C CNN
F 3 "~" H 6650 5400 50  0001 C CNN
	1    6650 5400
	0    1    1    0   
$EndComp
Text Label 7900 5800 2    50   ~ 0
GND
Text Label 6150 5600 0    50   ~ 0
ECHO7
Wire Wire Line
	6100 5400 6500 5400
Wire Wire Line
	6300 5800 6100 5800
Wire Wire Line
	6600 5800 7100 5800
Wire Wire Line
	7100 5800 7100 5500
Wire Wire Line
	6100 5400 6100 5600
Wire Wire Line
	6100 5800 6100 5600
Wire Wire Line
	6100 5600 6700 5600
Wire Wire Line
	7100 5000 7900 5000
Wire Wire Line
	7100 5800 7900 5800
Connection ~ 7100 5800
Entry Wire Line
	7900 5800 8000 5700
Entry Wire Line
	7900 5000 8000 4900
Wire Wire Line
	6000 5300 7300 5300
Wire Wire Line
	6800 5400 7300 5400
Wire Wire Line
	7300 5500 7100 5500
Wire Wire Line
	7100 5200 7100 5000
Wire Wire Line
	7100 5200 7300 5200
Text Label 7900 5000 2    50   ~ 0
5V
Entry Wire Line
	6000 5300 5900 5400
Connection ~ 6100 5600
$Comp
L Device:R_US R34
U 1 1 5F8BD350
P 9150 2300
F 0 "R34" V 9100 2050 50  0000 L CNN
F 1 "22KΩ;1608" V 9250 2050 50  0000 L CNN
F 2 "" V 9190 2290 50  0001 C CNN
F 3 "~" H 9150 2300 50  0001 C CNN
	1    9150 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	8500 2600 9800 2600
Wire Wire Line
	9350 2500 9800 2500
Entry Wire Line
	8000 1700 8100 1800
Wire Wire Line
	8100 1800 8350 1800
Text Label 8100 1800 0    50   ~ 0
GND
$Comp
L HR2:BRIDGE_NE_INNER;3xM1x1+M1x5 CN101
U 1 1 5F49C584
P 9150 1800
F 0 "CN101" H 9650 1950 50  0000 R CNN
F 1 "BRIDGE_NE_INNER;3xM1x1+M1x5" H 9900 1650 50  0000 R CNN
F 2 "HR2:BRIDGE_NE_INNER_3xM1x1+M1x5" H 9350 1850 60  0001 L CNN
F 3 "" H 9350 1650 60  0001 L CNN
F 4 "NE Inner Bridge Connectors" H 9350 1550 60  0001 L CNN "Field5"
	1    9150 1800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9350 2500 9350 2300
Wire Wire Line
	9350 2300 9300 2300
$Comp
L Device:R_US R33
U 1 1 5F8BEAFB
P 8600 2300
F 0 "R33" V 8550 2100 50  0000 L CNN
F 1 "33KΩ;1608" V 8700 2100 50  0000 L CNN
F 2 "" V 8640 2290 50  0001 C CNN
F 3 "~" H 8600 2300 50  0001 C CNN
	1    8600 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	8450 2300 8350 2300
Wire Wire Line
	8350 2300 8350 1800
Connection ~ 8350 1800
Wire Wire Line
	8350 1800 8450 1800
Wire Wire Line
	8750 2300 8850 2300
Wire Wire Line
	8850 2300 8850 2100
Wire Wire Line
	8850 2100 9350 2100
Connection ~ 8850 2300
Wire Wire Line
	8850 2300 9000 2300
Wire Wire Line
	7100 1900 7900 1900
$Comp
L Device:R_US R21
U 1 1 5FADB964
P 4400 2250
F 0 "R21" H 4450 2150 50  0000 L CNN
F 1 "22KΩ;1608" H 3950 2350 50  0000 L CNN
F 2 "" V 4440 2240 50  0001 C CNN
F 3 "~" H 4400 2250 50  0001 C CNN
	1    4400 2250
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R22
U 1 1 5FADC37B
P 4400 3100
F 0 "R22" H 4450 3000 50  0000 L CNN
F 1 "33KΩ;1608" H 3950 3200 50  0000 L CNN
F 2 "" V 4440 3090 50  0001 C CNN
F 3 "~" H 4400 3100 50  0001 C CNN
	1    4400 3100
	-1   0    0    1   
$EndComp
Wire Wire Line
	4400 2000 4400 2100
Wire Wire Line
	4400 2000 4950 2000
Wire Wire Line
	4400 2950 4400 2850
Wire Wire Line
	4950 2850 4400 2850
Connection ~ 4400 2850
Wire Wire Line
	4400 2850 4400 2400
Text Label 4500 2850 0    50   ~ 0
ECHO2
Wire Wire Line
	1500 4700 2200 4700
Entry Wire Line
	2200 4700 2300 4600
Wire Wire Line
	1600 3900 2200 3900
Connection ~ 1600 3900
Wire Wire Line
	2200 3800 1500 3800
Wire Wire Line
	2200 3700 1500 3700
Wire Wire Line
	2200 3600 1500 3600
Wire Wire Line
	2200 3500 1500 3500
Wire Wire Line
	2200 3400 1500 3400
Wire Wire Line
	2200 3300 1500 3300
Wire Wire Line
	2200 1400 1500 1400
Wire Wire Line
	2200 1500 1500 1500
Wire Wire Line
	2200 1600 1500 1600
Wire Wire Line
	2200 1700 1500 1700
Wire Wire Line
	2200 1800 1500 1800
Wire Wire Line
	2200 1900 1500 1900
Wire Wire Line
	1500 2000 2200 2000
Wire Wire Line
	2200 2800 1500 2800
Entry Wire Line
	2200 1400 2300 1300
Entry Wire Line
	2200 1500 2300 1400
Entry Wire Line
	2200 1600 2300 1500
Entry Wire Line
	2200 1700 2300 1600
Entry Wire Line
	2200 1800 2300 1700
Entry Wire Line
	2200 1900 2300 1800
Entry Wire Line
	2200 2000 2300 1900
Entry Wire Line
	2200 2800 2300 2700
Entry Wire Line
	2200 3300 2300 3200
Entry Wire Line
	2200 3400 2300 3300
Entry Wire Line
	2200 3500 2300 3400
Entry Wire Line
	2200 3600 2300 3500
Entry Wire Line
	2200 3700 2300 3600
Entry Wire Line
	2200 3800 2300 3700
Entry Wire Line
	2200 3900 2300 3800
Text Label 2200 1900 2    50   ~ 0
LED11
Text Label 2200 1800 2    50   ~ 0
LED12
Text Label 2200 2800 2    50   ~ 0
LED9
Text Label 2200 3900 2    50   ~ 0
LED8
Text Label 2200 3800 2    50   ~ 0
LED7
Text Label 2200 4700 2    50   ~ 0
LED1
Text Label 2200 3300 2    50   ~ 0
LED2
Text Label 2200 3400 2    50   ~ 0
LED3
Text Label 2200 3500 2    50   ~ 0
LED4
Text Label 2200 3600 2    50   ~ 0
LED5
Text Label 2200 3700 2    50   ~ 0
LED6
Text Label 2200 2000 2    50   ~ 0
LED10
Text Label 2200 1700 2    50   ~ 0
LED13
Text Label 2200 1600 2    50   ~ 0
LED14
Text Label 2200 1500 2    50   ~ 0
LED15
Text Label 2200 1400 2    50   ~ 0
LED16
Entry Wire Line
	8100 5600 8000 5500
Text Label 8100 5600 0    50   ~ 0
9V
Text Label 8100 5500 0    50   ~ 0
GND
Entry Wire Line
	8100 5500 8000 5400
Wire Bus Line
	2300 5000 3800 5000
Wire Bus Line
	3800 6200 9200 6200
Wire Bus Line
	5900 6000 8400 6000
Entry Wire Line
	8500 2600 8400 2700
Wire Bus Line
	4100 3500 5900 3500
Connection ~ 5900 3500
Wire Wire Line
	2300 5700 3200 5700
Entry Wire Line
	3200 5700 3300 5600
Entry Wire Line
	3200 6900 3300 6800
Wire Wire Line
	2300 7000 2400 7000
Entry Wire Line
	3400 4800 3300 4700
Entry Wire Line
	3400 4900 3300 4800
Text Label 3400 4800 0    50   ~ 0
GND
Text Label 3400 4900 0    50   ~ 0
9V
Entry Wire Line
	3900 5000 3800 5100
Entry Wire Line
	3900 5100 3800 5200
Entry Wire Line
	3900 5200 3800 5300
Entry Wire Line
	3900 5300 3800 5400
Entry Wire Line
	3900 5400 3800 5500
Entry Wire Line
	4200 1800 4100 1900
Wire Bus Line
	4500 900  2300 900 
Wire Wire Line
	4600 1600 4950 1600
Wire Wire Line
	4600 1200 4950 1200
Wire Wire Line
	4600 1100 4950 1100
Entry Wire Line
	4600 1100 4500 1000
Entry Wire Line
	4600 1200 4500 1100
Entry Wire Line
	4600 1600 4500 1500
Wire Wire Line
	4300 1900 4300 2000
Wire Wire Line
	4300 2000 3400 2000
Wire Wire Line
	4300 1900 4950 1900
Entry Wire Line
	3400 2000 3300 1900
Text Label 3400 2000 0    50   ~ 0
5V
Entry Wire Line
	3400 1700 3300 1600
Text Label 3400 1700 0    50   ~ 0
GND
Entry Wire Line
	3400 3350 3300 3250
Text Label 3400 3350 0    50   ~ 0
GND
Wire Wire Line
	3400 3350 4400 3350
Wire Wire Line
	4400 3350 4400 3250
Wire Wire Line
	9300 2700 9800 2700
Wire Wire Line
	9300 2800 9800 2800
Wire Wire Line
	9300 2900 9800 2900
Entry Wire Line
	9300 2700 9200 2800
Entry Wire Line
	9300 2800 9200 2900
Entry Wire Line
	9300 2900 9200 3000
Text Notes 7050 6150 0    50   ~ 0
LED Bus
Text Notes 7000 5950 0    50   ~ 0
Sonar Bus
Text Notes 4850 3450 0    50   ~ 0
Sonar Bus
Text Notes 4800 550  0    50   ~ 0
Power Bus
Text Notes 2800 850  0    50   ~ 0
LED Bus
Text Notes 4350 7250 0    50   ~ 0
Sonar Bus
Text Notes 8450 2950 0    50   ~ 0
Sonar\nBus
Text Label 3900 5200 0    50   ~ 0
LED9
Text Label 3900 5100 0    50   ~ 0
LED8
Text Label 3900 5000 0    50   ~ 0
LED7
Entry Wire Line
	3200 2900 3300 2800
Wire Wire Line
	1500 4500 3200 4500
Entry Wire Line
	3200 4500 3300 4400
Text Label 3000 4500 0    50   ~ 0
GND
Text Label 3000 4000 0    50   ~ 0
GND
Wire Wire Line
	1500 4200 3200 4200
Entry Wire Line
	3200 4200 3300 4100
Text Label 3000 4200 0    50   ~ 0
3.3V
Wire Wire Line
	1500 2600 3200 2600
Wire Wire Line
	1500 2300 3200 2300
Entry Wire Line
	3200 2300 3300 2200
Entry Wire Line
	3200 2600 3300 2500
Text Label 3000 2600 0    50   ~ 0
GND
Text Label 3000 2300 0    50   ~ 0
3.3V
Text Notes 2700 5100 0    50   ~ 0
LED Bus
Wire Wire Line
	1500 2900 3200 2900
Text Notes 1400 6750 0    50   ~ 0
3.3V to 5V Level Shifters
Text Notes 1050 1250 0    50   ~ 0
16-Bit SPI LED Shift Register
Text Notes 4750 900  0    50   ~ 0
NW Quadrant Connector
Text Notes 9450 700  0    50   ~ 0
NE Quadrant Connector
Text Notes 3700 4700 0    50   ~ 0
SW Quadrant Connector
Text Notes 9300 5300 0    50   ~ 0
SE Quadrant Connector
Wire Wire Line
	1500 4800 3200 4800
Wire Wire Line
	1500 2100 3200 2100
Wire Wire Line
	1500 4000 3200 4000
Text Notes 3350 4450 0    50   ~ 0
Notes:\n1. The LED's are on seperate PCB's that fit the Bantam Tools\n   PCB blank restriction of 4in x 5in.  There are 4 of them,\n   and they are imaginatively called NE, NW, SE, and SW and\n   they are connected to by CN101, CN102, CN103, and CN104.\n2. The LED's are controlled by a 16-bit shift regester that is\n   filled using an SPI controller.  The 3.3V outputs are fed into\n   FET's that drive the LED's.\n3. The Sonars are ar HC04's that operat off of 5V.  There are\n   3.3V level shifter to shift the trigger signals to 5V an resistor\n   voltage dividers to convert the 5V echo signals down to 3.3V.\n4. All footprints are metric (i.e. 1608 instead of 0603.)
Wire Notes Line
	5750 800  5750 3300
Wire Notes Line
	5750 3300 3650 3300
Wire Notes Line
	3650 3300 3650 800 
Wire Notes Line
	3650 800  5750 800 
Wire Notes Line
	10450 3150 10450 600 
Wire Notes Line
	10450 600  8100 600 
Wire Notes Line
	8100 600  8100 3150
Wire Notes Line
	8100 3150 10450 3150
Wire Notes Line
	10450 6450 9250 6450
Wire Notes Line
	9250 5200 10450 5200
Wire Notes Line
	5200 4700 5200 5700
Wire Notes Line
	5200 5700 3700 5700
Wire Notes Line
	3700 5700 3700 4700
Wire Notes Line
	3700 4600 5200 4600
$Comp
L Device:C C10
U 1 1 5F7FE24F
P 3800 7650
F 0 "C10" H 3600 7750 50  0000 L CNN
F 1 "10pF;1608" H 3850 7550 50  0000 L CNN
F 2 "" H 3838 7500 50  0001 C CNN
F 3 "~" H 3800 7650 50  0001 C CNN
	1    3800 7650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 7500 3800 7400
Wire Wire Line
	3800 7400 3400 7400
Wire Wire Line
	3800 7800 3800 7900
Wire Wire Line
	3800 7900 3400 7900
Entry Wire Line
	3400 7400 3300 7500
Entry Wire Line
	3400 7900 3300 7800
$Comp
L Device:C C11
U 1 1 5F8B3EAF
P 4400 7650
F 0 "C11" H 4200 7750 50  0000 L CNN
F 1 "10pF;1608" H 4450 7550 50  0000 L CNN
F 2 "" H 4438 7500 50  0001 C CNN
F 3 "~" H 4400 7650 50  0001 C CNN
	1    4400 7650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5F8B4468
P 5000 7650
F 0 "C12" H 4800 7750 50  0000 L CNN
F 1 "10pF;1608" H 5050 7550 50  0000 L CNN
F 2 "" H 5038 7500 50  0001 C CNN
F 3 "~" H 5000 7650 50  0001 C CNN
	1    5000 7650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 7400 4400 7400
Connection ~ 3800 7400
Wire Wire Line
	4400 7500 4400 7400
Connection ~ 4400 7400
Wire Wire Line
	4400 7400 5000 7400
Wire Wire Line
	5000 7500 5000 7400
Connection ~ 5000 7400
Wire Wire Line
	5000 7400 5600 7400
Wire Wire Line
	3800 7900 4400 7900
Connection ~ 3800 7900
Wire Wire Line
	5000 7800 5000 7900
Connection ~ 5000 7900
Wire Wire Line
	5000 7900 5600 7900
Wire Wire Line
	4400 7800 4400 7900
Connection ~ 4400 7900
Wire Wire Line
	4400 7900 5000 7900
Wire Wire Line
	5600 7900 5600 7800
Wire Wire Line
	5600 7400 5600 7500
$Comp
L Device:C C13
U 1 1 5F8E5BE2
P 5600 7650
F 0 "C13" H 5400 7750 50  0000 L CNN
F 1 "10pF;1608" H 5650 7550 50  0000 L CNN
F 2 "" H 5638 7500 50  0001 C CNN
F 3 "~" H 5600 7650 50  0001 C CNN
	1    5600 7650
	1    0    0    -1  
$EndComp
Text Notes 4200 8000 0    50   ~ 0
Filter Capacitors
Text Notes 9150 3250 2    50   ~ 0
LED Bus
Text Notes 3550 9000 0    50   ~ 0
TO DO:\n1. Add 2 buttons to both the SE and SW connectors.
Text Label 3200 6800 2    50   ~ 0
3.3V
Text Label 3050 7900 0    50   ~ 0
GND
$Comp
L HR2:SERVO;M1x3 CN6
U 1 1 5F5AF403
P 10500 3400
F 0 "CN6" H 10950 3550 50  0000 C CNN
F 1 "SERVO;M1x3" H 10850 3050 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10700 3450 60  0001 L CNN
F 3 "Servo M1X3" H 10700 3250 60  0001 L CNN
F 4 "Servo M1X3" H 10700 3350 60  0001 L CNN "manf#"
	1    10500 3400
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;M1x3 CN4
U 1 1 5F5E6C81
P 10050 3900
F 0 "CN4" H 10500 4050 50  0000 C CNN
F 1 "SERVO;M1x3" H 10450 3550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10250 3950 60  0001 L CNN
F 3 "Servo M1X3" H 10250 3750 60  0001 L CNN
F 4 "Servo M1X3" H 10250 3850 60  0001 L CNN "manf#"
	1    10050 3900
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;M1x3 CN7
U 1 1 5F5E7210
P 10500 4400
F 0 "CN7" H 10950 4550 50  0000 C CNN
F 1 "SERVO;M1x3" H 10850 4050 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10700 4450 60  0001 L CNN
F 3 "Servo M1X3" H 10700 4250 60  0001 L CNN
F 4 "Servo M1X3" H 10700 4350 60  0001 L CNN "manf#"
	1    10500 4400
	-1   0    0    -1  
$EndComp
$Comp
L HR2:SERVO;M1x3 CN5
U 1 1 5F5E7655
P 10050 4800
F 0 "CN5" H 10500 4950 50  0000 C CNN
F 1 "SERVO;M1x3" H 10450 4450 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10250 4850 60  0001 L CNN
F 3 "Servo M1X3" H 10250 4650 60  0001 L CNN
F 4 "Servo M1X3" H 10250 4750 60  0001 L CNN "manf#"
	1    10050 4800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9800 3500 8100 3500
Wire Wire Line
	9800 3400 8100 3400
Wire Wire Line
	9350 3900 8100 3900
Wire Wire Line
	8100 4000 9350 4000
Wire Wire Line
	9800 4400 8100 4400
Wire Wire Line
	8100 4500 9800 4500
Wire Wire Line
	9350 4800 8100 4800
Wire Wire Line
	8100 4900 9350 4900
Entry Wire Line
	8100 3400 8000 3300
Entry Wire Line
	8100 3500 8000 3400
Entry Wire Line
	8100 3900 8000 3800
Entry Wire Line
	8100 4000 8000 3900
Entry Wire Line
	8100 4400 8000 4300
Entry Wire Line
	8100 4500 8000 4400
Entry Wire Line
	8100 4800 8000 4700
Entry Wire Line
	8100 4900 8000 4800
Text Label 8100 3500 0    50   ~ 0
5V
Text Label 8100 4000 0    50   ~ 0
5V
Text Label 8100 4500 0    50   ~ 0
5V
Text Label 8100 4900 0    50   ~ 0
5V
Entry Wire Line
	9200 5800 9300 5700
Text Label 2800 5900 2    50   ~ 0
TRIG2_5V
Wire Wire Line
	3200 6900 2400 6900
Wire Wire Line
	2400 6900 2400 7000
Wire Wire Line
	2800 6300 2300 6300
Text Label 2800 6300 2    50   ~ 0
TRIG6_5V
Text Label 950  6000 0    50   ~ 0
TRIG3
Text Label 950  5900 0    50   ~ 0
TRIG2
Text Label 950  5800 0    50   ~ 0
TRIG1
Wire Wire Line
	1050 5400 3200 5400
Wire Wire Line
	1200 5700 1050 5700
Wire Wire Line
	1050 5700 1050 5400
Wire Wire Line
	1050 6600 2500 6600
Wire Wire Line
	1200 6400 1050 6400
Wire Wire Line
	1050 6400 1050 6600
Wire Wire Line
	1000 6800 3200 6800
Wire Wire Line
	1200 7000 1000 7000
Wire Wire Line
	1000 7000 1000 6800
Wire Wire Line
	900  7100 1200 7100
Text HLabel 900  7100 0    50   Input ~ 0
TRIG7
Wire Wire Line
	900  6300 1200 6300
Text HLabel 900  6300 0    50   Input ~ 0
TRIG6
Wire Wire Line
	900  6200 1200 6200
Text HLabel 900  6200 0    50   Input ~ 0
TRIG5
Wire Wire Line
	900  6100 1200 6100
Text HLabel 900  6100 0    50   Input ~ 0
TRIG4
Wire Wire Line
	900  6000 1200 6000
Text HLabel 900  6000 0    50   Input ~ 0
TRIG3
Wire Wire Line
	900  5900 1200 5900
Text HLabel 900  5900 0    50   Input ~ 0
TRIG2
Wire Wire Line
	900  5800 1200 5800
Text HLabel 900  5800 0    50   Input ~ 0
TRIG1
Wire Wire Line
	2300 7100 2800 7100
Text HLabel 900  7200 0    50   Input ~ 0
SERVO1
Text HLabel 900  7300 0    50   Input ~ 0
SERVO2
Text HLabel 900  7400 0    50   Input ~ 0
SERVO3
Text HLabel 900  7500 0    50   Input ~ 0
SERVO4
Wire Wire Line
	900  7200 1200 7200
Wire Wire Line
	1200 7300 900  7300
Wire Wire Line
	900  7400 1200 7400
Wire Wire Line
	1200 7500 900  7500
Text Label 950  7200 0    50   ~ 0
SERVO1
Text Label 950  7300 0    50   ~ 0
SERVO2
Text Label 950  7400 0    50   ~ 0
SERVO3
Text Label 950  7500 0    50   ~ 0
SERVO4
Text Label 3000 7200 2    50   ~ 0
SERVO1_5V
Wire Wire Line
	3000 7200 2300 7200
Text Label 3000 7300 2    50   ~ 0
SERVO2_5V
Text Label 3000 7400 2    50   ~ 0
SERVO3_5V
Text Label 3000 7500 2    50   ~ 0
SERVO4_5V
Wire Wire Line
	3000 7300 2300 7300
Wire Wire Line
	3000 7400 2300 7400
Wire Wire Line
	2300 7500 3000 7500
Entry Wire Line
	3000 7200 3100 7100
Entry Wire Line
	3000 7300 3100 7200
Entry Wire Line
	3000 7400 3100 7300
Entry Wire Line
	3000 7500 3100 7400
Wire Bus Line
	3100 7000 3800 7000
Wire Bus Line
	3800 7000 3800 6400
Wire Bus Line
	3800 6400 8600 6400
Wire Wire Line
	8700 3600 9800 3600
Wire Wire Line
	9800 4600 8700 4600
Wire Wire Line
	8700 5000 9350 5000
Entry Wire Line
	8700 5000 8600 5100
Entry Wire Line
	8700 4600 8600 4700
Entry Wire Line
	8700 4100 8600 4200
Entry Wire Line
	8700 3600 8600 3700
Wire Wire Line
	8700 4100 9350 4100
Text Label 8100 4800 0    50   ~ 0
GND
Text Label 8100 4400 0    50   ~ 0
GND
Text Label 8100 3900 0    50   ~ 0
GND
Text Label 8700 3600 0    50   ~ 0
SERVO1_5V
Text Label 8700 4100 0    50   ~ 0
SERVO2_5V
Text Label 8700 4600 0    50   ~ 0
SERVO3_5V
Text Label 8700 5000 0    50   ~ 0
SERVO4_5V
Wire Wire Line
	4200 5000 3900 5000
Wire Wire Line
	3900 5100 4200 5100
Wire Wire Line
	4200 5200 3900 5200
Wire Wire Line
	3900 5300 4200 5300
Wire Wire Line
	4200 5400 3900 5400
Connection ~ 5900 5800
Wire Bus Line
	5900 5800 5900 6000
Wire Bus Line
	2900 5800 5900 5800
Text Notes 7050 6350 0    50   ~ 0
Servo Bus
Text Notes 5350 5750 0    50   ~ 0
Sonar Bus
Text Notes 4100 6150 0    50   ~ 0
LED Bus
Text Notes 4050 6350 0    50   ~ 0
Servo Bus
Wire Notes Line
	9250 3200 10450 3200
Wire Notes Line
	9250 3200 9250 6450
Wire Notes Line
	10450 3200 10450 6450
Text Notes 9950 5000 0    50   ~ 0
Servo\nConnectors
Wire Bus Line
	4100 1800 4100 3500
Wire Bus Line
	8400 2600 8400 6000
Wire Bus Line
	5900 3500 5900 5800
Wire Bus Line
	4500 900  4500 1600
Wire Bus Line
	5900 1150 5900 3500
Wire Bus Line
	1600 600  1600 1100
Wire Bus Line
	3100 7000 3100 7500
Wire Bus Line
	2900 5800 2900 7100
Wire Bus Line
	3800 5000 3800 6200
Wire Bus Line
	9200 2700 9200 6200
Wire Bus Line
	8600 3600 8600 6400
Wire Bus Line
	2300 900  2300 5000
Wire Bus Line
	3300 600  3300 7900
Wire Bus Line
	8000 600  8000 5800
Text Label 8100 3400 0    50   ~ 0
GND
$EndSCHEMATC
