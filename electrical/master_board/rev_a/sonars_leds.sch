EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 4 8
Title "HR2 Sonars and LED's"
Date "2020-08-27"
Rev "A"
Comp "Home Brew Robotics Club"
Comment1 "Copyright © 2020 by Home Brew Robotics Club"
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F490633
P 5600 3500
AR Path="/5F490633" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490633" Ref="CN51"  Part="1" 
F 0 "CN51" H 6000 3650 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 5950 3050 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 5800 3550 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 3350 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 3450 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 5800 3250 60  0001 L CNN "Field5"
	1    5600 3500
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F49063B
P 5600 4200
AR Path="/5F49063B" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F49063B" Ref="CN52"  Part="1" 
F 0 "CN52" H 6000 4350 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 5950 3750 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 5800 4250 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 4050 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 4150 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 5800 3950 60  0001 L CNN "Field5"
	1    5600 4200
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F490643
P 5600 4950
AR Path="/5F490643" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490643" Ref="CN53"  Part="1" 
F 0 "CN53" H 6000 5100 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 5950 4500 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 5800 5000 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 4800 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 4900 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 5800 4700 60  0001 L CNN "Field5"
	1    5600 4950
	-1   0    0    -1  
$EndComp
$Comp
L HR2:HCSR04;F1X4 CN?
U 1 1 5F49064B
P 5600 2800
AR Path="/5F49064B" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F49064B" Ref="CN50"  Part="1" 
F 0 "CN50" H 6000 2950 50  0000 C CNN
F 1 "HCSR04;F1X4" H 5950 2350 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4" H 5800 2850 60  0001 L CNN
F 3 "HC-SR04" H 5800 2650 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 2750 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Normal Profile Receptacle)" H 5800 2550 60  0001 L CNN "Field5"
	1    5600 2800
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NW_INNER;M1x2+M2x5 CN?
U 1 1 5F490676
P 3250 1500
AR Path="/5F490676" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490676" Ref="CN102"  Part="1" 
F 0 "CN102" H 3600 1650 50  0000 C CNN
F 1 "BRIDGE_NW_INNER;M1x2+M2x5" H 3450 1250 50  0000 C CNN
F 2 "HR2:BRIDGE_NW_INNER_M1x2+M2x5" H 3450 1550 60  0001 L CNN
F 3 "" H 3450 1350 60  0001 L CNN
F 4 "NW Inner Bridge Connectors" H 3450 1250 60  0001 L CNN "Field5"
	1    3250 1500
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NW_INNER;M1x2+M2x5 CN?
U 2 1 5F49067D
P 3250 2000
AR Path="/5F49067D" Ref="CN?"  Part="2" 
AR Path="/5F48CAAD/5F49067D" Ref="CN102"  Part="2" 
F 0 "CN102" H 3600 2150 50  0000 C CNN
F 1 "BRIDGE_NW_INNER;M1x2+M2x5" H 3450 950 50  0000 C CNN
F 2 "HR2:BRIDGE_NW_INNER_M1x2+M2x5" H 3450 2050 60  0001 L CNN
F 3 "" H 3450 1850 60  0001 L CNN
F 4 "NW Inner Bridge Connectors" H 3450 1750 60  0001 L CNN "Field5"
	2    3250 2000
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_SE_INNER;M1x8 CN?
U 1 1 5F490690
P 8500 5500
AR Path="/5F490690" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F490690" Ref="CN103"  Part="1" 
F 0 "CN103" H 8800 5650 50  0000 C CNN
F 1 "BRIDGE_SE_INNER;M1x8" H 8850 4650 50  0000 C CNN
F 2 "HR2:BRIDGE_SE_INNER_M1x8" H 8700 5550 60  0001 L CNN
F 3 "" H 8700 5350 60  0001 L CNN
F 4 "SE Inner Bridge Connector" H 8700 5250 60  0001 L CNN "Field5"
	1    8500 5500
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_SW_INNER;M1x8 CN?
U 1 1 5F49069F
P 3300 5500
AR Path="/5F49069F" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F49069F" Ref="CN104"  Part="1" 
F 0 "CN104" H 3600 5650 50  0000 C CNN
F 1 "BRIDGE_SW_INNER;M1x8" H 3600 4650 50  0000 C CNN
F 2 "HR2:BRIDGE_SW_INNER_M1x8" H 3500 5550 60  0001 L CNN
F 3 "" H 3500 5350 60  0001 L CNN
F 4 "SW Inner Bridge Connector" H 3500 5250 60  0001 L CNN "Field5"
	1    3300 5500
	-1   0    0    -1  
$EndComp
Text HLabel 7050 2100 0    50   Input ~ 0
LED1
Text HLabel 7050 2200 0    50   Input ~ 0
LED2
Text HLabel 7050 2300 0    50   Input ~ 0
LED3
Text HLabel 1850 1500 0    50   Input ~ 0
LED4
Text HLabel 1850 1600 0    50   Input ~ 0
LED5
Text HLabel 1850 2000 0    50   Input ~ 0
LED6
Text HLabel 1800 5700 0    50   Input ~ 0
LED7
Text HLabel 1800 5800 0    50   Input ~ 0
LED8
Text HLabel 1800 5900 0    50   Input ~ 0
LED9
Text HLabel 1800 6000 0    50   Input ~ 0
LED10
Text HLabel 1800 6100 0    50   Input ~ 0
LED11
Text HLabel 7100 5700 0    50   Input ~ 0
LED12
Text HLabel 7100 5800 0    50   Input ~ 0
LED13
Text HLabel 7100 5900 0    50   Input ~ 0
LED14
Text HLabel 7100 6000 0    50   Input ~ 0
LED15
Text HLabel 7100 6100 0    50   Input ~ 0
LED16
Text HLabel 1850 2200 0    50   Input ~ 0
TRIG2
Text HLabel 1850 2400 0    50   Output ~ 0
ECHO2
Text HLabel 4300 2900 0    50   Input ~ 0
TRIG3
Text HLabel 4300 3000 0    50   Output ~ 0
ECHO3
Text HLabel 4300 3600 0    50   Input ~ 0
TRIG4
Text HLabel 4300 3700 0    50   Output ~ 0
ECHO4
Text HLabel 4300 4300 0    50   Input ~ 0
TRIG5
Text HLabel 4300 4400 0    50   Output ~ 0
ECHO5
Text HLabel 4300 5050 0    50   Input ~ 0
TRIG6
Text HLabel 4300 5150 0    50   Output ~ 0
ECHO6
Text HLabel 4300 5750 0    50   Input ~ 0
TRIG7
Text HLabel 4300 5850 0    50   Output ~ 0
ECHO7
Text HLabel 1850 900  0    50   Input ~ 0
9V
Text HLabel 1850 1000 0    50   Input ~ 0
5V
Text HLabel 1800 6500 0    50   Input ~ 0
GND
$Comp
L HR2:HCSR04LP;F1X4 CN?
U 1 1 5F4A9A9E
P 5600 5650
AR Path="/5F4A9A9E" Ref="CN?"  Part="1" 
AR Path="/5F48CAAD/5F4A9A9E" Ref="CN54"  Part="1" 
F 0 "CN54" H 6000 5800 50  0000 C CNN
F 1 "HCSR04LP;F1X4" H 5950 5200 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4LP" H 5800 5700 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 5500 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5800 5600 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (Low Profile Receptacle)" H 5800 5400 60  0001 L CNN "Field5"
	1    5600 5650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4900 3100 4800 3100
Wire Wire Line
	4800 3100 4800 3800
Wire Wire Line
	4800 3800 4900 3800
Wire Wire Line
	4800 3800 4800 4500
Wire Wire Line
	4800 4500 4900 4500
Connection ~ 4800 3800
Wire Wire Line
	4800 4500 4800 5250
Wire Wire Line
	4800 5250 4900 5250
Connection ~ 4800 4500
Wire Wire Line
	4800 5250 4800 5950
Wire Wire Line
	4800 5950 4900 5950
Connection ~ 4800 5250
Wire Wire Line
	4900 5650 4700 5650
Wire Wire Line
	4700 4200 4900 4200
Wire Wire Line
	4700 4200 4700 3500
Wire Wire Line
	4700 3500 4900 3500
Connection ~ 4700 4200
Wire Wire Line
	4700 3500 4700 2800
Wire Wire Line
	4700 2800 4900 2800
Connection ~ 4700 3500
Wire Wire Line
	4300 2900 4900 2900
Wire Wire Line
	4300 3000 4900 3000
Wire Wire Line
	4300 3600 4900 3600
Wire Wire Line
	4300 3700 4900 3700
Wire Wire Line
	4300 4300 4900 4300
Wire Wire Line
	4300 4400 4900 4400
Wire Wire Line
	4300 5050 4900 5050
Wire Wire Line
	4900 5150 4300 5150
Wire Wire Line
	4300 5750 4900 5750
Wire Wire Line
	4900 5850 4300 5850
Wire Wire Line
	4800 5950 4800 6500
Wire Wire Line
	4800 6500 2400 6500
Connection ~ 4800 5950
Text Label 4350 2900 0    50   ~ 0
TRIG3
Text Label 4350 3000 0    50   ~ 0
ECHO3
Text Label 4350 3600 0    50   ~ 0
TRIG4
Text Label 4350 3700 0    50   ~ 0
ECHO4
Text Label 4350 4300 0    50   ~ 0
TRIG5
Text Label 4350 4400 0    50   ~ 0
ECHO5
Text Label 4350 5050 0    50   ~ 0
TRIG6
Text Label 4350 5750 0    50   ~ 0
TRIG7
Text Label 4350 5150 0    50   ~ 0
ECHO6
Text Label 4350 5850 0    50   ~ 0
ECHO7
Text HLabel 7050 1800 0    50   Input ~ 0
TRIG1
Text HLabel 7050 1900 0    50   Output ~ 0
ECHO1
Text Label 7050 1800 0    50   ~ 0
TRIG1
Text Label 7050 1900 0    50   ~ 0
ECHO1
Wire Wire Line
	7050 1900 7800 1900
Wire Wire Line
	7050 1800 7350 1800
Wire Wire Line
	7350 1800 7350 2000
Wire Wire Line
	7350 2000 7800 2000
Wire Wire Line
	7050 2100 7800 2100
Wire Wire Line
	7800 2200 7050 2200
Wire Wire Line
	7050 2300 7800 2300
Text Label 7050 2100 0    50   ~ 0
LED1
Text Label 7050 2200 0    50   ~ 0
LED2
Text Label 7050 2300 0    50   ~ 0
LED3
Wire Wire Line
	7100 5700 7700 5700
Wire Wire Line
	7100 5800 7700 5800
Wire Wire Line
	7100 5900 7700 5900
Wire Wire Line
	7100 6000 7700 6000
Wire Wire Line
	7100 6100 7700 6100
Text Label 7150 5700 0    50   ~ 0
LED12
Text Label 7150 5800 0    50   ~ 0
LED13
Text Label 7150 5900 0    50   ~ 0
LED14
Text Label 7150 6000 0    50   ~ 0
LED15
Text Label 7150 6100 0    50   ~ 0
LED16
Wire Wire Line
	1800 5700 2500 5700
Wire Wire Line
	1800 5800 2500 5800
Wire Wire Line
	1800 5900 2500 5900
Wire Wire Line
	1800 6000 2500 6000
Wire Wire Line
	1800 6100 2500 6100
Text Label 1850 5700 0    50   ~ 0
LED7
Text Label 1850 5800 0    50   ~ 0
LED8
Text Label 1850 5900 0    50   ~ 0
LED9
Text Label 1850 6000 0    50   ~ 0
LED10
Text Label 1850 6100 0    50   ~ 0
LED11
NoConn ~ 2500 6200
Wire Wire Line
	1850 2400 2550 2400
Wire Wire Line
	1850 2200 2550 2200
Wire Wire Line
	1850 2000 2550 2000
Wire Wire Line
	1850 1600 2550 1600
Wire Wire Line
	1850 1500 2550 1500
Text Label 1900 1500 0    50   ~ 0
LED4
Text Label 1900 1600 0    50   ~ 0
LED5
Text Label 1900 2000 0    50   ~ 0
LED6
Text Label 1900 2200 0    50   ~ 0
TRIG2
Text Label 1900 2400 0    50   ~ 0
ECHO2
NoConn ~ 2550 2500
NoConn ~ 2550 2600
NoConn ~ 2550 2700
NoConn ~ 2550 2800
NoConn ~ 2550 2900
Wire Wire Line
	7750 2800 7600 2800
Wire Wire Line
	7600 2800 7600 5500
Wire Wire Line
	7700 5500 7600 5500
NoConn ~ 7700 6200
Wire Wire Line
	4800 6500 7600 6500
Wire Wire Line
	7600 5500 7600 6500
Wire Wire Line
	2500 5500 2400 5500
Wire Wire Line
	2400 5500 2400 6500
Connection ~ 2400 6500
Wire Wire Line
	2400 6500 1800 6500
Wire Wire Line
	2400 5500 2400 2100
Wire Wire Line
	2400 2100 2550 2100
Connection ~ 2400 5500
Connection ~ 4800 6500
Wire Wire Line
	2550 2300 2300 2300
Wire Wire Line
	2300 2300 2300 1000
Wire Wire Line
	2300 1000 1850 1000
Wire Wire Line
	2300 1000 4700 1000
Wire Wire Line
	7500 1000 7500 1400
Wire Wire Line
	7500 1400 7800 1400
Connection ~ 2300 1000
Wire Wire Line
	7700 5600 7500 5600
Wire Wire Line
	7500 5600 7500 1400
Wire Wire Line
	2500 5600 2200 5600
Wire Wire Line
	2200 5600 2200 900 
Connection ~ 2200 900 
Wire Wire Line
	2200 900  1850 900 
Wire Wire Line
	4700 4200 4700 4950
Wire Wire Line
	4900 4950 4700 4950
Connection ~ 4700 4950
Wire Wire Line
	4700 4950 4700 5650
Wire Wire Line
	4700 2800 4700 1000
Connection ~ 4700 2800
Connection ~ 4700 1000
Wire Wire Line
	4700 1000 7500 1000
$Comp
L HR2:BRIDGE_NE_INNER;3xM1x1+M1x5 CN101
U 4 1 5F48CE60
P 8400 900
F 0 "CN101" H 8800 1050 50  0000 R CNN
F 1 "BRIDGE_NE_INNER;3xM1x1+M1x5" H 9100 750 50  0000 R CNN
F 2 "HR2:BRIDGE_NE_INNER_3xM1x1+M1x5" H 8600 950 60  0001 L CNN
F 3 "" H 8600 750 60  0001 L CNN
F 4 "NE Inner Bridge Connectors" H 8600 650 60  0001 L CNN "Field5"
	4    8400 900 
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NE_INNER;3xM1x1+M1x5 CN101
U 2 1 5F49BA1D
P 8400 1400
F 0 "CN101" H 8800 1550 50  0000 R CNN
F 1 "BRIDGE_NE_INNER;3xM1x1+M1x5" H 9150 1250 50  0000 R CNN
F 2 "HR2:BRIDGE_NE_INNER_3xM1x1+M1x5" H 8600 1450 60  0001 L CNN
F 3 "" H 8600 1250 60  0001 L CNN
F 4 "NE Inner Bridge Connectors" H 8600 1150 60  0001 L CNN "Field5"
	2    8400 1400
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NE_INNER;3xM1x1+M1x5 CN101
U 3 1 5F49BE0B
P 8500 1900
F 0 "CN101" H 8900 2050 50  0000 R CNN
F 1 "BRIDGE_NE_INNER;3xM1x1+M1x5" H 9250 1350 50  0000 R CNN
F 2 "HR2:BRIDGE_NE_INNER_3xM1x1+M1x5" H 8700 1950 60  0001 L CNN
F 3 "" H 8700 1750 60  0001 L CNN
F 4 "NE Inner Bridge Connectors" H 8700 1650 60  0001 L CNN "Field5"
	3    8500 1900
	-1   0    0    -1  
$EndComp
$Comp
L HR2:BRIDGE_NE_INNER;3xM1x1+M1x5 CN101
U 1 1 5F49C584
P 8450 2800
F 0 "CN101" H 8950 2950 50  0000 R CNN
F 1 "BRIDGE_NE_INNER;3xM1x1+M1x5" H 9200 2650 50  0000 R CNN
F 2 "HR2:BRIDGE_NE_INNER_3xM1x1+M1x5" H 8650 2850 60  0001 L CNN
F 3 "" H 8650 2650 60  0001 L CNN
F 4 "NE Inner Bridge Connectors" H 8650 2550 60  0001 L CNN "Field5"
	1    8450 2800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2200 900  7800 900 
Connection ~ 7500 1400
Connection ~ 7600 5500
$EndSCHEMATC
