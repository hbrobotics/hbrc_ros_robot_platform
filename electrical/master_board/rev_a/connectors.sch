EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 3 9
Title "HR2 Connectors"
Date "2020-10-03"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
NoConn ~ 4600 3400
NoConn ~ 4600 3500
NoConn ~ 4600 3600
NoConn ~ 4600 3700
NoConn ~ 4600 3800
NoConn ~ 4600 3900
NoConn ~ 6450 3400
NoConn ~ 6450 3500
NoConn ~ 6450 3600
NoConn ~ 6450 3700
NoConn ~ 6450 3800
NoConn ~ 6450 3900
NoConn ~ 6450 4200
NoConn ~ 6450 4300
NoConn ~ 6450 4400
NoConn ~ 6450 4500
NoConn ~ 6450 4600
NoConn ~ 6450 4700
NoConn ~ 4600 4200
NoConn ~ 4600 4300
NoConn ~ 4600 4400
NoConn ~ 4600 4500
NoConn ~ 4600 4600
NoConn ~ 4600 4700
$Comp
L HR2:MIKROBUS_MATE;SMALL CN?
U 1 1 5F4CFB97
P 5400 3200
AR Path="/5F4CFB97" Ref="CN?"  Part="1" 
AR Path="/5F4CEE66/5F4CFB97" Ref="CN40"  Part="1" 
F 0 "CN40" H 5700 3350 50  0000 C CNN
F 1 "MIKROBUS_MATE;SMALL" H 5800 1550 50  0000 C CNN
F 2 "HR2:MIKROBUS_SMALL_MATE" H 5600 3250 60  0001 L CNN
F 3 "MikroBus Small Mate" H 5600 3050 60  0001 L CNN
F 4 "MikroBus Small Mate" H 5600 2950 60  0001 L CNN "Field5"
	1    5400 3200
	-1   0    0    -1  
$EndComp
$Comp
L HR2:MIKROBUS_MATE;MEDIUM CN?
U 1 1 5F4CFB9E
P 7250 3200
AR Path="/5F4CFB9E" Ref="CN?"  Part="1" 
AR Path="/5F4CEE66/5F4CFB9E" Ref="CN41"  Part="1" 
F 0 "CN41" H 7550 3350 50  0000 C CNN
F 1 "MIKROBUS_MATE;MEDIUM" H 7700 1550 50  0000 C CNN
F 2 "HR2:MIKROBUS_MEDIUM_MATE" H 7450 3250 60  0001 L CNN
F 3 "MikroBus Mate Medium" H 7450 3050 60  0001 L CNN
F 4 "MikroBus Mate Medium" H 7450 2950 60  0001 L CNN "Field5"
	1    7250 3200
	-1   0    0    -1  
$EndComp
Text HLabel 3750 2500 0    50   Input ~ 0
5V
Text HLabel 3750 2600 0    50   Input ~ 0
3.3V
Text HLabel 3750 2700 0    50   Input ~ 0
GND
Text Label 3850 2500 0    50   ~ 0
5V
Text Label 3850 2600 0    50   ~ 0
3.3V
Text Label 3850 2700 0    50   ~ 0
GND
Wire Wire Line
	4100 2500 3750 2500
Wire Wire Line
	4100 2600 3750 2600
Wire Wire Line
	4100 2700 3750 2700
Entry Wire Line
	4100 2700 4200 2800
Entry Wire Line
	4100 2600 4200 2700
Entry Wire Line
	4100 2500 4200 2600
Text Label 4350 4100 0    50   ~ 0
3.3V
Text Label 4350 3300 0    50   ~ 0
5V
Text Label 4350 3200 0    50   ~ 0
GND
Text Label 4350 4000 0    50   ~ 0
GND
Entry Wire Line
	4300 4100 4200 4000
Entry Wire Line
	4300 4000 4200 3900
Entry Wire Line
	4300 3300 4200 3200
Entry Wire Line
	4300 3200 4200 3100
Wire Wire Line
	4300 4100 4600 4100
Wire Wire Line
	4300 4000 4600 4000
Wire Wire Line
	4300 3300 4600 3300
Wire Wire Line
	4600 3200 4300 3200
Text Label 6200 4100 0    50   ~ 0
3.3V
Text Label 6200 3300 0    50   ~ 0
5V
Text Label 6200 3200 0    50   ~ 0
GND
Text Label 6200 4000 0    50   ~ 0
GND
Entry Wire Line
	6150 4100 6050 4000
Entry Wire Line
	6150 4000 6050 3900
Entry Wire Line
	6150 3300 6050 3200
Entry Wire Line
	6150 3200 6050 3100
Wire Wire Line
	6150 4100 6450 4100
Wire Wire Line
	6150 4000 6450 4000
Wire Wire Line
	6150 3300 6450 3300
Wire Wire Line
	6450 3200 6150 3200
Wire Bus Line
	6050 2400 4200 2400
Wire Bus Line
	6050 2400 6050 4100
Wire Bus Line
	4200 2400 4200 4100
$EndSCHEMATC
