EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "LD06 Lidar Adapter"
Date "2020-12-28"
Rev "A"
Comp "Homebrew Robotics Club"
Comment1 "Copyright © 2020 by Hombrew Robotics Club"
Comment2 "License: MIT"
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	3200 2800 3100 2800
Wire Wire Line
	3100 2800 3100 2900
Wire Wire Line
	3100 3100 3200 3100
Wire Wire Line
	3200 3000 3100 3000
Connection ~ 3100 3000
Wire Wire Line
	3100 3000 3100 3100
Wire Wire Line
	3200 2900 3100 2900
Connection ~ 3100 2900
Wire Wire Line
	3100 2900 3100 3000
NoConn ~ 4200 2900
NoConn ~ 4200 2800
Wire Wire Line
	5000 2900 4700 2900
Wire Wire Line
	4700 2900 4700 3400
Wire Wire Line
	4700 3400 3100 3400
Wire Wire Line
	3100 3400 3100 3100
Connection ~ 3100 3100
Wire Wire Line
	4200 2600 4600 2600
Wire Wire Line
	4600 2600 4600 2700
Wire Wire Line
	4600 2700 5000 2700
Wire Wire Line
	4200 3000 4600 3000
Wire Wire Line
	4600 3000 4600 2800
Wire Wire Line
	4600 2800 5000 2800
Wire Wire Line
	3200 2700 3000 2700
Wire Wire Line
	3000 2700 3000 3500
Wire Wire Line
	3000 3500 4800 3500
Wire Wire Line
	4800 3500 4800 3000
Wire Wire Line
	4800 3000 5000 3000
NoConn ~ 3200 2600
NoConn ~ 4200 2700
$Comp
L HR2:LD06_CONN;M1x4P1.5ZH CN2
U 1 1 5FEA6F47
P 5000 2700
F 0 "CN2" H 5200 2850 50  0000 L CNN
F 1 "LD06_CONN;M1x4P1.5ZH" H 4900 2250 50  0000 L CNN
F 2 "Connector_Wuerth:Wuerth_WR-WTB_64800411622_1x04_P1.50mm_Vertical" H 5200 2750 60  0001 L CNN
F 3 "LD06 Lidar Connector" H 5200 2550 60  0001 L CNN
F 4 "https://www.inno-maker.com/wp-content/uploads/2020/11/LDROBOT_LD06_Datasheet.pdf" H 5200 2650 60  0001 L CNN "manf#"
F 5 "LD06 Lidar COnnector" H 5200 2450 60  0001 L CNN "desc"
	1    5000 2700
	1    0    0    -1  
$EndComp
$Comp
L HR2:LIDAR_ADAPTER;2xM1x4+M1x3 CN1
U 1 1 5FEA71BE
P 4200 2600
F 0 "CN1" H 4950 2750 50  0000 C CNN
F 1 "LIDAR_ADAPTER;2xM1x4+M1x3" H 4700 1950 50  0000 C CNN
F 2 "HR2:LIDAR_ADAPTER_2xF1x4+F1x3" H 4400 2650 60  0001 L CNN
F 3 "Lidar Adapter Connectors" H 4400 2450 60  0001 L CNN
F 4 "Lidar Adapter Connectors" H 4400 2350 60  0001 L CNN "desc"
	1    4200 2600
	-1   0    0    -1  
$EndComp
Text Label 3100 3400 0    50   ~ 0
GND
Text Label 3000 3500 0    50   ~ 0
5V
Text Label 4600 2700 0    50   ~ 0
TX
Text Label 4600 3000 2    50   ~ 0
PWM
$EndSCHEMATC
