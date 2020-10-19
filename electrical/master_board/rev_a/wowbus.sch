EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 10
Title "HR2: WOWBus"
Date "2020-10-17"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L HR2:MCP2542;SOIC8 U14
U 1 1 5F846C97
P 4800 3100
F 0 "U14" H 5050 3250 50  0000 C CNN
F 1 "LTV-827" H 5350 2550 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5000 3150 60  0001 L CNN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 5000 2950 60  0001 L CNN
F 4 "MCP2542" H 5000 2850 60  0001 L CNN "Field5"
	1    4800 3100
	1    0    0    -1  
$EndComp
$Comp
L Isolator:LTV-827 U17
U 1 1 5F8476A1
P 6800 2300
F 0 "U17" H 6650 2500 50  0000 C CNN
F 1 "LTV-827" H 6800 2100 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 6600 2100 50  0001 L CIN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 6800 2300 50  0001 L CNN
	1    6800 2300
	1    0    0    -1  
$EndComp
$Comp
L Isolator:LTV-827 U18
U 1 1 5F849475
P 7800 2300
F 0 "U18" H 7950 2100 50  0000 C CNN
F 1 "LTV-827" H 7800 2500 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 7600 2100 50  0001 L CIN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 7800 2300 50  0001 L CNN
	1    7800 2300
	-1   0    0    1   
$EndComp
Wire Wire Line
	7100 2400 7400 2400
Wire Wire Line
	7100 2200 7200 2200
Wire Wire Line
	6500 2400 6400 2400
Wire Wire Line
	6400 2600 8200 2600
Wire Wire Line
	8200 2600 8200 2400
Wire Wire Line
	8200 2400 8100 2400
Wire Wire Line
	6400 2400 6400 2600
$Comp
L Device:R_US R41
U 1 1 5F846FDA
P 7400 2850
F 0 "R41" H 7250 2950 50  0000 L CNN
F 1 "120Ω1/4W;1608" H 7450 2750 50  0000 L CNN
F 2 "" V 7440 2840 50  0001 C CNN
F 3 "~" H 7400 2850 50  0001 C CNN
	1    7400 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 3100 7200 3100
Wire Wire Line
	7200 3100 7200 2200
Connection ~ 7200 2200
Wire Wire Line
	7200 2200 7500 2200
Wire Wire Line
	7400 2700 7400 2400
Connection ~ 7400 2400
Wire Wire Line
	7400 2400 7500 2400
Wire Wire Line
	5900 3200 7400 3200
Wire Wire Line
	7400 3200 7400 3000
$Comp
L HR2:POWER_PFET_GSD Q6
U 1 1 5F84ED87
P 3450 1800
F 0 "Q6" H 3700 1950 60  0000 C CNN
F 1 "PFET_6A_GSD;SOT23" H 3850 1550 50  0000 C CNN
F 2 "HR2:" H 3650 1850 60  0001 L CNN
F 3 "Power PFG (GSD pinout)" H 3650 1650 60  0001 L CNN
F 4 "Power PFG (GSD pinout)" H 3650 1750 60  0001 L CNN "manf#"
F 5 "Power PFET (GSD pinout)" H 3650 1550 60  0001 L CNN "Field5"
	1    3450 1800
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F86962A
P 3400 5600
AR Path="/5F79BC00/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5F86962A" Ref="Q5"  Part="1" 
F 0 "Q5" H 3300 5750 50  0000 L CNN
F 1 "2N7000;SOT23" H 2900 5400 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3600 5525 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 3400 5600 50  0001 L CNN
	1    3400 5600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R37
U 1 1 5F869783
P 2800 2100
F 0 "R37" H 2650 2200 50  0000 L CNN
F 1 "100KΩ;1608" H 2850 2000 50  0000 L CNN
F 2 "" V 2840 2090 50  0001 C CNN
F 3 "~" H 2800 2100 50  0001 C CNN
	1    2800 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1800 2800 1800
Wire Wire Line
	2800 1800 2800 1950
Wire Wire Line
	2800 2350 3350 2350
Connection ~ 2800 2350
Wire Wire Line
	2800 2350 2800 2250
Wire Wire Line
	6300 3500 5900 3500
Wire Wire Line
	4700 3400 4800 3400
Text HLabel 1000 1800 0    50   Input ~ 0
9V
Wire Wire Line
	1000 1800 2800 1800
Connection ~ 2800 1800
Text HLabel 1000 6800 0    50   Input ~ 0
GND
Text HLabel 1000 5600 0    50   Input ~ 0
WOW_EN
$Comp
L Device:R_US R38
U 1 1 5F873ADD
P 3500 4150
F 0 "R38" H 3350 4250 50  0000 L CNN
F 1 "100KΩ;1608" H 3550 4050 50  0000 L CNN
F 2 "" V 3540 4140 50  0001 C CNN
F 3 "~" H 3500 4150 50  0001 C CNN
	1    3500 4150
	1    0    0    -1  
$EndComp
$Comp
L HR2:WOW_OUT;M2x6S CN8
U 1 1 5F848F57
P 10300 1900
F 0 "CN8" H 10500 2050 50  0000 L CNN
F 1 "WOW_OUT;M2x6S" H 10450 650 50  0000 L CNN
F 2 "Connector_PinHeader_2.54:PinHeader_2x06_P2.54mm_Vertical" H 10500 1950 60  0001 L CNN
F 3 "WOW Output Connector" H 10500 1750 60  0001 L CNN
F 4 "WOW Output Connector" H 10500 1650 60  0001 L CNN "Field5"
	1    10300 1900
	1    0    0    -1  
$EndComp
$Comp
L HR2:MCP2542;SOIC8 U15
U 1 1 5F8496E2
P 4800 4800
F 0 "U15" H 5050 4950 50  0000 C CNN
F 1 "LTV-827" H 5350 4250 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5000 4850 60  0001 L CNN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 5000 4650 60  0001 L CNN
F 4 "MCP2542" H 5000 4550 60  0001 L CNN "Field5"
	1    4800 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3400 4700 3800
Wire Wire Line
	4700 5100 4800 5100
$Comp
L Isolator:LTV-827 U17
U 2 1 5F84B285
P 6800 4000
F 0 "U17" H 6650 4200 50  0000 C CNN
F 1 "LTV-827" H 6800 3800 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 6600 3800 50  0001 L CIN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 6800 4000 50  0001 L CNN
	2    6800 4000
	1    0    0    -1  
$EndComp
$Comp
L Isolator:LTV-827 U18
U 2 1 5F84B28F
P 7800 4000
F 0 "U18" H 7950 3800 50  0000 C CNN
F 1 "LTV-827" H 7800 4200 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 7600 3800 50  0001 L CIN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 7800 4000 50  0001 L CNN
	2    7800 4000
	-1   0    0    1   
$EndComp
Wire Wire Line
	6500 4100 6400 4100
Wire Wire Line
	6400 4300 8200 4300
Wire Wire Line
	6400 4100 6400 4300
$Comp
L Device:R_US R42
U 1 1 5F84B29E
P 7400 4550
F 0 "R42" H 7250 4650 50  0000 L CNN
F 1 "120Ω1/4W;1608" H 7450 4450 50  0000 L CNN
F 2 "" V 7440 4540 50  0001 C CNN
F 3 "~" H 7400 4550 50  0001 C CNN
	1    7400 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 4800 7200 4800
Wire Wire Line
	7400 4400 7400 4100
Wire Wire Line
	5900 4900 7400 4900
Wire Wire Line
	7400 4900 7400 4700
Wire Wire Line
	7200 4800 7200 3900
Wire Wire Line
	8100 4100 8200 4100
Wire Wire Line
	8200 4100 8200 4300
Wire Wire Line
	7100 4100 7400 4100
Wire Wire Line
	7100 3900 7200 3900
Connection ~ 7400 4100
Wire Wire Line
	7400 4100 7500 4100
Connection ~ 7200 3900
Wire Wire Line
	7200 3900 7500 3900
Wire Wire Line
	7200 4800 9900 4800
Wire Wire Line
	9900 4800 9900 2600
Wire Wire Line
	9900 2600 10300 2600
Connection ~ 7200 4800
Wire Wire Line
	7400 4900 9800 4900
Wire Wire Line
	9800 4900 9800 2500
Wire Wire Line
	9800 2500 10300 2500
Connection ~ 7400 4900
Wire Wire Line
	7400 3200 9600 3200
Wire Wire Line
	9600 3200 9600 2300
Wire Wire Line
	9600 2300 10300 2300
Connection ~ 7400 3200
Wire Wire Line
	7200 3100 9700 3100
Wire Wire Line
	9700 3100 9700 2400
Wire Wire Line
	9700 2400 10300 2400
Connection ~ 7200 3100
Wire Wire Line
	3450 1900 3350 1900
Wire Wire Line
	3350 1900 3350 2350
Wire Wire Line
	10300 3000 10100 3000
Wire Wire Line
	10100 3000 10100 2900
Wire Wire Line
	10100 1900 10300 1900
Wire Wire Line
	10300 2000 10100 2000
Connection ~ 10100 2000
Wire Wire Line
	10100 2000 10100 1900
Wire Wire Line
	10300 2900 10100 2900
Connection ~ 10100 2900
Wire Wire Line
	10100 2900 10100 2000
Wire Wire Line
	4250 1900 10100 1900
Connection ~ 10100 1900
Wire Wire Line
	10300 2100 10200 2100
Wire Wire Line
	10200 2100 10200 2200
Wire Wire Line
	10200 2800 10300 2800
Wire Wire Line
	10300 2700 10200 2700
Connection ~ 10200 2700
Wire Wire Line
	10200 2700 10200 2800
Wire Wire Line
	10300 2200 10200 2200
Connection ~ 10200 2200
Wire Wire Line
	10200 2200 10200 2700
Wire Wire Line
	10200 2800 10200 3500
Connection ~ 10200 2800
Connection ~ 6300 3500
Text HLabel 1000 3900 0    50   Input ~ 0
3.3V
Wire Wire Line
	6200 3400 5900 3400
Wire Wire Line
	5900 5100 6200 5100
Wire Wire Line
	8100 2200 9000 2200
Wire Wire Line
	8100 3900 8800 3900
Text HLabel 1000 2500 0    50   Input ~ 0
5V
Wire Wire Line
	1000 2500 6100 2500
Wire Wire Line
	6100 2500 6100 3300
Wire Wire Line
	6100 3300 5900 3300
Wire Wire Line
	5900 5000 6100 5000
Wire Wire Line
	6100 5000 6100 3300
Connection ~ 6100 3300
Wire Wire Line
	5900 5200 6300 5200
Wire Wire Line
	6200 2200 6500 2200
Connection ~ 6200 3400
Wire Wire Line
	6500 3900 6200 3900
Connection ~ 6200 3900
Wire Wire Line
	6200 3900 6200 3400
Wire Wire Line
	6300 6800 6300 6200
Connection ~ 6300 5200
Wire Wire Line
	6200 3900 6200 5100
Wire Wire Line
	6300 3500 6300 5200
$Comp
L Device:R_US R43
U 1 1 5F8E9E6C
P 9000 4150
F 0 "R43" H 8850 4250 50  0000 L CNN
F 1 "100KΩ;1608" H 9050 4050 50  0000 L CNN
F 2 "" V 9040 4140 50  0001 C CNN
F 3 "~" H 9000 4150 50  0001 C CNN
	1    9000 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5800 3500 6800
Connection ~ 3500 6800
Wire Wire Line
	1000 3900 1700 3900
Wire Wire Line
	3500 3900 3500 4000
$Comp
L Device:R_US R35
U 1 1 5F98DD9D
P 1600 6500
F 0 "R35" H 1450 6600 50  0000 L CNN
F 1 "100KΩ;1608" H 1600 6350 50  0000 L CNN
F 2 "" V 1640 6490 50  0001 C CNN
F 3 "~" H 1600 6500 50  0001 C CNN
	1    1600 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 6350 1600 5600
Connection ~ 3500 3900
Wire Wire Line
	6200 2200 6200 3400
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F9A0944
P 2700 4500
AR Path="/5F79BC00/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5F9A0944" Ref="Q4"  Part="1" 
F 0 "Q4" H 2600 4650 50  0000 L CNN
F 1 "2N7000;SOT23" H 2850 4350 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2900 4425 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 2700 4500 50  0001 L CNN
	1    2700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 2350 2800 4300
Wire Wire Line
	6300 3500 10200 3500
Text HLabel 1000 3200 0    50   Input ~ 0
WOW_TX
Text HLabel 1000 3300 0    50   Output ~ 0
WOW_RX
Wire Wire Line
	4800 3200 1000 3200
Wire Wire Line
	1000 3300 4800 3300
Wire Wire Line
	9000 2200 9000 2300
$Comp
L Device:R_US R60
U 1 1 5F8EAE9D
P 9000 2450
F 0 "R60" H 8850 2550 50  0000 L CNN
F 1 "100KΩ;1608" H 9050 2350 50  0000 L CNN
F 2 "" V 9040 2440 50  0001 C CNN
F 3 "~" H 9000 2450 50  0001 C CNN
	1    9000 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2600 9000 3600
Wire Wire Line
	9000 4400 8800 4400
$Comp
L HR2:74LVC1G74;TSSOP8 U16
U 1 1 5F8C5AF5
P 4800 5900
F 0 "U16" H 5050 6050 50  0000 C CNN
F 1 "74LVC1G74;TSSOP8" H 5350 5450 50  0000 C CNN
F 2 "Package_SO:TSSOP-8_3x3mm_P0.65mm" H 5000 5950 60  0001 L CNN
F 3 "D-type flip-flop edge-trigger/set/reset" H 5000 5750 60  0001 L CNN
F 4 "D-type flip-flop edge-trigger/set/reset" H 5000 5650 60  0001 L CNN "Field5"
	1    4800 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 6100 5900 6100
Wire Wire Line
	6100 5900 5900 5900
Connection ~ 6100 5000
Wire Wire Line
	5900 6200 6300 6200
Connection ~ 6300 6200
Wire Wire Line
	6300 6200 6300 5200
Wire Wire Line
	6000 6100 6000 7200
Wire Wire Line
	6000 7200 1000 7200
Wire Wire Line
	5900 6000 6100 6000
Wire Wire Line
	6100 6000 6100 7300
Wire Wire Line
	6100 7300 1000 7300
Text HLabel 1000 7300 0    50   Output ~ 0
ESTOP
Text HLabel 1000 7200 0    50   Output ~ 0
~ESTOP
Wire Wire Line
	6100 5000 6100 5700
Text HLabel 1000 6000 0    50   Input ~ 0
~ESTOP_CLR
Text HLabel 1000 6100 0    50   Input ~ 0
~NRST
Wire Wire Line
	1000 6000 1700 6000
Wire Wire Line
	6100 5700 4700 5700
Wire Wire Line
	4700 5700 4700 5900
Wire Wire Line
	4700 6000 4800 6000
Connection ~ 6100 5700
Wire Wire Line
	6100 5700 6100 5900
Wire Wire Line
	4800 5900 4700 5900
Connection ~ 4700 5900
Wire Wire Line
	4700 5900 4700 6000
Text Notes 6350 5650 0    50   ~ 0
Red ESTOP LED
$Comp
L 74xGxx:74LVC1G08 U13
U 1 1 5FB2A0BD
P 2200 6050
F 0 "U13" H 2100 6200 50  0000 C CNN
F 1 "74LVC1G08;TTSOP8" H 2600 5900 50  0000 C CNN
F 2 "" H 2200 6050 50  0001 C CNN
F 3 "http://www.ti.com/lit/sg/scyt129e/scyt129e.pdf" H 2200 6050 50  0001 C CNN
	1    2200 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 6100 1900 6100
Wire Wire Line
	2200 6150 2200 6800
Connection ~ 2200 6800
Wire Wire Line
	2200 5950 2200 3900
Connection ~ 2200 3900
Wire Wire Line
	2200 3900 3500 3900
Wire Wire Line
	4800 6200 3100 6200
Wire Wire Line
	3100 6200 3100 6050
Wire Wire Line
	3100 6050 2450 6050
Text Notes 2000 5950 0    40   ~ 0
5
Text Notes 1950 6250 0    40   ~ 0
3
Wire Wire Line
	2200 6800 2800 6800
Wire Wire Line
	2800 4700 2800 6800
Connection ~ 2800 6800
Wire Wire Line
	2800 6800 3500 6800
Text Notes 5150 2950 0    50   ~ 0
CANBus Transceiver
Text Notes 5150 4650 0    50   ~ 0
CANBus Transceiver
Text Notes 3650 1550 0    40   ~ 0
6 Amp PFET
Text Notes 5000 6500 0    50   ~ 0
'74 D Flip-Flop
Text Notes 6950 2000 0    50   ~ 0
Opto-isolator
Text Notes 7100 3750 0    50   ~ 0
Opto-isolator
Text Notes 9950 1650 0    50   ~ 0
WOWBus Output Connector
Wire Wire Line
	1600 6800 2200 6800
Connection ~ 1600 6800
Wire Wire Line
	1600 6650 1600 6800
Wire Wire Line
	1000 6800 1600 6800
Wire Wire Line
	1000 5600 1600 5600
Wire Wire Line
	2400 4500 2400 5600
Wire Wire Line
	2400 4500 2500 4500
Wire Wire Line
	2400 5600 3200 5600
$Comp
L Device:R_US R36
U 1 1 5FC61F4D
P 1700 4850
F 0 "R36" H 1550 4950 50  0000 L CNN
F 1 "100KΩ;1608" H 1700 4700 50  0000 L CNN
F 2 "" V 1740 4840 50  0001 C CNN
F 3 "~" H 1700 4850 50  0001 C CNN
	1    1700 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 4700 1700 3900
Connection ~ 1700 3900
Wire Wire Line
	1700 3900 2200 3900
Wire Wire Line
	1700 5000 1700 6000
Connection ~ 1700 6000
Wire Wire Line
	1700 6000 1900 6000
$Comp
L Device:LED D?
U 1 1 5FC8C292
P 6500 6450
AR Path="/5F79BC00/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5FA4A874/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5FAD82EE/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5F8C2F57/5FC8C292" Ref="D2"  Part="1" 
F 0 "D2" V 6600 6550 50  0000 R CNN
F 1 "REDLED;1608" V 6450 6350 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 6500 6450 50  0001 C CNN
F 3 "~" H 6500 6450 50  0001 C CNN
	1    6500 6450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R40
U 1 1 5FC946D0
P 6700 5950
F 0 "R40" H 6550 6050 50  0000 L CNN
F 1 "470Ω;1608" H 6750 5850 50  0000 L CNN
F 2 "" V 6740 5940 50  0001 C CNN
F 3 "~" H 6700 5950 50  0001 C CNN
	1    6700 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 6800 6500 6800
Wire Wire Line
	6500 6800 6500 6600
Connection ~ 6300 6800
Wire Wire Line
	6100 6000 6400 6000
Wire Wire Line
	6400 6000 6400 5700
Wire Wire Line
	6400 5700 6700 5700
Wire Wire Line
	6700 5700 6700 5800
Connection ~ 6100 6000
Text Label 8150 3100 0    50   ~ 0
WOW_COMH
Text Label 8150 3200 0    50   ~ 0
WOW_COML
Text Label 7900 4800 0    50   ~ 0
WOW_STOPH
Text Label 7900 4900 0    50   ~ 0
WOW_STOPL
Text Label 1000 1800 0    50   ~ 0
9V
Text Label 1000 2500 0    50   ~ 0
5V
Text Label 1000 3200 0    50   ~ 0
WOW_TX
Text Label 1000 3300 0    50   ~ 0
WOW_RX
Text Label 1000 3900 0    50   ~ 0
3.3V
Text Label 1000 5600 0    50   ~ 0
WOW_EN
Text Label 1000 6000 0    50   ~ 0
~ESTOP_CLR
Text Label 1000 6100 0    50   ~ 0
~NRST
Text Label 1000 7200 0    50   ~ 0
~ESTOP
Text Label 1000 7300 0    50   ~ 0
ESTOP
Text Label 1000 6800 0    50   ~ 0
GND
Text Label 4250 6200 0    50   ~ 0
~ESTOP_RESET
Text Label 4250 6100 0    50   ~ 0
ESTOP_TRIG
Text Label 4400 5000 0    50   ~ 0
~ESTOP
Text Label 4100 4500 0    50   ~ 0
WOW_STDBY
Wire Wire Line
	2400 5600 1600 5600
Connection ~ 2400 5600
Connection ~ 1600 5600
Text Label 7900 1900 0    50   ~ 0
WOW_POW_OUT
Text Label 7500 2600 0    50   ~ 0
WOW_COM_TERM
Text Label 7450 4300 0    50   ~ 0
WOW_ESTOP_TERM
Wire Wire Line
	4700 3800 6000 3800
Wire Wire Line
	6000 3800 6000 3600
Wire Wire Line
	6000 3600 9000 3600
Connection ~ 4700 3800
Wire Wire Line
	4700 3800 4700 4500
Wire Wire Line
	9000 4000 9000 3600
Connection ~ 9000 3600
Wire Wire Line
	9000 4400 9000 4300
Wire Wire Line
	8800 3900 8800 4400
Text Label 8150 3900 0    50   ~ 0
WOW_ESTOP_OPTO
Text Label 8150 2200 0    50   ~ 0
WOW_COM_OPTO
Text Notes 7200 6450 0    50   ~ 0
WOWBus Notes:\n\n1. WOWBus is an an expansion bus.\n2. WOWBus stands for Wayne's Omnipotent Wonderful Bus (heavy sarcasm.)\n3. WOWBus uses 2x6 ribbon daisy chain cables for interconnect.\n4. WOWBus has a nominal 9V rail, but this can droop due to resistive losses down the daisy chain.\n5. The WOWBus connector is designed to mirror image power, ground, and signaling.\n6. If a WOWBus connector is accidently reversed on a daisy cable, nothing shorts out.\n7. However, an accidentally reversed cable, will basically make the bus non-fuctional until corrected.\n8. The WOWBus has two "CAN bus" (ISO-11898-2) physical differential pairs for siginalling.\n9. The first differential pair is used as a multi-drop bidirectional data transfer.\n10. The second differential pair is used to signal Emergency Stop.\n11. When the WOWBus is enabled, it powers up both the transceivers and the differential\n    pair termination resistors.\n12. The expansions boards are responsible for providing the bus terminator resistors for\n    for the other end of the differential pairs.\n13. The Bus powers up in the EStop conndition and must be explicitly cleared out.\n14. When ESTOP occurs, it latches up and must be explicitly cleared.\n
Wire Wire Line
	6500 6300 6500 6200
Wire Wire Line
	6500 6200 6700 6200
Wire Wire Line
	6700 6200 6700 6100
Wire Wire Line
	4800 6100 4200 6100
Wire Wire Line
	4800 4900 4200 4900
Wire Wire Line
	4200 4900 4200 6100
Connection ~ 4700 4500
Wire Wire Line
	4700 4500 4700 5100
Wire Wire Line
	6000 6100 6000 5500
Wire Wire Line
	6000 5500 4300 5500
Wire Wire Line
	4300 5500 4300 5000
Wire Wire Line
	4300 5000 4800 5000
Connection ~ 6000 6100
Wire Wire Line
	3500 6800 6300 6800
Wire Wire Line
	3500 3900 6200 3900
Wire Wire Line
	3500 4300 3500 4500
Connection ~ 3500 4500
Wire Wire Line
	3500 4500 3500 5400
Wire Wire Line
	3500 4500 4700 4500
$EndSCHEMATC
