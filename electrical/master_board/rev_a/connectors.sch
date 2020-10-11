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
$Comp
L HR2:MIKROBUS_MATE;SMALL CN?
U 1 1 5F4CFB97
P 4500 3500
AR Path="/5F4CFB97" Ref="CN?"  Part="1" 
AR Path="/5F4CEE66/5F4CFB97" Ref="CN40"  Part="1" 
F 0 "CN40" H 4800 3650 50  0000 C CNN
F 1 "MIKROBUS_MATE;SMALL" H 5050 2250 50  0000 C CNN
F 2 "HR2:MIKROBUS_SMALL_MATE" H 4700 3550 60  0001 L CNN
F 3 "MikroBus Small Mate" H 4700 3350 60  0001 L CNN
F 4 "MikroBus Small Mate" H 4700 3250 60  0001 L CNN "Field5"
	1    4500 3500
	1    0    0    -1  
$EndComp
$Comp
L HR2:MIKROBUS_MATE;MEDIUM CN?
U 1 1 5F4CFB9E
P 4500 1900
AR Path="/5F4CFB9E" Ref="CN?"  Part="1" 
AR Path="/5F4CEE66/5F4CFB9E" Ref="CN41"  Part="1" 
F 0 "CN41" H 4800 2050 50  0000 C CNN
F 1 "MIKROBUS_MATE;MEDIUM" H 5050 650 50  0000 C CNN
F 2 "HR2:MIKROBUS_MEDIUM_MATE" H 4700 1950 60  0001 L CNN
F 3 "MikroBus Mate Medium" H 4700 1750 60  0001 L CNN
F 4 "MikroBus Mate Medium" H 4700 1650 60  0001 L CNN "Field5"
	1    4500 1900
	1    0    0    -1  
$EndComp
Text HLabel 5600 700  0    50   Input ~ 0
5V
Text HLabel 5600 800  0    50   Input ~ 0
3.3V
Text HLabel 5600 900  0    50   Input ~ 0
GND
Text Label 5900 700  2    50   ~ 0
5V
Text Label 5900 800  2    50   ~ 0
3.3V
Text Label 5900 900  2    50   ~ 0
GND
Wire Wire Line
	5900 700  5600 700 
Wire Wire Line
	5900 800  5600 800 
Wire Wire Line
	5900 900  5600 900 
Entry Wire Line
	5900 900  6000 1000
Entry Wire Line
	5900 800  6000 900 
Entry Wire Line
	5900 700  6000 800 
Text Label 5900 3900 2    50   ~ 0
5V
Text Label 5900 4200 2    50   ~ 0
GND
Entry Wire Line
	5900 4200 6000 4100
Wire Wire Line
	5900 3900 5600 3900
Wire Wire Line
	5600 4200 5900 4200
Text Label 5900 2300 2    50   ~ 0
5V
Text Label 5900 4100 2    50   ~ 0
GND
Entry Wire Line
	5900 4100 6000 4000
Entry Wire Line
	5900 2300 6000 2200
Wire Wire Line
	5900 4100 5600 4100
Wire Wire Line
	5900 2300 5600 2300
Text HLabel 2900 5500 0    50   Input ~ 0
LDR_PWM
Text HLabel 2900 7600 0    50   Input ~ 0
SER_IN
Text HLabel 2900 7800 0    50   Input ~ 0
MISC_SCK
Text HLabel 2900 7700 0    50   Input ~ 0
MISC_NSS
$Comp
L HR2:SN74HCS595;TTSOP16 U10
U 1 1 5FD710B8
P 3200 5800
F 0 "U10" H 3450 5950 50  0000 C CNN
F 1 "SN74HCS595;TTSOP16" H 3550 4150 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 3400 5850 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 3400 5650 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 3400 5550 60  0001 L CNN "Field5"
	1    3200 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 7600 4100 7600
Wire Wire Line
	4100 7600 4100 7100
Wire Wire Line
	4100 7100 4000 7100
Entry Wire Line
	4700 7300 4800 7200
Entry Wire Line
	4700 6500 4800 6400
Entry Wire Line
	4700 7000 4800 6900
Entry Wire Line
	4700 6700 4800 6600
NoConn ~ 4000 6000
NoConn ~ 4000 6100
NoConn ~ 4000 6400
NoConn ~ 4000 6600
NoConn ~ 4000 7200
Text Label 4700 7300 2    50   ~ 0
3.3V
Text Label 4700 6700 2    50   ~ 0
3.3V
Text Label 4700 6500 2    50   ~ 0
GND
Text Label 4700 7000 2    50   ~ 0
GND
Wire Wire Line
	4000 6800 4300 6800
Wire Wire Line
	4300 6800 4300 7800
Wire Wire Line
	4300 7800 2900 7800
Wire Wire Line
	2900 7700 4200 7700
Wire Wire Line
	4200 7700 4200 6900
Wire Wire Line
	4200 6900 4000 6900
Wire Wire Line
	2900 5500 4600 5500
Text Label 5900 5300 2    50   ~ 0
GND
Entry Wire Line
	6000 5200 5900 5300
Entry Wire Line
	6000 5300 5900 5400
Wire Wire Line
	2900 5200 4600 5200
Wire Wire Line
	4600 5100 2900 5100
Text Label 3200 5500 0    50   ~ 0
LDR_PWM
Text Label 4200 5400 0    50   ~ 0
LDR_EN
Text Label 4200 5300 0    50   ~ 0
LDR_SPIN
Text Label 3200 5200 0    50   ~ 0
LDR_RX
Text Label 3200 5100 0    50   ~ 0
LDR_TX
Text Label 5900 5400 2    50   ~ 0
5V
Text HLabel 2900 5200 0    50   Input ~ 0
LDR_RX
Text HLabel 2900 5100 0    50   Output ~ 0
LDR_TX
$Comp
L HR2:LIDAR_ADAPTER;2xF1x2 CN?
U 1 1 5F84DF13
P 4600 5100
AR Path="/5F4A826A/5F84DF13" Ref="CN?"  Part="1" 
AR Path="/5F4CEE66/5F84DF13" Ref="CN3"  Part="1" 
F 0 "CN3" H 4850 5250 50  0000 C CNN
F 1 "LIDAR_ADAPTER;2xF1x2" H 5100 4550 50  0000 C CNN
F 2 "HR2:LIDAR_ADAPTER_2xF1x2" H 4800 5150 60  0001 L CNN
F 3 "Lidar Adapter Connectors" H 4800 4950 60  0001 L CNN
F 4 "Lidar Adapter Connectors" H 4800 5050 60  0001 L CNN "manf#"
	1    4600 5100
	1    0    0    -1  
$EndComp
Text Label 5900 5200 2    50   ~ 0
3.3V
Entry Wire Line
	5900 5200 6000 5100
Wire Wire Line
	5600 5200 5900 5200
Wire Wire Line
	5600 5300 5900 5300
Wire Wire Line
	5600 5400 5900 5400
Text Label 2900 7600 0    50   ~ 0
SER_IN
Text Label 2900 7700 0    50   ~ 0
MISC_NSS
Text Label 2900 7800 0    50   ~ 0
MISC_SCK
Wire Wire Line
	4000 5800 4100 5800
Wire Wire Line
	4100 5800 4100 5300
Wire Wire Line
	4100 5300 4600 5300
Wire Wire Line
	4000 5900 4200 5900
Wire Wire Line
	4200 5900 4200 5400
Wire Wire Line
	4200 5400 4600 5400
Wire Wire Line
	5600 2500 5900 2500
Entry Wire Line
	5900 2500 6000 2400
Text Label 5900 2500 2    50   ~ 0
GND
Wire Wire Line
	5600 2600 5900 2600
Entry Wire Line
	5900 2600 6000 2500
Text Label 5900 2600 2    50   ~ 0
GND
Wire Wire Line
	5900 4000 5600 4000
Text Label 5900 4000 2    50   ~ 0
3.3V
Entry Wire Line
	6000 3800 5900 3900
Entry Wire Line
	5900 4000 6000 3900
Text Label 5900 2400 2    50   ~ 0
3.3V
Entry Wire Line
	5900 2400 6000 2300
Wire Wire Line
	5600 2400 5900 2400
Wire Bus Line
	4800 6000 6000 6000
Wire Wire Line
	4000 6500 4700 6500
Wire Wire Line
	4000 6700 4700 6700
Wire Wire Line
	4000 7000 4700 7000
Wire Wire Line
	4000 7300 4700 7300
Text HLabel 2900 3000 0    50   UnSpc ~ 0
A0
Text HLabel 2900 4600 0    50   UnSpc ~ 0
A1
Text HLabel 2900 1500 0    50   Input ~ 0
NRST
Text HLabel 2900 1400 0    50   BiDi ~ 0
A5_SCL
Text HLabel 2900 1300 0    50   BiDi ~ 0
A4_SDA
Text HLabel 2900 1200 0    50   Input ~ 0
D13_SCK
Text HLabel 2900 1100 0    50   BiDi ~ 0
D12_MISO
Text HLabel 2900 1000 0    50   Input ~ 0
D11_PWM_MOSI
Text HLabel 2900 900  0    50   Input ~ 0
D10_NSS
Text HLabel 2900 800  0    50   BiDi ~ 0
D1_TX
Text HLabel 2900 700  0    50   Input ~ 0
D0_RX
Text Label 3600 1000 2    50   ~ 0
D11_PWM_MOSI
Entry Wire Line
	3700 2400 3800 2500
NoConn ~ 4500 2400
Entry Wire Line
	3700 2500 3800 2600
Entry Wire Line
	3700 2600 3800 2700
Entry Wire Line
	3700 2700 3800 2800
Entry Wire Line
	3700 2800 3800 2900
Entry Wire Line
	3700 1800 3800 1900
Entry Wire Line
	3700 1900 3800 2000
Entry Wire Line
	3700 2000 3800 2100
Entry Wire Line
	3800 2200 3700 2100
Entry Wire Line
	3600 1200 3700 1300
Entry Wire Line
	3600 1300 3700 1400
Entry Wire Line
	3600 1400 3700 1500
Entry Wire Line
	3600 1500 3700 1600
Entry Wire Line
	3600 700  3700 800 
Entry Wire Line
	3600 800  3700 900 
Entry Wire Line
	3600 900  3700 1000
Entry Wire Line
	3700 1100 3600 1000
Entry Wire Line
	3600 1100 3700 1200
Text Label 3600 900  2    50   ~ 0
D10_NSS
Text Label 3600 800  2    50   ~ 0
D1_TX
Text Label 3600 700  2    50   ~ 0
D0_RX
Text Label 3600 1100 2    50   ~ 0
D12_MISO
Text Label 3600 1200 2    50   ~ 0
D13_SCK
Text Label 3600 1300 2    50   ~ 0
A4_SDA
Text Label 3600 1400 2    50   ~ 0
A5_SCL
Text Label 3600 1500 2    50   ~ 0
NRST
Text Label 3800 2500 0    50   ~ 0
D11_PWM_MOSI
Text Label 3800 2800 0    50   ~ 0
D10_NSS
Text Label 3800 2100 0    50   ~ 0
D1_TX
Text Label 3800 2200 0    50   ~ 0
D0_RX
Text Label 3800 2600 0    50   ~ 0
D12_MISO
Text Label 3800 2700 0    50   ~ 0
D13_SCK
Text Label 3800 1900 0    50   ~ 0
A4_SDA
Text Label 3800 2000 0    50   ~ 0
A5_SCL
Text Label 3800 2900 0    50   ~ 0
NRST
Wire Wire Line
	2900 700  3600 700 
Wire Wire Line
	3600 800  2900 800 
Wire Wire Line
	2900 900  3600 900 
Wire Wire Line
	3600 1000 2900 1000
Wire Wire Line
	2900 1100 3600 1100
Wire Wire Line
	3600 1200 2900 1200
Wire Wire Line
	2900 1300 3600 1300
Wire Wire Line
	3600 1400 2900 1400
Wire Wire Line
	2900 1500 3600 1500
Wire Wire Line
	4500 1900 3800 1900
Wire Wire Line
	3800 2000 4500 2000
Wire Wire Line
	4500 2100 3800 2100
Wire Wire Line
	3800 2200 4500 2200
Wire Wire Line
	4500 2500 3800 2500
Wire Wire Line
	3800 2600 4500 2600
Wire Wire Line
	4500 2700 3800 2700
Wire Wire Line
	3800 2800 4500 2800
Wire Wire Line
	4500 2900 3800 2900
Wire Wire Line
	4500 2300 2900 2300
Entry Wire Line
	3700 4000 3800 4100
Entry Wire Line
	3700 4100 3800 4200
Entry Wire Line
	3700 4200 3800 4300
Entry Wire Line
	3700 4300 3800 4400
Entry Wire Line
	3700 4400 3800 4500
Entry Wire Line
	3700 3400 3800 3500
Entry Wire Line
	3700 3500 3800 3600
Entry Wire Line
	3700 3600 3800 3700
Entry Wire Line
	3800 3800 3700 3700
Text Label 3800 4100 0    50   ~ 0
D11_PWM_MOSI
Text Label 3800 4400 0    50   ~ 0
D10_NSS
Text Label 3800 3700 0    50   ~ 0
D1_TX
Text Label 3800 3800 0    50   ~ 0
D0_RX
Text Label 3800 4200 0    50   ~ 0
D12_MISO
Text Label 3800 4300 0    50   ~ 0
D13_SCK
Text Label 3800 3500 0    50   ~ 0
A4_SDA
Text Label 3800 3600 0    50   ~ 0
A5_SCL
Text Label 3800 4500 0    50   ~ 0
NRST
Wire Wire Line
	4500 3500 3800 3500
Wire Wire Line
	3800 3600 4500 3600
Wire Wire Line
	4500 3700 3800 3700
Wire Wire Line
	3800 3800 4500 3800
Wire Wire Line
	4500 4100 3800 4100
Wire Wire Line
	3800 4200 4500 4200
Wire Wire Line
	4500 4300 3800 4300
Wire Wire Line
	3800 4400 4500 4400
Wire Wire Line
	4500 4500 3800 4500
Wire Wire Line
	4500 3900 2900 3900
Wire Wire Line
	2900 3000 4500 3000
Wire Wire Line
	2900 4600 4500 4600
Text HLabel 2900 3900 0    50   Output ~ 0
INT1
Text HLabel 2900 2300 0    50   Output ~ 0
INT0
NoConn ~ 4500 4000
NoConn ~ 4000 6200
NoConn ~ 4000 6300
Wire Bus Line
	4800 6000 4800 7300
Wire Bus Line
	6000 700  6000 6000
Wire Bus Line
	3700 700  3700 4500
$EndSCHEMATC
