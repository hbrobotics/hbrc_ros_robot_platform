EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 3 10
Title "HR2 Connectors"
Date "2020-10-17"
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
Text HLabel 2000 7500 0    50   Input ~ 0
5V
Text HLabel 2000 7600 0    50   Input ~ 0
3.3V
Text HLabel 2000 7700 0    50   Input ~ 0
GND
Text Label 2300 7500 2    50   ~ 0
5V
Text Label 2300 7600 2    50   ~ 0
3.3V
Text Label 2300 7700 2    50   ~ 0
GND
Wire Wire Line
	2300 7500 2000 7500
Wire Wire Line
	2300 7600 2000 7600
Wire Wire Line
	2300 7700 2000 7700
Entry Wire Line
	2300 7700 2400 7600
Entry Wire Line
	2300 7600 2400 7500
Entry Wire Line
	2300 7500 2400 7400
Text Label 5900 3900 2    50   ~ 0
5V
Text Label 5900 4200 2    50   ~ 0
GND
Entry Wire Line
	6000 4300 5900 4200
Wire Wire Line
	5900 3900 5600 3900
Wire Wire Line
	5600 4200 5900 4200
Text Label 5900 2300 2    50   ~ 0
5V
Text Label 5900 4100 2    50   ~ 0
GND
Entry Wire Line
	6000 4200 5900 4100
Entry Wire Line
	5900 2300 6000 2400
Wire Wire Line
	5900 4100 5600 4100
Wire Wire Line
	5900 2300 5600 2300
Text HLabel 2000 5500 0    50   Input ~ 0
LDR_PWM
Text HLabel 2000 6100 0    50   Input ~ 0
SER_IN
Text HLabel 2000 6200 0    50   Input ~ 0
MISC_SCK
Text HLabel 2000 6300 0    50   Input ~ 0
MISC_NSS
$Comp
L HR2:SN74HCS595;TTSOP16 U10
U 1 1 5FD710B8
P 2800 5800
F 0 "U10" H 3050 5950 50  0000 C CNN
F 1 "SN74HCS595;TTSOP16" H 3300 4650 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 3000 5850 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 3000 5650 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 3000 5550 60  0001 L CNN "Field5"
	1    2800 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 6100 2800 6100
Entry Wire Line
	4600 5900 4500 5800
Entry Wire Line
	4600 6900 4500 6800
Text Label 4500 5800 2    50   ~ 0
3.3V
Text Label 4500 6800 2    50   ~ 0
GND
Wire Wire Line
	2800 6200 2000 6200
Wire Wire Line
	2000 6300 2800 6300
Wire Wire Line
	2000 5500 4600 5500
Text Label 5900 5300 2    50   ~ 0
GND
Entry Wire Line
	5900 5300 6000 5400
Entry Wire Line
	5900 5400 6000 5500
Wire Wire Line
	2000 5200 4600 5200
Wire Wire Line
	4600 5100 2000 5100
Text Label 2000 5500 0    50   ~ 0
LDR_PWM
Text Label 4200 5400 0    50   ~ 0
LDR_EN
Text Label 4200 5300 0    50   ~ 0
LDR_SPIN
Text Label 2000 5200 0    50   ~ 0
LDR_RX
Text Label 2000 5100 0    50   ~ 0
LDR_TX
Text Label 5900 5400 2    50   ~ 0
5V
Text HLabel 2000 5200 0    50   Input ~ 0
LDR_RX
Text HLabel 2000 5100 0    50   Output ~ 0
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
	6000 5300 5900 5200
Wire Wire Line
	5600 5200 5900 5200
Wire Wire Line
	5600 5300 5900 5300
Wire Wire Line
	5600 5400 5900 5400
Text Label 2000 6100 0    50   ~ 0
SER_IN
Text Label 2000 6300 0    50   ~ 0
MISC_NSS
Text Label 2000 6200 0    50   ~ 0
MISC_SCK
Wire Wire Line
	3900 5900 4100 5900
Wire Wire Line
	4100 5900 4100 5300
Wire Wire Line
	4100 5300 4600 5300
Wire Wire Line
	3900 6000 4200 6000
Wire Wire Line
	4200 6000 4200 5400
Wire Wire Line
	4200 5400 4600 5400
Wire Wire Line
	5600 2500 5900 2500
Entry Wire Line
	5900 2500 6000 2600
Text Label 5900 2500 2    50   ~ 0
GND
Wire Wire Line
	5600 2600 5900 2600
Entry Wire Line
	5900 2600 6000 2700
Text Label 5900 2600 2    50   ~ 0
GND
Wire Wire Line
	5900 4000 5600 4000
Text Label 5900 4000 2    50   ~ 0
3.3V
Entry Wire Line
	5900 3900 6000 4000
Entry Wire Line
	6000 4100 5900 4000
Text Label 5900 2400 2    50   ~ 0
3.3V
Entry Wire Line
	5900 2400 6000 2500
Wire Wire Line
	5600 2400 5900 2400
Wire Bus Line
	4600 5800 6000 5800
Wire Wire Line
	3900 6800 4000 6800
Wire Wire Line
	3900 5800 4500 5800
Text HLabel 2000 3000 0    50   UnSpc ~ 0
A0
Text HLabel 2000 4600 0    50   UnSpc ~ 0
A1
Text HLabel 2000 7800 0    50   Input ~ 0
~NRST
Text HLabel 2000 1400 0    50   BiDi ~ 0
A5_SCL
Text HLabel 2000 1300 0    50   BiDi ~ 0
A4_SDA
Text HLabel 2000 1200 0    50   Input ~ 0
D13_SCK
Text HLabel 2000 1100 0    50   BiDi ~ 0
D12_MISO
Text HLabel 2000 1000 0    50   Input ~ 0
D11_PWM_MOSI
Text HLabel 2000 900  0    50   Input ~ 0
D10_NSS
Text HLabel 2000 800  0    50   BiDi ~ 0
D1_TX
Text HLabel 2000 700  0    50   Input ~ 0
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
	2300 7800 2400 7700
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
Text Label 2300 7800 2    50   ~ 0
~NRST
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
Wire Wire Line
	2000 700  3600 700 
Wire Wire Line
	3600 800  2000 800 
Wire Wire Line
	2000 900  3600 900 
Wire Wire Line
	3600 1000 2000 1000
Wire Wire Line
	2000 1100 3600 1100
Wire Wire Line
	3600 1200 2000 1200
Wire Wire Line
	2000 1300 3600 1300
Wire Wire Line
	3600 1400 2000 1400
Wire Wire Line
	2000 7800 2300 7800
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
	4500 2900 4400 2900
Wire Wire Line
	4500 2300 2000 2300
Entry Wire Line
	3700 4000 3800 4100
Entry Wire Line
	3700 4100 3800 4200
Entry Wire Line
	3700 4200 3800 4300
Entry Wire Line
	3700 4300 3800 4400
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
Text Label 5900 4900 2    50   ~ 0
~NRST
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
	4500 3900 2000 3900
Wire Wire Line
	2000 3000 4500 3000
Wire Wire Line
	2000 4600 4500 4600
Text HLabel 2000 3900 0    50   Output ~ 0
INT1
Text HLabel 2000 2300 0    50   Output ~ 0
INT0
NoConn ~ 4500 4000
NoConn ~ 3900 6700
NoConn ~ 3900 6500
NoConn ~ 3900 6400
NoConn ~ 3900 6300
NoConn ~ 3900 6200
NoConn ~ 3900 6100
Wire Wire Line
	2800 6500 2700 6500
Wire Wire Line
	2700 6500 2700 7000
Wire Wire Line
	2700 7000 4000 7000
Wire Wire Line
	4000 7000 4000 6800
Connection ~ 4000 6800
Wire Wire Line
	4000 6800 4500 6800
Entry Wire Line
	4600 7200 4500 7100
Wire Wire Line
	2800 6400 2600 6400
Wire Wire Line
	2600 6400 2600 7100
Wire Wire Line
	2600 7100 4500 7100
Text Label 4500 7100 2    50   ~ 0
~NRST
Wire Bus Line
	2400 7300 4600 7300
Wire Wire Line
	4400 2900 4400 3300
Wire Wire Line
	4400 3300 5900 3300
Entry Wire Line
	5900 3300 6000 3400
Text Label 5900 3300 2    50   ~ 0
~NRST
Wire Wire Line
	4500 4500 4400 4500
Wire Wire Line
	4400 4500 4400 4900
Wire Wire Line
	4400 4900 5900 4900
Entry Wire Line
	5900 4900 6000 5000
$Comp
L HR2:SN74HCS166;TTSOP16 U11
U 1 1 5F845F8F
P 8300 1900
F 0 "U11" H 8550 2050 60  0000 C CNN
F 1 "SN74HCS166;TTSOP16" H 8900 550 60  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 8500 1950 60  0001 L CNN
F 3 "8-Bit Parallel In Serial Out Shift Register" H 8500 1750 60  0001 L CNN
F 4 "8-Bit Parallel In Serial Out Shift Register" H 8500 1650 60  0001 L CNN "Field5"
	1    8300 1900
	1    0    0    -1  
$EndComp
$Comp
L HR2:SN74HCS166;TTSOP16 U12
U 1 1 5F849624
P 8300 4100
F 0 "U12" H 8550 4250 60  0000 C CNN
F 1 "SN74HCS166;TTSOP16" H 8900 2750 60  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 8500 4150 60  0001 L CNN
F 3 "8-Bit Parallel In Serial Out Shift Register" H 8500 3950 60  0001 L CNN
F 4 "8-Bit Parallel In Serial Out Shift Register" H 8500 3850 60  0001 L CNN "Field5"
	1    8300 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 6600 8100 4900
Wire Wire Line
	8100 4900 8300 4900
Wire Wire Line
	9500 4700 9700 4700
Wire Wire Line
	9700 4700 9700 3450
Wire Wire Line
	9700 3450 8100 3450
Wire Wire Line
	8100 3450 8100 2700
Wire Wire Line
	8100 2700 8300 2700
Wire Wire Line
	9500 2500 9700 2500
Wire Wire Line
	9700 2500 9700 1100
Wire Wire Line
	9700 1100 3800 1100
Text Label 3800 1100 0    50   ~ 0
D12_MISO
Entry Wire Line
	3800 1100 3700 1200
Wire Wire Line
	8300 1900 8000 1900
Wire Wire Line
	8000 1900 8000 2000
Wire Wire Line
	8000 2600 8300 2600
Wire Wire Line
	8300 2500 8000 2500
Connection ~ 8000 2500
Wire Wire Line
	8000 2500 8000 2600
Wire Wire Line
	8300 2400 8000 2400
Connection ~ 8000 2400
Wire Wire Line
	8000 2400 8000 2500
Wire Wire Line
	8300 2300 8000 2300
Connection ~ 8000 2300
Wire Wire Line
	8000 2300 8000 2400
Wire Wire Line
	8300 2200 8000 2200
Connection ~ 8000 2200
Wire Wire Line
	8000 2200 8000 2300
Wire Wire Line
	8300 2100 8000 2100
Connection ~ 8000 2100
Wire Wire Line
	8000 2100 8000 2200
Wire Wire Line
	8300 2000 8000 2000
Connection ~ 8000 2000
Wire Wire Line
	8000 2000 8000 2100
Wire Wire Line
	8300 4100 8000 4100
Wire Wire Line
	8000 4100 8000 4200
Wire Wire Line
	8000 4800 8300 4800
Wire Wire Line
	8300 4700 8000 4700
Connection ~ 8000 4700
Wire Wire Line
	8000 4700 8000 4800
Wire Wire Line
	8300 4600 8000 4600
Connection ~ 8000 4600
Wire Wire Line
	8000 4600 8000 4700
Wire Wire Line
	8300 4500 8000 4500
Connection ~ 8000 4500
Wire Wire Line
	8000 4500 8000 4600
Wire Wire Line
	8300 4400 8000 4400
Connection ~ 8000 4400
Wire Wire Line
	8000 4400 8000 4500
Wire Wire Line
	8300 4300 8000 4300
Connection ~ 8000 4300
Wire Wire Line
	8000 4300 8000 4400
Wire Wire Line
	8300 4200 8000 4200
Connection ~ 8000 4200
Wire Wire Line
	8000 4200 8000 4300
Text Label 7500 5500 0    50   ~ 0
~NRST
Entry Wire Line
	7500 5500 7400 5600
Wire Wire Line
	7500 5500 7900 5500
Connection ~ 6000 5800
Entry Wire Line
	10200 2400 10300 2500
Entry Wire Line
	10200 2600 10300 2700
Entry Wire Line
	10200 4600 10300 4700
Entry Wire Line
	10200 4800 10300 4900
Wire Wire Line
	9500 4600 10200 4600
Wire Wire Line
	9500 4800 9700 4800
Wire Wire Line
	9500 2600 9700 2600
Wire Wire Line
	9500 2400 10200 2400
Text Label 10200 2600 2    50   ~ 0
GND
Text Label 10200 2400 2    50   ~ 0
3.3V
Text Label 10200 4800 2    50   ~ 0
GND
Text Label 10200 4600 2    50   ~ 0
3.3V
Wire Bus Line
	3700 1650 7400 1650
Connection ~ 3700 1650
Wire Wire Line
	7900 3100 8300 3100
Wire Bus Line
	6000 5800 7400 5800
Connection ~ 7400 5800
Wire Bus Line
	7400 5800 10300 5800
Wire Wire Line
	7900 3100 7900 5300
Wire Wire Line
	8300 5300 7900 5300
Connection ~ 7900 5300
Wire Wire Line
	7900 5300 7900 5500
Wire Wire Line
	8000 2600 8000 3650
Connection ~ 8000 2600
Connection ~ 8000 4100
Entry Wire Line
	7500 5200 7400 5100
Entry Wire Line
	7500 5100 7400 5000
Wire Wire Line
	8200 5600 9700 5600
Wire Wire Line
	9700 5600 9700 4800
Connection ~ 9700 4800
Wire Wire Line
	9700 4800 10200 4800
Wire Wire Line
	7500 5200 8300 5200
Wire Wire Line
	7500 5100 8300 5100
Wire Wire Line
	8300 5000 8200 5000
Wire Wire Line
	8200 5000 8200 5600
Wire Wire Line
	8300 2800 8200 2800
Wire Wire Line
	8200 2800 8200 3350
Wire Wire Line
	8200 3350 9700 3350
Wire Wire Line
	9700 3350 9700 2600
Connection ~ 9700 2600
Wire Wire Line
	9700 2600 10200 2600
Wire Wire Line
	3900 6600 8100 6600
Wire Wire Line
	8300 3000 7500 3000
Wire Wire Line
	7500 2900 8300 2900
Entry Wire Line
	7400 2800 7500 2900
Entry Wire Line
	7400 2900 7500 3000
Text Label 7500 2900 0    50   ~ 0
D13_SCK
Text Label 7500 5100 0    50   ~ 0
D13_SCK
Text Label 7500 3000 0    50   ~ 0
D10_NSS
Text Label 7500 5200 0    50   ~ 0
D10_NSS
Text Label 5500 6600 0    50   ~ 0
SER_OUT_TO_IN1
Text Label 8550 3450 0    50   ~ 0
SER_IN1_TO_IN2
Wire Wire Line
	8000 3650 10200 3650
Entry Wire Line
	10200 3650 10300 3750
Text Label 10200 3650 2    50   ~ 0
GND
Text Notes 8050 3600 0    50   ~ 0
Temporarily\nGround Inputs
Text Notes 8500 1650 0    50   ~ 0
8-Bit Parallel to\nSerial Shift Register
Text Notes 8550 3850 0    50   ~ 0
8-Bit Parallel to\nSerial Shift Register
Connection ~ 8000 3650
Wire Wire Line
	8000 3650 8000 4100
Wire Bus Line
	7400 5500 7400 5800
Wire Bus Line
	4600 5800 4600 7300
Wire Bus Line
	2400 7300 2400 7800
Wire Bus Line
	10300 2400 10300 5800
Wire Bus Line
	7400 1650 7400 5200
Wire Bus Line
	3700 700  3700 1650
Wire Bus Line
	6000 2300 6000 5800
Wire Bus Line
	3700 1650 3700 4400
Text Notes 5450 3700 0    50   ~ 0
MikroBus\nClick\nConnector
Text Notes 5450 2100 0    50   ~ 0
MikroBus\nClick\nConnector
Text Notes 5450 5100 0    50   ~ 0
Lidar\nConnector
Text Notes 2400 5950 0    50   ~ 0
8-Bit Parallel\nto Serial\nShift Register
Text Notes 2900 7250 0    50   ~ 0
Power and Reset Bus
Text Notes 6350 5750 0    50   ~ 0
Power and Reset Bus
Text Notes 5200 1600 0    50   ~ 0
Arduino Connector Pins Bus
$EndSCHEMATC
