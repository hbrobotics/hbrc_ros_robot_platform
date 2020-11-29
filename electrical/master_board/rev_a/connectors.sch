EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 10 10
Title "HR2 Connectors"
Date "2020-11-22"
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
P 4500 4100
AR Path="/5F4CFB97" Ref="CN?"  Part="1" 
AR Path="/5F4CEE66/5F4CFB97" Ref="CN88"  Part="1" 
F 0 "CN88" H 4800 4250 50  0000 C CNN
F 1 "MIKROBUS_MATE;SMALL" H 5050 2850 50  0000 C CNN
F 2 "HR2:MIKROBUS_SMALL_MATE" H 4700 4150 60  0001 L CNN
F 3 "MikroBus Small Mate" H 4700 3950 60  0001 L CNN
F 4 "MikroBus Small Mate" H 4700 3850 60  0001 L CNN "Field5"
	1    4500 4100
	1    0    0    -1  
$EndComp
$Comp
L HR2:MIKROBUS_MATE;MEDIUM CN?
U 1 1 5F4CFB9E
P 4500 2500
AR Path="/5F4CFB9E" Ref="CN?"  Part="1" 
AR Path="/5F4CEE66/5F4CFB9E" Ref="CN89"  Part="1" 
F 0 "CN89" H 4800 2650 50  0000 C CNN
F 1 "MIKROBUS_MATE;MEDIUM" H 5050 1250 50  0000 C CNN
F 2 "HR2:MIKROBUS_MEDIUM_MATE" H 4700 2550 60  0001 L CNN
F 3 "MikroBus Mate Medium" H 4700 2350 60  0001 L CNN
F 4 "MikroBus Mate Medium" H 4700 2250 60  0001 L CNN "Field5"
	1    4500 2500
	1    0    0    -1  
$EndComp
Text HLabel 2000 800  0    50   Input ~ 0
5V
Text HLabel 2000 900  0    50   Input ~ 0
3.3V
Text HLabel 2000 1000 0    50   Input ~ 0
GND
Text Label 2300 800  2    50   ~ 0
5V
Text Label 2300 900  2    50   ~ 0
3.3V
Text Label 2300 1000 2    50   ~ 0
GND
Wire Wire Line
	2300 800  2000 800 
Wire Wire Line
	2300 900  2000 900 
Wire Wire Line
	2300 1000 2000 1000
Entry Wire Line
	2400 1100 2300 1000
Entry Wire Line
	2400 1000 2300 900 
Entry Wire Line
	2400 900  2300 800 
Text Label 5900 4500 2    50   ~ 0
5V
Text Label 5900 4800 2    50   ~ 0
GND
Entry Wire Line
	6000 4900 5900 4800
Wire Wire Line
	5900 4500 5600 4500
Wire Wire Line
	5600 4800 5900 4800
Text Label 5900 2900 2    50   ~ 0
5V
Text Label 5900 4700 2    50   ~ 0
GND
Entry Wire Line
	6000 4800 5900 4700
Entry Wire Line
	5900 2900 6000 3000
Wire Wire Line
	5900 4700 5600 4700
Wire Wire Line
	5900 2900 5600 2900
Text HLabel 2000 6100 0    50   Input ~ 0
LDR_PWM
Text HLabel 1500 6700 0    50   Input ~ 0
SER_IN
Text HLabel 1500 7450 0    50   Input ~ 0
DIO_SCK
Text HLabel 1500 7550 0    50   Input ~ 0
~DIO_NSS
$Comp
L HR2:SN74HCS595;TTSOP16 U5
U 1 1 5FD710B8
P 2800 6400
F 0 "U5" H 3050 6550 50  0000 C CNN
F 1 "SN74HCS595;TTSOP16" H 3300 5250 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 3000 6450 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 3000 6250 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 3000 6150 60  0001 L CNN "Field5"
	1    2800 6400
	1    0    0    -1  
$EndComp
Entry Wire Line
	4700 6500 4600 6400
Entry Wire Line
	4700 7300 4600 7400
Text Label 4500 6400 2    50   ~ 0
3.3V
Wire Wire Line
	2000 6100 4600 6100
Text Label 5900 5900 2    50   ~ 0
GND
Entry Wire Line
	5900 5900 6000 6000
Entry Wire Line
	5900 6000 6000 6100
Wire Wire Line
	2000 5800 4600 5800
Wire Wire Line
	4600 5700 2000 5700
Text Label 2000 6100 0    50   ~ 0
LDR_PWM
Text Label 4200 6000 0    50   ~ 0
LDR_EN
Text Label 4200 5900 0    50   ~ 0
LDR_SPIN
Text Label 2000 5800 0    50   ~ 0
LDR_RX
Text Label 2000 5700 0    50   ~ 0
LDR_TX
Text Label 5900 6000 2    50   ~ 0
5V
Text HLabel 2000 5800 0    50   Input ~ 0
LDR_RX
Text HLabel 2000 5700 0    50   Output ~ 0
LDR_TX
$Comp
L HR2:LIDAR_ADAPTER;2xF1x2 CN?
U 1 1 5F84DF13
P 4600 5700
AR Path="/5F4A826A/5F84DF13" Ref="CN?"  Part="1" 
AR Path="/5F4CEE66/5F84DF13" Ref="CN5"  Part="1" 
F 0 "CN5" H 4850 5850 50  0000 C CNN
F 1 "LIDAR_ADAPTER;2xF1x3+F1x2" H 5100 5150 50  0000 C CNN
F 2 "HR2:LIDAR_ADAPTER_2xF1x3+F1x2" H 4800 5750 60  0001 L CNN
F 3 "Lidar Adapter Connectors" H 4800 5550 60  0001 L CNN
F 4 "Lidar Adapter Connectors" H 4800 5650 60  0001 L CNN "manf#"
	1    4600 5700
	1    0    0    -1  
$EndComp
Text Label 5900 5800 2    50   ~ 0
3.3V
Entry Wire Line
	6000 5900 5900 5800
Wire Wire Line
	5600 5800 5900 5800
Wire Wire Line
	5600 5900 5900 5900
Wire Wire Line
	5600 6000 5900 6000
Text Label 1500 6700 0    50   ~ 0
SER_IN
Text Label 1500 7550 0    50   ~ 0
~DIO_NSS
Wire Wire Line
	3900 6500 4100 6500
Wire Wire Line
	4100 6500 4100 5900
Wire Wire Line
	4100 5900 4600 5900
Wire Wire Line
	3900 6600 4200 6600
Wire Wire Line
	4200 6600 4200 6000
Wire Wire Line
	4200 6000 4600 6000
Wire Wire Line
	5600 3100 5900 3100
Entry Wire Line
	5900 3100 6000 3200
Text Label 5900 3100 2    50   ~ 0
GND
Wire Wire Line
	5600 3200 5900 3200
Entry Wire Line
	5900 3200 6000 3300
Text Label 5900 3200 2    50   ~ 0
GND
Wire Wire Line
	5900 4600 5600 4600
Text Label 5900 4600 2    50   ~ 0
3.3V
Entry Wire Line
	5900 4500 6000 4600
Entry Wire Line
	6000 4700 5900 4600
Text Label 5900 3000 2    50   ~ 0
3.3V
Entry Wire Line
	5900 3000 6000 3100
Wire Wire Line
	5600 3000 5900 3000
Wire Bus Line
	4700 6400 6000 6400
Wire Wire Line
	3900 7400 4000 7400
Wire Wire Line
	3900 6400 4600 6400
Text HLabel 1700 3600 0    50   UnSpc ~ 0
A0
Text HLabel 1700 5200 0    50   UnSpc ~ 0
A1
Text HLabel 2000 1100 0    50   Input ~ 0
~NRST
Text HLabel 1700 2200 0    50   BiDi ~ 0
A5_SCL
Text HLabel 1700 2100 0    50   BiDi ~ 0
A4_SDA
Text HLabel 1700 2000 0    50   Input ~ 0
D13_SCK
Text HLabel 1700 1900 0    50   BiDi ~ 0
D12_MISO
Text HLabel 1700 1800 0    50   Input ~ 0
D11_PWM_MOSI
Text HLabel 1700 1700 0    50   Input ~ 0
D10_PWM_NSS
Text HLabel 1700 1600 0    50   BiDi ~ 0
D1_TX
Text HLabel 1700 1500 0    50   Input ~ 0
D0_RX
Text Label 3300 1800 2    50   ~ 0
D11_PWM_MOSI
Entry Wire Line
	3400 3000 3500 3100
NoConn ~ 4500 3000
Entry Wire Line
	3400 3100 3500 3200
Entry Wire Line
	3400 3200 3500 3300
Entry Wire Line
	3400 3300 3500 3400
Entry Wire Line
	3400 2400 3500 2500
Entry Wire Line
	3400 2500 3500 2600
Entry Wire Line
	3400 2600 3500 2700
Entry Wire Line
	3500 2800 3400 2700
Entry Wire Line
	3300 2000 3400 2100
Entry Wire Line
	3300 2100 3400 2200
Entry Wire Line
	3300 2200 3400 2300
Entry Wire Line
	2300 1100 2400 1200
Entry Wire Line
	3300 1500 3400 1600
Entry Wire Line
	3300 1600 3400 1700
Entry Wire Line
	3300 1700 3400 1800
Entry Wire Line
	3400 1900 3300 1800
Entry Wire Line
	3300 1900 3400 2000
Text Label 3300 1700 2    50   ~ 0
D10_PWM_NSS
Text Label 3300 1600 2    50   ~ 0
D1_TX
Text Label 3300 1500 2    50   ~ 0
D0_RX
Text Label 3300 1900 2    50   ~ 0
D12_MISO
Text Label 3300 2000 2    50   ~ 0
D13_SCK
Text Label 3300 2100 2    50   ~ 0
A4_SDA
Text Label 3300 2200 2    50   ~ 0
A5_SCL
Text Label 2300 1100 2    50   ~ 0
~NRST
Text Label 3500 3100 0    50   ~ 0
D11_PWM_MOSI
Text Label 3500 2800 0    50   ~ 0
D1_TX
Text Label 3500 2700 0    50   ~ 0
D0_RX
Text Label 3500 3200 0    50   ~ 0
D12_MISO
Text Label 3500 3300 0    50   ~ 0
D13_SCK
Text Label 3500 2500 0    50   ~ 0
A4_SDA
Text Label 3500 2600 0    50   ~ 0
A5_SCL
Wire Wire Line
	1700 1500 3300 1500
Wire Wire Line
	3300 1600 1700 1600
Wire Wire Line
	1700 1700 3300 1700
Wire Wire Line
	3300 1800 1700 1800
Wire Wire Line
	1700 1900 3300 1900
Wire Wire Line
	3300 2000 1700 2000
Wire Wire Line
	1700 2100 3300 2100
Wire Wire Line
	3300 2200 1700 2200
Wire Wire Line
	2000 1100 2300 1100
Wire Wire Line
	4200 2500 3500 2500
Wire Wire Line
	3500 2600 4200 2600
Wire Wire Line
	4200 2700 3500 2700
Wire Wire Line
	3500 2800 4200 2800
Wire Wire Line
	4200 3100 3500 3100
Wire Wire Line
	3500 3200 4200 3200
Wire Wire Line
	4200 3300 3500 3300
Wire Wire Line
	3500 3400 4200 3400
Wire Wire Line
	4500 3500 4400 3500
Wire Wire Line
	4200 2900 1700 2900
Entry Wire Line
	3400 4600 3500 4700
Entry Wire Line
	3400 4700 3500 4800
Entry Wire Line
	3400 4800 3500 4900
Entry Wire Line
	3400 4900 3500 5000
Entry Wire Line
	3400 4000 3500 4100
Entry Wire Line
	3400 4100 3500 4200
Entry Wire Line
	3400 4200 3500 4300
Entry Wire Line
	3500 4400 3400 4300
Text Label 3500 4700 0    50   ~ 0
D11_PWM_MOSI
Text Label 3500 5000 0    50   ~ 0
D10_PWM_NSS
Text Label 3500 4400 0    50   ~ 0
D1_TX
Text Label 3500 4300 0    50   ~ 0
D0_RX
Text Label 3500 4800 0    50   ~ 0
D12_MISO
Text Label 3500 4900 0    50   ~ 0
D13_SCK
Text Label 3500 4100 0    50   ~ 0
A4_SDA
Text Label 3500 4200 0    50   ~ 0
A5_SCL
Text Label 5900 5500 2    50   ~ 0
~NRST
Wire Wire Line
	4200 4100 3500 4100
Wire Wire Line
	3500 4200 4200 4200
Wire Wire Line
	4200 4300 3500 4300
Wire Wire Line
	3500 4400 4200 4400
Wire Wire Line
	4200 4700 3500 4700
Wire Wire Line
	3500 4800 4200 4800
Wire Wire Line
	4200 4900 3500 4900
Wire Wire Line
	3500 5000 4200 5000
Wire Wire Line
	4200 4500 1700 4500
Wire Wire Line
	1700 3600 4200 3600
Wire Wire Line
	1700 5200 4200 5200
Text HLabel 1700 4500 0    50   Output ~ 0
INT1
Text HLabel 1700 2900 0    50   Output ~ 0
INT0
NoConn ~ 4500 4600
NoConn ~ 3900 7100
NoConn ~ 3900 7000
NoConn ~ 3900 6900
NoConn ~ 3900 6800
NoConn ~ 3900 6700
Wire Wire Line
	2800 7100 2700 7100
Wire Wire Line
	2700 7100 2700 7600
Wire Wire Line
	2700 7600 4000 7600
Wire Wire Line
	4000 7600 4000 7400
Connection ~ 4000 7400
Wire Wire Line
	4000 7400 4600 7400
Entry Wire Line
	4700 7600 4600 7700
Wire Wire Line
	2800 7000 2600 7000
Wire Wire Line
	2600 7000 2600 7700
Wire Wire Line
	2600 7700 4600 7700
Text Label 4500 7700 2    50   ~ 0
~NRST
Wire Wire Line
	4500 5100 4400 5100
Entry Wire Line
	5900 5500 6000 5600
Wire Wire Line
	9900 5200 10100 5200
Wire Wire Line
	10100 5200 10100 4050
Wire Wire Line
	10100 4050 8700 4050
Wire Wire Line
	8700 4050 8700 3300
Wire Wire Line
	8700 3300 8900 3300
Wire Wire Line
	9900 3000 10100 3000
Wire Wire Line
	10100 3000 10100 1700
Wire Wire Line
	8900 4700 8500 4700
Wire Wire Line
	8500 4700 8500 4800
Wire Wire Line
	8500 5400 8900 5400
Wire Wire Line
	8900 5300 8500 5300
Connection ~ 8500 5300
Wire Wire Line
	8500 5300 8500 5400
Wire Wire Line
	8900 5200 8500 5200
Connection ~ 8500 5200
Wire Wire Line
	8500 5200 8500 5300
Wire Wire Line
	8900 5100 8500 5100
Connection ~ 8500 5100
Wire Wire Line
	8500 5100 8500 5200
Wire Wire Line
	8900 5000 8500 5000
Connection ~ 8500 5000
Wire Wire Line
	8500 5000 8500 5100
Wire Wire Line
	8900 4900 8500 4900
Connection ~ 8500 4900
Wire Wire Line
	8500 4900 8500 5000
Wire Wire Line
	8900 4800 8500 4800
Connection ~ 8500 4800
Wire Wire Line
	8500 4800 8500 4900
Entry Wire Line
	10300 2900 10400 3000
Entry Wire Line
	10300 3200 10400 3300
Entry Wire Line
	10300 5100 10400 5200
Entry Wire Line
	10300 5400 10400 5500
Wire Wire Line
	9900 5100 10300 5100
Wire Wire Line
	9900 5400 10100 5400
Wire Wire Line
	9900 2900 10300 2900
Text Label 10300 3200 2    50   ~ 0
GND
Text Label 10300 2900 2    50   ~ 0
3.3V
Text Label 10300 5400 2    50   ~ 0
GND
Text Label 10300 5100 2    50   ~ 0
3.3V
Connection ~ 8500 4700
Entry Wire Line
	8100 5700 8000 5800
Entry Wire Line
	8100 5600 8000 5700
Wire Wire Line
	8800 6100 10100 6100
Wire Wire Line
	10100 6100 10100 5400
Connection ~ 10100 5400
Wire Wire Line
	10100 5400 10300 5400
Wire Wire Line
	8100 5700 8900 5700
Wire Wire Line
	8900 3600 8800 3600
Wire Wire Line
	8800 3600 8800 3950
Wire Wire Line
	8800 3950 10100 3950
Wire Wire Line
	10100 3950 10100 3200
Connection ~ 10100 3200
Wire Wire Line
	10100 3200 10300 3200
Wire Wire Line
	8900 3400 8100 3400
Wire Wire Line
	8100 3500 8900 3500
Entry Wire Line
	8000 3600 8100 3500
Entry Wire Line
	8000 3500 8100 3400
Text Label 8100 3500 0    50   ~ 0
DIO_SCK
Text Label 8100 5600 0    50   ~ 0
DIO_SCK
Text Label 8100 3400 0    50   ~ 0
DIO_SS
Text Label 8100 5700 0    50   ~ 0
DIO_SS
Text Label 9050 4050 0    50   ~ 0
DIO_IN_BRIDGE
Wire Wire Line
	8500 4250 10300 4250
Entry Wire Line
	10300 4250 10400 4350
Text Label 10300 4250 2    50   ~ 0
GND
Text Notes 8600 2250 0    50   ~ 0
8-Bit Parallel to\nSerial Shift Register
Text Notes 8550 4450 0    50   ~ 0
8-Bit Parallel to\nSerial Shift Register
Wire Wire Line
	8500 4250 8500 4700
Text Notes 5450 4300 0    50   ~ 0
MikroBus\nClick\nConnector
Text Notes 5450 2700 0    50   ~ 0
MikroBus\nClick\nConnector
Text Notes 5450 5700 0    50   ~ 0
Lidar\nConnector
Text Notes 2400 6550 0    50   ~ 0
8-Bit Parallel\nto Serial\nShift Register
Text Label 3500 3400 0    50   ~ 0
D10_PWM_NSS
Text Label 1500 7450 0    50   ~ 0
DIO_SCK
Text Notes 2250 3450 0    50   ~ 0
Arduino Connector Pins Bus
Wire Wire Line
	1500 6700 2800 6700
Wire Wire Line
	2250 6800 2800 6800
Wire Wire Line
	2250 6900 2800 6900
Text Label 4500 7400 2    50   ~ 0
GND
Wire Wire Line
	1500 7550 2050 7550
Wire Wire Line
	1500 7450 2050 7450
Entry Wire Line
	2050 7550 2150 7650
Entry Wire Line
	2050 7450 2150 7550
Entry Wire Line
	2250 6800 2150 6900
Entry Wire Line
	2250 6900 2150 7000
Wire Bus Line
	4850 7900 4850 6700
Wire Bus Line
	2150 7900 4850 7900
Text Label 2250 6800 0    50   ~ 0
DIO_SCK
Text Label 2250 6900 0    50   ~ 0
~DIO_NSS
Wire Wire Line
	10100 1700 8100 1700
Entry Wire Line
	8100 1700 8000 1800
Text HLabel 1500 7350 0    50   Input ~ 0
DIO_MISO
Text Label 1500 7350 0    50   ~ 0
DIO_MISO
Wire Wire Line
	2050 7350 1500 7350
Entry Wire Line
	2050 7350 2150 7450
Text Label 8100 1700 0    50   ~ 0
DIO_MISO
Text Notes 6100 1200 0    50   ~ 0
Power/Ground/Reset Bus
Text Notes 6150 6350 0    50   ~ 0
Power/Ground/Reset Bus
Text Notes 2900 7850 0    50   ~ 0
Digitial I/O Bus
Text Notes 5350 6800 0    50   ~ 0
Digitial I/O Bus
Text Notes 8050 1950 0    50   ~ 0
Digitial I/O Bus
Text Notes 1300 2750 0    50   ~ 0
Note\n1. D0_TX transmits from Nucleo to Arduiono pins.\n2. D1_RX recevies form Arduino pins t Nucleo.
Text Notes 1300 4350 0    50   ~ 0
Note\n1. D0_TX transmits from Nucleo to Arduiono pins.\n2. D1_RX recevies form Arduino pins t Nucleo.
Text Label 3900 1400 0    50   ~ 0
3.3V
Text Label 5800 1400 0    50   ~ 0
GND
$Comp
L Device:C C?
U 1 1 5F97D822
P 4100 1750
AR Path="/5F48CAAD/5F97D822" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D822" Ref="C14"  Part="1" 
F 0 "C14" H 3900 1850 50  0000 L CNN
F 1 "10pF;1608" H 4150 1650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4138 1600 50  0001 C CNN
F 3 "~" H 4100 1750 50  0001 C CNN
	1    4100 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1600 4100 1500
Wire Wire Line
	4100 1500 3900 1500
Wire Wire Line
	4100 1900 4100 2000
Entry Wire Line
	5800 1350 5700 1250
$Comp
L Device:C C?
U 1 1 5F97D82E
P 4700 1750
AR Path="/5F48CAAD/5F97D82E" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D82E" Ref="C15"  Part="1" 
F 0 "C15" H 4500 1850 50  0000 L CNN
F 1 "10pF;1608" H 4750 1650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4738 1600 50  0001 C CNN
F 3 "~" H 4700 1750 50  0001 C CNN
	1    4700 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5F97D834
P 5300 1750
AR Path="/5F48CAAD/5F97D834" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D834" Ref="C16"  Part="1" 
F 0 "C16" H 5100 1850 50  0000 L CNN
F 1 "10pF;1608" H 5350 1650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5338 1600 50  0001 C CNN
F 3 "~" H 5300 1750 50  0001 C CNN
	1    5300 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1500 4700 1500
Connection ~ 4100 1500
Wire Wire Line
	4700 1600 4700 1500
Connection ~ 4700 1500
Wire Wire Line
	4700 1500 5300 1500
Wire Wire Line
	5300 1600 5300 1500
Wire Wire Line
	4100 2000 4700 2000
Wire Wire Line
	5300 1900 5300 2000
Wire Wire Line
	4700 1900 4700 2000
Connection ~ 4700 2000
Wire Wire Line
	4700 2000 5300 2000
Wire Wire Line
	3900 1500 3900 1350
Wire Wire Line
	5300 2000 5800 2000
Wire Wire Line
	5800 2000 5800 1350
Connection ~ 5300 2000
Text Notes 4400 1450 0    50   ~ 0
ByPass Capacitors
Text Notes 7300 1550 0    50   ~ 0
Note: The 8 extra input pins on U& that are currently unused.
NoConn ~ 4500 2500
NoConn ~ 4500 2600
NoConn ~ 4500 2700
NoConn ~ 4500 2800
NoConn ~ 4500 2900
NoConn ~ 4500 3100
NoConn ~ 4500 3200
NoConn ~ 4500 3300
NoConn ~ 4500 3400
NoConn ~ 4500 3600
NoConn ~ 4200 2700
NoConn ~ 4200 2800
NoConn ~ 4200 2900
NoConn ~ 4200 3100
NoConn ~ 4200 3200
NoConn ~ 4200 3300
NoConn ~ 4200 3400
NoConn ~ 4200 3600
NoConn ~ 4200 5200
NoConn ~ 4500 5200
NoConn ~ 4500 5000
NoConn ~ 4500 4900
NoConn ~ 4500 4800
NoConn ~ 4500 4700
NoConn ~ 4500 4500
NoConn ~ 4500 4400
NoConn ~ 4500 4300
NoConn ~ 4500 4200
NoConn ~ 4500 4100
NoConn ~ 4200 4300
NoConn ~ 4200 4400
NoConn ~ 4200 4500
NoConn ~ 4200 4700
NoConn ~ 4200 4800
NoConn ~ 4200 4900
NoConn ~ 4200 5000
Wire Wire Line
	4400 5500 5900 5500
Entry Wire Line
	5900 3900 6000 4000
Text Label 5900 3900 2    50   ~ 0
~NRST
Wire Wire Line
	4400 3900 5900 3900
Wire Wire Line
	4400 3900 4400 3500
Wire Wire Line
	4400 5500 4400 5100
Entry Wire Line
	3900 1350 3800 1250
$Comp
L HR2:SENSE;M1x3 CN4
U 1 1 5FBBDB3C
P 6300 1900
F 0 "CN4" H 6550 2050 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 1650 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 1950 60  0001 L CNN
F 3 "Sense M1X3" H 6500 1750 60  0001 L CNN
F 4 "Sense F1X3" H 6500 1650 60  0001 L CNN "Field5"
	1    6300 1900
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN11
U 1 1 5FBBE822
P 6300 2400
F 0 "CN11" H 6550 2550 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 2150 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 2450 60  0001 L CNN
F 3 "Sense M1X3" H 6500 2250 60  0001 L CNN
F 4 "Sense F1X3" H 6500 2150 60  0001 L CNN "Field5"
	1    6300 2400
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN12
U 1 1 5FBBECC7
P 6300 2900
F 0 "CN12" H 6550 3050 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 2650 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 2950 60  0001 L CNN
F 3 "Sense M1X3" H 6500 2750 60  0001 L CNN
F 4 "Sense F1X3" H 6500 2650 60  0001 L CNN "Field5"
	1    6300 2900
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN13
U 1 1 5FBBEE32
P 6300 3400
F 0 "CN13" H 6550 3550 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 3150 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 3450 60  0001 L CNN
F 3 "Sense M1X3" H 6500 3250 60  0001 L CNN
F 4 "Sense F1X3" H 6500 3150 60  0001 L CNN "Field5"
	1    6300 3400
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN14
U 1 1 5FBCA45F
P 6300 3900
F 0 "CN14" H 6550 4050 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 3650 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 3950 60  0001 L CNN
F 3 "Sense M1X3" H 6500 3750 60  0001 L CNN
F 4 "Sense F1X3" H 6500 3650 60  0001 L CNN "Field5"
	1    6300 3900
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN15
U 1 1 5FBCA5EE
P 6300 4400
F 0 "CN15" H 6550 4550 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 4150 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 4450 60  0001 L CNN
F 3 "Sense M1X3" H 6500 4250 60  0001 L CNN
F 4 "Sense F1X3" H 6500 4150 60  0001 L CNN "Field5"
	1    6300 4400
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN16
U 1 1 5FBCA5F9
P 6300 4900
F 0 "CN16" H 6550 5050 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 4650 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 4950 60  0001 L CNN
F 3 "Sense M1X3" H 6500 4750 60  0001 L CNN
F 4 "Sense F1X3" H 6500 4650 60  0001 L CNN "Field5"
	1    6300 4900
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN17
U 1 1 5FBCA604
P 6300 5400
F 0 "CN17" H 6550 5550 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 5150 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 5450 60  0001 L CNN
F 3 "Sense M1X3" H 6500 5250 60  0001 L CNN
F 4 "Sense F1X3" H 6500 5150 60  0001 L CNN "Field5"
	1    6300 5400
	1    0    0    -1  
$EndComp
Entry Wire Line
	6000 1800 6100 1900
Entry Wire Line
	6000 1900 6100 2000
Text Label 6100 2000 0    50   ~ 0
GND
Text Label 6100 1900 0    50   ~ 0
3.3V
Wire Wire Line
	6300 1900 6100 1900
Wire Wire Line
	6300 2000 6100 2000
Entry Wire Line
	6000 2300 6100 2400
Entry Wire Line
	6000 2400 6100 2500
Text Label 6100 2500 0    50   ~ 0
GND
Text Label 6100 2400 0    50   ~ 0
3.3V
Wire Wire Line
	6300 2400 6100 2400
Wire Wire Line
	6300 2500 6100 2500
Entry Wire Line
	6000 2800 6100 2900
Entry Wire Line
	6000 2900 6100 3000
Text Label 6100 3000 0    50   ~ 0
GND
Text Label 6100 2900 0    50   ~ 0
3.3V
Wire Wire Line
	6300 2900 6100 2900
Wire Wire Line
	6300 3000 6100 3000
Entry Wire Line
	6000 3300 6100 3400
Entry Wire Line
	6000 3400 6100 3500
Text Label 6100 3500 0    50   ~ 0
GND
Text Label 6100 3400 0    50   ~ 0
3.3V
Wire Wire Line
	6300 3400 6100 3400
Wire Wire Line
	6300 3500 6100 3500
Entry Wire Line
	6000 3800 6100 3900
Entry Wire Line
	6000 3900 6100 4000
Text Label 6100 4000 0    50   ~ 0
GND
Text Label 6100 3900 0    50   ~ 0
3.3V
Wire Wire Line
	6300 3900 6100 3900
Wire Wire Line
	6300 4000 6100 4000
Entry Wire Line
	6000 4300 6100 4400
Entry Wire Line
	6000 4400 6100 4500
Text Label 6100 4500 0    50   ~ 0
GND
Text Label 6100 4400 0    50   ~ 0
3.3V
Wire Wire Line
	6300 4400 6100 4400
Wire Wire Line
	6300 4500 6100 4500
Entry Wire Line
	6000 4800 6100 4900
Entry Wire Line
	6000 4900 6100 5000
Text Label 6100 5000 0    50   ~ 0
GND
Text Label 6100 4900 0    50   ~ 0
3.3V
Wire Wire Line
	6300 4900 6100 4900
Wire Wire Line
	6300 5000 6100 5000
Entry Wire Line
	6000 5300 6100 5400
Entry Wire Line
	6000 5400 6100 5500
Text Label 6100 5500 0    50   ~ 0
GND
Text Label 6100 5400 0    50   ~ 0
3.3V
Wire Wire Line
	6300 5400 6100 5400
Wire Wire Line
	6300 5500 6100 5500
Wire Wire Line
	7300 5500 7900 5500
Wire Wire Line
	7900 5500 7900 3200
Wire Wire Line
	7300 5000 7800 5000
Wire Wire Line
	7800 5000 7800 3100
Wire Wire Line
	7300 4500 7700 4500
Wire Wire Line
	7700 4500 7700 3000
Wire Wire Line
	7700 3000 8900 3000
Wire Wire Line
	7300 4000 7600 4000
Wire Wire Line
	7600 4000 7600 2900
Wire Wire Line
	7600 2900 8900 2900
Wire Wire Line
	7300 3500 7500 3500
Wire Wire Line
	7500 3500 7500 2800
Wire Wire Line
	7500 2800 8900 2800
Wire Bus Line
	4850 6700 8000 6700
Connection ~ 6000 6400
Wire Bus Line
	6000 1250 10400 1250
Connection ~ 6000 1250
Wire Wire Line
	7300 3000 7400 3000
Wire Wire Line
	7400 3000 7400 2700
Wire Wire Line
	7400 2700 8900 2700
Wire Wire Line
	7400 2600 7400 2500
Wire Wire Line
	7400 2500 7300 2500
Wire Wire Line
	7400 2600 8900 2600
Wire Wire Line
	7500 2500 7500 2000
Wire Wire Line
	7500 2000 7300 2000
Wire Wire Line
	7500 2500 8900 2500
Text Label 7300 2000 0    50   ~ 0
SIG1
Text Label 7400 2600 0    50   ~ 0
SIG2
Text Label 7400 2700 0    50   ~ 0
SIG3
Text Label 7300 3500 0    50   ~ 0
SIG4
Text Label 7300 4000 0    50   ~ 0
SIG5
Text Label 7300 4500 0    50   ~ 0
SIG6
Text Label 7300 5000 0    50   ~ 0
SIG7
Text Label 7300 5500 0    50   ~ 0
SIG8
NoConn ~ 3900 7200
Text HLabel 1500 7650 0    50   Input ~ 0
DIO_SS
Entry Wire Line
	2050 7650 2150 7750
Wire Wire Line
	1500 7650 2050 7650
Text Label 1500 7650 0    50   ~ 0
DIO_SS
$Comp
L HR2:SN74HCS165;TTSOP16 U7
U 1 1 5FC4FA21
P 8900 4700
F 0 "U7" H 9150 4850 50  0000 C CNN
F 1 "SN74HCS165;TTSOP16" H 9400 3450 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 9100 4750 60  0001 L CNN
F 3 "8-Bit Parallel In Serial Out Shift Register" H 9100 4550 60  0001 L CNN
F 4 "8-Bit Parallel In Serial Out Shift Register" H 9100 4450 60  0001 L CNN "desc"
	1    8900 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 3200 10100 3200
Wire Wire Line
	7900 3200 8900 3200
Wire Wire Line
	7800 3100 8900 3100
NoConn ~ 9900 3100
Wire Bus Line
	6000 6400 10400 6400
Text Label 3900 7300 0    50   ~ 0
DIO_OUT_TO_IN
Wire Wire Line
	3900 7300 4600 7300
Wire Wire Line
	4600 6600 4600 7300
Wire Wire Line
	4600 6600 8600 6600
Wire Wire Line
	8600 5500 8900 5500
Wire Wire Line
	8600 5500 8600 6600
Wire Wire Line
	8100 5600 8900 5600
Wire Wire Line
	8900 5800 8800 5800
Wire Wire Line
	8800 5800 8800 6100
NoConn ~ 9900 5300
Wire Bus Line
	2400 1250 6000 1250
Wire Bus Line
	4700 6400 4700 7700
Wire Bus Line
	2400 800  2400 1250
Wire Bus Line
	10400 1250 10400 6400
Wire Bus Line
	8000 1700 8000 6700
Wire Bus Line
	2150 6800 2150 7900
Wire Bus Line
	3400 1500 3400 5000
Wire Bus Line
	6000 1250 6000 6400
$Comp
L HR2:SN74HCS165;TTSOP16 U6
U 1 1 5FC4EDF6
P 8900 2500
F 0 "U6" H 9150 2650 50  0000 C CNN
F 1 "SN74HCS165;TTSOP16" H 9400 1250 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 9100 2550 60  0001 L CNN
F 3 "8-Bit Parallel In Serial Out Shift Register" H 9100 2350 60  0001 L CNN
F 4 "8-Bit Parallel In Serial Out Shift Register" H 9100 2250 60  0001 L CNN "desc"
	1    8900 2500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
