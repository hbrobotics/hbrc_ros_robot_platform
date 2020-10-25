EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 7 10
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
F 1 "LIDAR_ADAPTER;2xF1x2" H 5100 5150 50  0000 C CNN
F 2 "HR2:LIDAR_ADAPTER_2xF1x2" H 4800 5750 60  0001 L CNN
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
Text HLabel 2000 3600 0    50   UnSpc ~ 0
A0
Text HLabel 2000 5200 0    50   UnSpc ~ 0
A1
Text HLabel 2000 1100 0    50   Input ~ 0
~NRST
Text HLabel 2000 2200 0    50   BiDi ~ 0
A5_SCL
Text HLabel 2000 2100 0    50   BiDi ~ 0
A4_SDA
Text HLabel 2000 2000 0    50   Input ~ 0
D13_SCK
Text HLabel 2000 1900 0    50   BiDi ~ 0
D12_MISO
Text HLabel 2000 1800 0    50   Input ~ 0
D11_PWM_MOSI
Text HLabel 2000 1700 0    50   Input ~ 0
D10_PWM_NSS
Text HLabel 2000 1600 0    50   BiDi ~ 0
D1_TX
Text HLabel 2000 1500 0    50   Input ~ 0
D0_RX
Text Label 3600 1800 2    50   ~ 0
D11_PWM_MOSI
Entry Wire Line
	3700 3000 3800 3100
NoConn ~ 4500 3000
Entry Wire Line
	3700 3100 3800 3200
Entry Wire Line
	3700 3200 3800 3300
Entry Wire Line
	3700 3300 3800 3400
Entry Wire Line
	3700 2400 3800 2500
Entry Wire Line
	3700 2500 3800 2600
Entry Wire Line
	3700 2600 3800 2700
Entry Wire Line
	3800 2800 3700 2700
Entry Wire Line
	3600 2000 3700 2100
Entry Wire Line
	3600 2100 3700 2200
Entry Wire Line
	3600 2200 3700 2300
Entry Wire Line
	2300 1100 2400 1200
Entry Wire Line
	3600 1500 3700 1600
Entry Wire Line
	3600 1600 3700 1700
Entry Wire Line
	3600 1700 3700 1800
Entry Wire Line
	3700 1900 3600 1800
Entry Wire Line
	3600 1900 3700 2000
Text Label 3600 1700 2    50   ~ 0
D10_PWM_NSS
Text Label 3600 1600 2    50   ~ 0
D1_TX
Text Label 3600 1500 2    50   ~ 0
D0_RX
Text Label 3600 1900 2    50   ~ 0
D12_MISO
Text Label 3600 2000 2    50   ~ 0
D13_SCK
Text Label 3600 2100 2    50   ~ 0
A4_SDA
Text Label 3600 2200 2    50   ~ 0
A5_SCL
Text Label 2300 1100 2    50   ~ 0
~NRST
Text Label 3800 3100 0    50   ~ 0
D11_PWM_MOSI
Text Label 3800 2800 0    50   ~ 0
D1_TX
Text Label 3800 2700 0    50   ~ 0
D0_RX
Text Label 3800 3200 0    50   ~ 0
D12_MISO
Text Label 3800 3300 0    50   ~ 0
D13_SCK
Text Label 3800 2500 0    50   ~ 0
A4_SDA
Text Label 3800 2600 0    50   ~ 0
A5_SCL
Wire Wire Line
	2000 1500 3600 1500
Wire Wire Line
	3600 1600 2000 1600
Wire Wire Line
	2000 1700 3600 1700
Wire Wire Line
	3600 1800 2000 1800
Wire Wire Line
	2000 1900 3600 1900
Wire Wire Line
	3600 2000 2000 2000
Wire Wire Line
	2000 2100 3600 2100
Wire Wire Line
	3600 2200 2000 2200
Wire Wire Line
	2000 1100 2300 1100
Wire Wire Line
	4500 2500 3800 2500
Wire Wire Line
	3800 2600 4500 2600
Wire Wire Line
	4500 2700 3800 2700
Wire Wire Line
	3800 2800 4500 2800
Wire Wire Line
	4500 3100 3800 3100
Wire Wire Line
	3800 3200 4500 3200
Wire Wire Line
	4500 3300 3800 3300
Wire Wire Line
	3800 3400 4500 3400
Wire Wire Line
	4500 3500 4400 3500
Wire Wire Line
	4500 2900 2000 2900
Entry Wire Line
	3700 4600 3800 4700
Entry Wire Line
	3700 4700 3800 4800
Entry Wire Line
	3700 4800 3800 4900
Entry Wire Line
	3700 4900 3800 5000
Entry Wire Line
	3700 4000 3800 4100
Entry Wire Line
	3700 4100 3800 4200
Entry Wire Line
	3700 4200 3800 4300
Entry Wire Line
	3800 4400 3700 4300
Text Label 3800 4700 0    50   ~ 0
D11_PWM_MOSI
Text Label 3800 5000 0    50   ~ 0
D10_PWM_NSS
Text Label 3800 4400 0    50   ~ 0
D1_TX
Text Label 3800 4300 0    50   ~ 0
D0_RX
Text Label 3800 4800 0    50   ~ 0
D12_MISO
Text Label 3800 4900 0    50   ~ 0
D13_SCK
Text Label 3800 4100 0    50   ~ 0
A4_SDA
Text Label 3800 4200 0    50   ~ 0
A5_SCL
Text Label 5900 5500 2    50   ~ 0
~NRST
Wire Wire Line
	4500 4100 3800 4100
Wire Wire Line
	3800 4200 4500 4200
Wire Wire Line
	4500 4300 3800 4300
Wire Wire Line
	3800 4400 4500 4400
Wire Wire Line
	4500 4700 3800 4700
Wire Wire Line
	3800 4800 4500 4800
Wire Wire Line
	4500 4900 3800 4900
Wire Wire Line
	3800 5000 4500 5000
Wire Wire Line
	4500 4500 2000 4500
Wire Wire Line
	2000 3600 4500 3600
Wire Wire Line
	2000 5200 4500 5200
Text HLabel 2000 4500 0    50   Output ~ 0
INT1
Text HLabel 2000 2900 0    50   Output ~ 0
INT0
NoConn ~ 4500 4600
NoConn ~ 3900 7300
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
	4400 3500 4400 3900
Wire Wire Line
	4400 3900 5900 3900
Entry Wire Line
	5900 3900 6000 4000
Text Label 5900 3900 2    50   ~ 0
~NRST
Wire Wire Line
	4500 5100 4400 5100
Wire Wire Line
	4400 5100 4400 5500
Wire Wire Line
	4400 5500 5900 5500
Entry Wire Line
	5900 5500 6000 5600
$Comp
L HR2:SN74HCS166;TTSOP16 U6
U 1 1 5F845F8F
P 8300 2500
F 0 "U6" H 8550 2650 60  0000 C CNN
F 1 "SN74HCS166;TTSOP16" H 8900 1150 60  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 8500 2550 60  0001 L CNN
F 3 "8-Bit Parallel In Serial Out Shift Register" H 8500 2350 60  0001 L CNN
F 4 "8-Bit Parallel In Serial Out Shift Register" H 8500 2250 60  0001 L CNN "Field5"
	1    8300 2500
	1    0    0    -1  
$EndComp
$Comp
L HR2:SN74HCS166;TTSOP16 U7
U 1 1 5F849624
P 8300 4700
F 0 "U7" H 8550 4850 60  0000 C CNN
F 1 "SN74HCS166;TTSOP16" H 8900 3350 60  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 8500 4750 60  0001 L CNN
F 3 "8-Bit Parallel In Serial Out Shift Register" H 8500 4550 60  0001 L CNN
F 4 "8-Bit Parallel In Serial Out Shift Register" H 8500 4450 60  0001 L CNN "Field5"
	1    8300 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 5500 8300 5500
Wire Wire Line
	9500 5300 9700 5300
Wire Wire Line
	9700 5300 9700 4050
Wire Wire Line
	9700 4050 8100 4050
Wire Wire Line
	8100 4050 8100 3300
Wire Wire Line
	8100 3300 8300 3300
Wire Wire Line
	9500 3100 9700 3100
Wire Wire Line
	9700 3100 9700 1700
Wire Wire Line
	8300 2500 8000 2500
Wire Wire Line
	8000 2500 8000 2600
Wire Wire Line
	8000 3200 8300 3200
Wire Wire Line
	8300 3100 8000 3100
Connection ~ 8000 3100
Wire Wire Line
	8000 3100 8000 3200
Wire Wire Line
	8300 3000 8000 3000
Connection ~ 8000 3000
Wire Wire Line
	8000 3000 8000 3100
Wire Wire Line
	8300 2900 8000 2900
Connection ~ 8000 2900
Wire Wire Line
	8000 2900 8000 3000
Wire Wire Line
	8300 2800 8000 2800
Connection ~ 8000 2800
Wire Wire Line
	8000 2800 8000 2900
Wire Wire Line
	8300 2700 8000 2700
Connection ~ 8000 2700
Wire Wire Line
	8000 2700 8000 2800
Wire Wire Line
	8300 2600 8000 2600
Connection ~ 8000 2600
Wire Wire Line
	8000 2600 8000 2700
Wire Wire Line
	8300 4700 8000 4700
Wire Wire Line
	8000 4700 8000 4800
Wire Wire Line
	8000 5400 8300 5400
Wire Wire Line
	8300 5300 8000 5300
Connection ~ 8000 5300
Wire Wire Line
	8000 5300 8000 5400
Wire Wire Line
	8300 5200 8000 5200
Connection ~ 8000 5200
Wire Wire Line
	8000 5200 8000 5300
Wire Wire Line
	8300 5100 8000 5100
Connection ~ 8000 5100
Wire Wire Line
	8000 5100 8000 5200
Wire Wire Line
	8300 5000 8000 5000
Connection ~ 8000 5000
Wire Wire Line
	8000 5000 8000 5100
Wire Wire Line
	8300 4900 8000 4900
Connection ~ 8000 4900
Wire Wire Line
	8000 4900 8000 5000
Wire Wire Line
	8300 4800 8000 4800
Connection ~ 8000 4800
Wire Wire Line
	8000 4800 8000 4900
Text Label 7500 6100 0    50   ~ 0
~NRST
Entry Wire Line
	7500 6100 7400 6200
Wire Wire Line
	7500 6100 7900 6100
Connection ~ 6000 6400
Entry Wire Line
	10200 3000 10300 3100
Entry Wire Line
	10200 3200 10300 3300
Entry Wire Line
	10200 5200 10300 5300
Entry Wire Line
	10200 5400 10300 5500
Wire Wire Line
	9500 5200 10200 5200
Wire Wire Line
	9500 5400 9700 5400
Wire Wire Line
	9500 3200 9700 3200
Wire Wire Line
	9500 3000 10200 3000
Text Label 10200 3200 2    50   ~ 0
GND
Text Label 10200 3000 2    50   ~ 0
3.3V
Text Label 10200 5400 2    50   ~ 0
GND
Text Label 10200 5200 2    50   ~ 0
3.3V
Wire Wire Line
	7900 3700 8300 3700
Wire Bus Line
	6000 6400 7400 6400
Connection ~ 7400 6400
Wire Bus Line
	7400 6400 10300 6400
Wire Wire Line
	7900 3700 7900 5900
Wire Wire Line
	8300 5900 7900 5900
Connection ~ 7900 5900
Wire Wire Line
	7900 5900 7900 6100
Wire Wire Line
	8000 3200 8000 4250
Connection ~ 8000 3200
Connection ~ 8000 4700
Entry Wire Line
	7350 5800 7250 5900
Entry Wire Line
	7350 5700 7250 5800
Wire Wire Line
	8200 6200 9700 6200
Wire Wire Line
	9700 6200 9700 5400
Connection ~ 9700 5400
Wire Wire Line
	9700 5400 10200 5400
Wire Wire Line
	7350 5800 8300 5800
Wire Wire Line
	7350 5700 8300 5700
Wire Wire Line
	8300 5600 8200 5600
Wire Wire Line
	8200 5600 8200 6200
Wire Wire Line
	8300 3400 8200 3400
Wire Wire Line
	8200 3400 8200 3950
Wire Wire Line
	8200 3950 9700 3950
Wire Wire Line
	9700 3950 9700 3200
Connection ~ 9700 3200
Wire Wire Line
	9700 3200 10200 3200
Wire Wire Line
	3900 7200 4600 7200
Wire Wire Line
	8300 3600 7350 3600
Wire Wire Line
	7350 3500 8300 3500
Entry Wire Line
	7250 3600 7350 3500
Entry Wire Line
	7250 3700 7350 3600
Text Label 7350 3500 0    50   ~ 0
DIO_SCK
Text Label 7350 5700 0    50   ~ 0
DIO_SCK
Text Label 7350 3600 0    50   ~ 0
~DIO_NSS
Text Label 7350 5800 0    50   ~ 0
~DIO_NSS
Text Label 3900 7200 0    50   ~ 0
SER_OUT_TO_IN1
Text Label 8550 4050 0    50   ~ 0
SER_IN1_TO_IN2
Wire Wire Line
	8000 4250 10200 4250
Entry Wire Line
	10200 4250 10300 4350
Text Label 10200 4250 2    50   ~ 0
GND
Text Notes 8050 4200 0    50   ~ 0
Temporarily\nGround Inputs
Text Notes 8500 2250 0    50   ~ 0
8-Bit Parallel to\nSerial Shift Register
Text Notes 8550 4450 0    50   ~ 0
8-Bit Parallel to\nSerial Shift Register
Connection ~ 8000 4250
Wire Wire Line
	8000 4250 8000 4700
Text Notes 5450 4300 0    50   ~ 0
MikroBus\nClick\nConnector
Text Notes 5450 2700 0    50   ~ 0
MikroBus\nClick\nConnector
Text Notes 5450 5700 0    50   ~ 0
Lidar\nConnector
Text Notes 2400 6550 0    50   ~ 0
8-Bit Parallel\nto Serial\nShift Register
Text Label 3800 3400 0    50   ~ 0
D10_PWM_NSS
Text Label 1500 7450 0    50   ~ 0
DIO_SCK
Text Notes 2550 3450 0    50   ~ 0
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
	4850 7900 4850 6600
Wire Bus Line
	2150 7900 4850 7900
Text Label 2250 6800 0    50   ~ 0
DIO_SCK
Text Label 2250 6900 0    50   ~ 0
~DIO_NSS
Wire Wire Line
	4600 7200 4600 6500
Wire Wire Line
	8100 6500 8100 5500
Wire Bus Line
	4850 6600 7250 6600
Wire Wire Line
	9700 1700 7350 1700
Entry Wire Line
	7350 1700 7250 1800
Text HLabel 1500 7350 0    50   Input ~ 0
DIO_MISO
Text Label 1500 7350 0    50   ~ 0
DIO_MISO
Wire Wire Line
	2050 7350 1500 7350
Entry Wire Line
	2050 7350 2150 7450
Text Label 7350 1700 0    50   ~ 0
DIO_MISO
Text Notes 5700 1250 0    50   ~ 0
Power/Ground/Reset Bus
Text Notes 6150 6350 0    50   ~ 0
Power/Ground/Reset Bus
Text Notes 2900 7850 0    50   ~ 0
Digitial I/O Bus
Text Notes 5350 6750 0    50   ~ 0
Digitial I/O Bus
Text Notes 6550 4150 0    50   ~ 0
Digitial I/O Bus
Text Notes 1750 2750 0    50   ~ 0
Note\n1. D0_TX transmits from Nucleo to Arduiono pins.\n2. D1_RX recevies form Arduino pins t Nucleo.
Text Notes 1750 4350 0    50   ~ 0
Note\n1. D0_TX transmits from Nucleo to Arduiono pins.\n2. D1_RX recevies form Arduino pins t Nucleo.
Text Label 4400 1400 0    50   ~ 0
3.3V
Text Label 6300 1400 0    50   ~ 0
GND
$Comp
L Device:C C?
U 1 1 5F97D822
P 4600 1750
AR Path="/5F48CAAD/5F97D822" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D822" Ref="C14"  Part="1" 
F 0 "C14" H 4400 1850 50  0000 L CNN
F 1 "10pF;1608" H 4650 1650 50  0000 L CNN
F 2 "" H 4638 1600 50  0001 C CNN
F 3 "~" H 4600 1750 50  0001 C CNN
	1    4600 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1600 4600 1500
Wire Wire Line
	4600 1500 4400 1500
Wire Wire Line
	4600 1900 4600 2000
Entry Wire Line
	4400 1400 4300 1300
Entry Wire Line
	6300 1400 6200 1300
$Comp
L Device:C C?
U 1 1 5F97D82E
P 5200 1750
AR Path="/5F48CAAD/5F97D82E" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D82E" Ref="C15"  Part="1" 
F 0 "C15" H 5000 1850 50  0000 L CNN
F 1 "10pF;1608" H 5250 1650 50  0000 L CNN
F 2 "" H 5238 1600 50  0001 C CNN
F 3 "~" H 5200 1750 50  0001 C CNN
	1    5200 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5F97D834
P 5800 1750
AR Path="/5F48CAAD/5F97D834" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D834" Ref="C16"  Part="1" 
F 0 "C16" H 5600 1850 50  0000 L CNN
F 1 "10pF;1608" H 5850 1650 50  0000 L CNN
F 2 "" H 5838 1600 50  0001 C CNN
F 3 "~" H 5800 1750 50  0001 C CNN
	1    5800 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1500 5200 1500
Connection ~ 4600 1500
Wire Wire Line
	5200 1600 5200 1500
Connection ~ 5200 1500
Wire Wire Line
	5200 1500 5800 1500
Wire Wire Line
	5800 1600 5800 1500
Wire Wire Line
	4600 2000 5200 2000
Wire Wire Line
	5800 1900 5800 2000
Wire Wire Line
	5200 1900 5200 2000
Connection ~ 5200 2000
Wire Wire Line
	5200 2000 5800 2000
Wire Wire Line
	4400 1500 4400 1400
Wire Wire Line
	5800 2000 6300 2000
Wire Wire Line
	6300 2000 6300 1400
Connection ~ 5800 2000
Text Notes 4900 1450 0    50   ~ 0
ByPass Capacitors
Text Notes 7200 1550 0    50   ~ 0
Note: There are many extra input pins.  Most of these will be assigned\nto Grove connectors and edge detectors.
Wire Wire Line
	4600 6500 8100 6500
Wire Bus Line
	7400 6100 7400 6400
Wire Bus Line
	2400 1300 10300 1300
Wire Bus Line
	4700 6400 4700 7700
Wire Bus Line
	2400 800  2400 1300
Wire Bus Line
	10300 1300 10300 6400
Wire Bus Line
	7250 1700 7250 6600
Wire Bus Line
	2150 6800 2150 7900
Wire Bus Line
	6000 2900 6000 6400
Wire Bus Line
	3700 1500 3700 5000
$EndSCHEMATC
