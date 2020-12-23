EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 9 10
Title "HR2 Connectors"
Date "2020-11-22"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2000 900  0    50   Input ~ 0
5V
Text HLabel 2000 1000 0    50   Input ~ 0
3.3V
Text HLabel 2000 1100 0    50   Input ~ 0
GND
Text Label 2300 900  2    50   ~ 0
5V
Text Label 2300 1000 2    50   ~ 0
3.3V
Text Label 2300 1100 2    50   ~ 0
GND
Wire Wire Line
	2300 900  2000 900 
Wire Wire Line
	2300 1000 2000 1000
Wire Wire Line
	2300 1100 2000 1100
Entry Wire Line
	2400 1000 2300 1100
Entry Wire Line
	2400 900  2300 1000
Entry Wire Line
	2400 800  2300 900 
Text HLabel 1500 5400 0    50   Input ~ 0
LDR_PWM
Text HLabel 1500 6100 0    50   Input ~ 0
SER_IN
Text HLabel 1500 4000 0    50   Input ~ 0
DIO_SCK
Entry Wire Line
	4100 5700 4000 5800
Entry Wire Line
	4100 6900 4000 7000
Text Label 4000 5800 2    50   ~ 0
3.3V
Wire Wire Line
	1500 5400 4200 5400
Text Label 5900 5900 2    50   ~ 0
GND
Entry Wire Line
	5900 5800 6000 5700
Entry Wire Line
	5900 5700 6000 5600
Wire Wire Line
	1500 5300 4300 5300
Wire Wire Line
	4400 5200 1500 5200
Text Label 1550 5400 0    50   ~ 0
LDR_PWM
Text Label 3700 6000 0    50   ~ 0
LDR_EN
Text Label 3700 5900 0    50   ~ 0
LDR_SPIN
Text Label 1550 5300 0    50   ~ 0
LDR_RX
Text Label 1550 5200 0    50   ~ 0
LDR_TX
Text Label 5900 5700 2    50   ~ 0
5V
Text HLabel 1500 5300 0    50   Input ~ 0
LDR_RX
Text HLabel 1500 5200 0    50   Output ~ 0
LDR_TX
Text Label 5900 5800 2    50   ~ 0
3.3V
Entry Wire Line
	6000 5800 5900 5900
Wire Wire Line
	5500 5800 5900 5800
Wire Wire Line
	5500 5900 5900 5900
Wire Wire Line
	5500 5700 5900 5700
Text Label 1500 6100 0    50   ~ 0
SER_IN
Wire Wire Line
	3700 6800 3800 6800
Wire Wire Line
	3700 5800 4000 5800
Text HLabel 2000 1200 0    50   Input ~ 0
~NRST
Entry Wire Line
	2300 1200 2400 1100
Text Label 2300 1200 2    50   ~ 0
~NRST
Wire Wire Line
	2000 1200 2300 1200
NoConn ~ 3700 6500
NoConn ~ 3700 6400
NoConn ~ 3700 6300
NoConn ~ 3700 6200
NoConn ~ 3700 6100
Wire Wire Line
	2600 6500 2500 6500
Wire Wire Line
	2500 6500 2500 7000
Wire Wire Line
	2500 7000 3800 7000
Wire Wire Line
	3800 7000 3800 6800
Wire Wire Line
	3800 7000 4000 7000
Entry Wire Line
	4100 7000 4000 7100
Wire Wire Line
	2600 6400 2400 6400
Wire Wire Line
	2400 6400 2400 7100
Wire Wire Line
	2400 7100 4000 7100
Text Label 4000 7100 2    50   ~ 0
~NRST
Wire Wire Line
	9900 4600 10100 4600
Wire Wire Line
	10100 4600 10100 3450
Wire Wire Line
	10100 3450 8700 3450
Wire Wire Line
	8700 3450 8700 2700
Wire Wire Line
	8700 2700 8900 2700
Wire Wire Line
	9900 2400 10000 2400
Wire Wire Line
	8900 4100 8800 4100
Wire Wire Line
	8800 4100 8800 4200
Wire Wire Line
	8900 4200 8800 4200
Connection ~ 8800 4200
Entry Wire Line
	10300 2300 10400 2400
Entry Wire Line
	10300 2600 10400 2700
Entry Wire Line
	10300 4500 10400 4600
Entry Wire Line
	10300 4800 10400 4900
Wire Wire Line
	9900 4500 10300 4500
Wire Wire Line
	9900 4800 10100 4800
Text Label 10300 2600 2    50   ~ 0
GND
Text Label 10300 2300 2    50   ~ 0
3.3V
Text Label 10300 4800 2    50   ~ 0
GND
Text Label 10300 4500 2    50   ~ 0
3.3V
Entry Wire Line
	8100 5100 8000 5200
Entry Wire Line
	8100 5000 8000 5100
Wire Wire Line
	8800 5500 10100 5500
Wire Wire Line
	10100 5500 10100 4800
Connection ~ 10100 4800
Wire Wire Line
	10100 4800 10300 4800
Wire Wire Line
	8100 5100 8900 5100
Wire Wire Line
	8900 3000 8800 3000
Wire Wire Line
	8800 3000 8800 3350
Wire Wire Line
	8800 3350 10100 3350
Connection ~ 10100 2600
Wire Wire Line
	10100 2600 10300 2600
Wire Wire Line
	8900 2800 8100 2800
Wire Wire Line
	8100 2900 8900 2900
Entry Wire Line
	8000 3000 8100 2900
Entry Wire Line
	8000 2900 8100 2800
Text Label 8100 2900 0    50   ~ 0
DIO_SCK
Text Label 8100 5000 0    50   ~ 0
DIO_SS
Text Label 8100 2800 0    50   ~ 0
DIO_SS
Text Label 8100 5100 0    50   ~ 0
DIO_SCK
Text Label 9050 3450 0    50   ~ 0
DIO_IN_BRIDGE
Text Notes 9000 1650 0    50   ~ 0
8-Bit Parallel to\nSerial Shift Register
Text Notes 9000 3850 0    50   ~ 0
8-Bit Parallel to\nSerial Shift Register
Text Notes 4750 5450 0    50   ~ 0
Lidar\nConnector
Text Notes 2250 5950 0    50   ~ 0
8-Bit Parallel\nto Serial\nShift Register
Text Label 1500 4000 0    50   ~ 0
DIO_SCK
Wire Wire Line
	1500 6100 2600 6100
Wire Wire Line
	2250 6200 2600 6200
Wire Wire Line
	2250 6300 2600 6300
Text Label 4000 7000 2    50   ~ 0
GND
Wire Wire Line
	1500 4000 2050 4000
Entry Wire Line
	2250 6200 2150 6300
Entry Wire Line
	2250 6300 2150 6400
Wire Bus Line
	4900 7300 4900 6700
Wire Bus Line
	2150 7300 4900 7300
Text Label 2250 6200 0    50   ~ 0
DIO_SCK
Text Label 2250 6300 0    50   ~ 0
~DIO_NSS
Entry Wire Line
	8100 3250 8000 3350
Text HLabel 1500 3900 0    50   Input ~ 0
DIO_MISO
Text Label 1500 3900 0    50   ~ 0
DIO_MISO
Wire Wire Line
	2050 3900 1500 3900
Entry Wire Line
	2050 3900 2150 4000
Text Label 8100 3250 0    50   ~ 0
DIO_MISO
Text Notes 6050 800  0    50   ~ 0
Power/Ground/Reset Bus
Text Notes 6700 6450 0    50   ~ 0
Power/Ground/Reset Bus
Text Notes 2900 7250 0    50   ~ 0
Digitial I/O Bus
Text Notes 5500 6800 0    50   ~ 0
Digitial I/O Bus
Text Notes 8000 3700 0    50   ~ 0
Digitial I/O Bus
Text Label 3900 800  0    50   ~ 0
3.3V
Text Label 3600 800  0    50   ~ 0
GND
$Comp
L Device:C C?
U 1 1 5F97D822
P 4100 1150
AR Path="/5F48CAAD/5F97D822" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D822" Ref="C14"  Part="1" 
F 0 "C14" H 3900 1250 50  0000 L CNN
F 1 "0.1µF;1005" H 4150 1050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4138 1000 50  0001 C CNN
F 3 "~" H 4100 1150 50  0001 C CNN
	1    4100 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1000 4100 900 
Wire Wire Line
	4100 900  3900 900 
Wire Wire Line
	4100 1300 4100 1400
Entry Wire Line
	3600 800  3500 700 
$Comp
L Device:C C?
U 1 1 5F97D82E
P 4700 1150
AR Path="/5F48CAAD/5F97D82E" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D82E" Ref="C15"  Part="1" 
F 0 "C15" H 4500 1250 50  0000 L CNN
F 1 "0.1µF;1005" H 4750 1050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4738 1000 50  0001 C CNN
F 3 "~" H 4700 1150 50  0001 C CNN
	1    4700 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5F97D834
P 5300 1150
AR Path="/5F48CAAD/5F97D834" Ref="C?"  Part="1" 
AR Path="/5F4CEE66/5F97D834" Ref="C16"  Part="1" 
F 0 "C16" H 5100 1250 50  0000 L CNN
F 1 "0.1µF;1005" H 5350 1050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5338 1000 50  0001 C CNN
F 3 "~" H 5300 1150 50  0001 C CNN
	1    5300 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 900  4700 900 
Connection ~ 4100 900 
Wire Wire Line
	4700 1000 4700 900 
Connection ~ 4700 900 
Wire Wire Line
	4700 900  5300 900 
Wire Wire Line
	5300 1000 5300 900 
Wire Wire Line
	4100 1400 4700 1400
Wire Wire Line
	5300 1300 5300 1400
Wire Wire Line
	4700 1300 4700 1400
Connection ~ 4700 1400
Wire Wire Line
	4700 1400 5300 1400
Wire Wire Line
	3900 900  3900 800 
Wire Wire Line
	4100 1400 3600 1400
Wire Wire Line
	3600 1400 3600 800 
Text Notes 4400 850  0    50   ~ 0
ByPass Capacitors
Text Notes 6400 1100 0    50   ~ 0
Edge Sensor Connectors
Entry Wire Line
	3900 800  3800 700 
$Comp
L HR2:SENSE;M1x3 CN4
U 1 1 5FBBDB3C
P 6300 1300
F 0 "CN4" H 6600 1450 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 1050 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 1350 60  0001 L CNN
F 3 "Sense M1X3" H 6500 1150 60  0001 L CNN
F 4 "Sense F1X3" H 6500 1050 60  0001 L CNN "Field5"
	1    6300 1300
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN11
U 1 1 5FBBE822
P 6300 1800
F 0 "CN11" H 6600 1950 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 1550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 1850 60  0001 L CNN
F 3 "Sense M1X3" H 6500 1650 60  0001 L CNN
F 4 "Sense F1X3" H 6500 1550 60  0001 L CNN "Field5"
	1    6300 1800
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN12
U 1 1 5FBBECC7
P 6300 2300
F 0 "CN12" H 6600 2450 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 2050 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 2350 60  0001 L CNN
F 3 "Sense M1X3" H 6500 2150 60  0001 L CNN
F 4 "Sense F1X3" H 6500 2050 60  0001 L CNN "Field5"
	1    6300 2300
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN13
U 1 1 5FBBEE32
P 6300 2800
F 0 "CN13" H 6600 2950 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 2550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 2850 60  0001 L CNN
F 3 "Sense M1X3" H 6500 2650 60  0001 L CNN
F 4 "Sense F1X3" H 6500 2550 60  0001 L CNN "Field5"
	1    6300 2800
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN14
U 1 1 5FBCA45F
P 6300 3300
F 0 "CN14" H 6600 3450 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 3050 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 3350 60  0001 L CNN
F 3 "Sense M1X3" H 6500 3150 60  0001 L CNN
F 4 "Sense F1X3" H 6500 3050 60  0001 L CNN "Field5"
	1    6300 3300
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN15
U 1 1 5FBCA5EE
P 6300 3800
F 0 "CN15" H 6600 3950 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 3550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 3850 60  0001 L CNN
F 3 "Sense M1X3" H 6500 3650 60  0001 L CNN
F 4 "Sense F1X3" H 6500 3550 60  0001 L CNN "Field5"
	1    6300 3800
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN16
U 1 1 5FBCA5F9
P 6300 4300
F 0 "CN16" H 6600 4450 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 4050 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 4350 60  0001 L CNN
F 3 "Sense M1X3" H 6500 4150 60  0001 L CNN
F 4 "Sense F1X3" H 6500 4050 60  0001 L CNN "Field5"
	1    6300 4300
	1    0    0    -1  
$EndComp
$Comp
L HR2:SENSE;M1x3 CN17
U 1 1 5FBCA604
P 6300 4800
F 0 "CN17" H 6600 4950 50  0000 C CNN
F 1 "SENSE;M1x3" H 6700 4550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6500 4850 60  0001 L CNN
F 3 "Sense M1X3" H 6500 4650 60  0001 L CNN
F 4 "Sense F1X3" H 6500 4550 60  0001 L CNN "Field5"
	1    6300 4800
	1    0    0    -1  
$EndComp
Entry Wire Line
	6000 1200 6100 1300
Entry Wire Line
	6000 1300 6100 1400
Text Label 6100 1400 0    50   ~ 0
GND
Text Label 6100 1300 0    50   ~ 0
3.3V
Wire Wire Line
	6300 1300 6100 1300
Wire Wire Line
	6300 1400 6100 1400
Entry Wire Line
	6000 1700 6100 1800
Entry Wire Line
	6000 1800 6100 1900
Text Label 6100 1900 0    50   ~ 0
GND
Text Label 6100 1800 0    50   ~ 0
3.3V
Wire Wire Line
	6300 1800 6100 1800
Wire Wire Line
	6300 1900 6100 1900
Entry Wire Line
	6000 2200 6100 2300
Entry Wire Line
	6000 2300 6100 2400
Text Label 6100 2400 0    50   ~ 0
GND
Text Label 6100 2300 0    50   ~ 0
3.3V
Wire Wire Line
	6300 2300 6100 2300
Wire Wire Line
	6300 2400 6100 2400
Entry Wire Line
	6000 2700 6100 2800
Entry Wire Line
	6000 2800 6100 2900
Text Label 6100 2900 0    50   ~ 0
GND
Text Label 6100 2800 0    50   ~ 0
3.3V
Wire Wire Line
	6300 2800 6100 2800
Wire Wire Line
	6300 2900 6100 2900
Entry Wire Line
	6000 3200 6100 3300
Entry Wire Line
	6000 3300 6100 3400
Text Label 6100 3400 0    50   ~ 0
GND
Text Label 6100 3300 0    50   ~ 0
3.3V
Wire Wire Line
	6300 3300 6100 3300
Wire Wire Line
	6300 3400 6100 3400
Entry Wire Line
	6000 3700 6100 3800
Entry Wire Line
	6000 3800 6100 3900
Text Label 6100 3900 0    50   ~ 0
GND
Text Label 6100 3800 0    50   ~ 0
3.3V
Wire Wire Line
	6300 3800 6100 3800
Wire Wire Line
	6300 3900 6100 3900
Entry Wire Line
	6000 4200 6100 4300
Entry Wire Line
	6000 4300 6100 4400
Text Label 6100 4400 0    50   ~ 0
GND
Text Label 6100 4300 0    50   ~ 0
3.3V
Wire Wire Line
	6300 4300 6100 4300
Wire Wire Line
	6300 4400 6100 4400
Entry Wire Line
	6000 4700 6100 4800
Entry Wire Line
	6000 4800 6100 4900
Text Label 6100 4900 0    50   ~ 0
GND
Text Label 6100 4800 0    50   ~ 0
3.3V
Wire Wire Line
	6300 4800 6100 4800
Wire Wire Line
	6300 4900 6100 4900
Wire Wire Line
	7300 4900 7900 4900
Wire Wire Line
	7900 4900 7900 2600
Wire Wire Line
	7300 4400 7800 4400
Wire Wire Line
	7800 4400 7800 2500
Wire Wire Line
	7300 3900 7700 3900
Wire Wire Line
	7700 3900 7700 2400
Wire Wire Line
	7700 2400 8900 2400
Wire Wire Line
	7300 3400 7600 3400
Wire Wire Line
	7600 3400 7600 2300
Wire Wire Line
	7600 2300 8900 2300
Wire Wire Line
	7300 2900 7500 2900
Wire Wire Line
	7500 2900 7500 2200
Wire Wire Line
	7500 2200 8900 2200
Wire Bus Line
	6000 700  10400 700 
Connection ~ 6000 700 
Wire Wire Line
	7300 2400 7400 2400
Wire Wire Line
	7400 2400 7400 2100
Wire Wire Line
	7400 2100 8900 2100
Wire Wire Line
	7400 2000 7400 1900
Wire Wire Line
	7400 1900 7300 1900
Wire Wire Line
	7400 2000 8900 2000
Wire Wire Line
	7500 1900 7500 1400
Wire Wire Line
	7500 1400 7300 1400
Wire Wire Line
	7500 1900 8900 1900
Text Label 7300 1400 0    50   ~ 0
SIG1
Text Label 7400 2000 0    50   ~ 0
SIG2
Text Label 7400 2100 0    50   ~ 0
SIG3
Text Label 7300 2900 0    50   ~ 0
SIG4
Text Label 7300 3400 0    50   ~ 0
SIG5
Text Label 7300 3900 0    50   ~ 0
SIG6
Text Label 7300 4400 0    50   ~ 0
SIG7
Text Label 7300 4900 0    50   ~ 0
SIG8
NoConn ~ 3700 6600
Wire Wire Line
	7900 2600 8900 2600
Wire Wire Line
	7800 2500 8900 2500
NoConn ~ 9900 2500
Wire Wire Line
	8100 5000 8900 5000
Wire Wire Line
	8900 5200 8800 5200
Wire Wire Line
	8800 5200 8800 5500
NoConn ~ 9900 4700
Wire Wire Line
	8900 4900 8800 4900
NoConn ~ 3700 6700
$Comp
L Device:R_US R?
U 1 1 5FDEB03A
P 3300 3250
AR Path="/5F52F39E/5FDEB03A" Ref="R?"  Part="1" 
AR Path="/5F3221A2/5FDEB03A" Ref="R?"  Part="1" 
AR Path="/5F4CEE66/5FDEB03A" Ref="R68"  Part="1" 
F 0 "R68" H 3150 3350 50  0000 L CNN
F 1 "100KΩ;1005" H 3350 3150 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3340 3240 50  0001 C CNN
F 3 "~" H 3300 3250 50  0001 C CNN
	1    3300 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 4000 3300 4100
Wire Wire Line
	2800 3000 3300 3000
Wire Wire Line
	3300 3000 3300 3100
Wire Wire Line
	3300 4100 2800 4100
Entry Wire Line
	2150 3900 2250 3800
Text Label 2250 3800 0    50   ~ 0
~DIO_NSS
Entry Wire Line
	2800 3000 2700 2900
Text Label 2800 3000 0    50   ~ 0
3.3V
Wire Wire Line
	2250 3800 3000 3800
Entry Wire Line
	2800 4100 2700 4000
Text Label 2800 4100 0    50   ~ 0
GND
Wire Bus Line
	2400 700  2700 700 
Connection ~ 2700 700 
Text Label 2250 4250 0    50   ~ 0
DIO_SS
Entry Wire Line
	2250 4250 2150 4350
Text Notes 2800 2850 0    50   ~ 0
~DIO_NSS~ to DIOS_SS Inverter
Connection ~ 4100 1400
Text HLabel 1500 4400 0    50   Input ~ 0
~NMOTOR_FAULT
Text HLabel 1500 4500 0    50   Input ~ 0
~NBTN_ESTOP
Text HLabel 1500 4600 0    50   Input ~ 0
~NWOW_ESTOP
Wire Wire Line
	1500 4600 2050 4600
Wire Wire Line
	1500 4500 2050 4500
Entry Wire Line
	2050 4600 2150 4700
Entry Wire Line
	2050 4500 2150 4600
Wire Wire Line
	2050 4400 1500 4400
Entry Wire Line
	2050 4400 2150 4500
Text Label 1500 4400 0    50   ~ 0
~NMOTOR_FAULT
Text Label 1500 4500 0    50   ~ 0
~NBTN_ESTOP
Text Label 1500 4600 0    50   ~ 0
~NWOW_ESTOP
Wire Wire Line
	3300 3400 3300 3500
Wire Wire Line
	3300 3500 4000 3500
Wire Wire Line
	4000 3500 4000 4250
Wire Wire Line
	2250 4250 4000 4250
Connection ~ 3300 3500
Wire Wire Line
	3300 3500 3300 3600
Entry Wire Line
	2050 4000 2150 4100
Entry Wire Line
	2050 3800 2150 3900
Wire Wire Line
	1500 3800 2050 3800
Text Label 1500 3800 0    50   ~ 0
~DIO_NSS
Text HLabel 1500 3800 0    50   Input ~ 0
~DIO_NSS
Entry Wire Line
	8100 4800 8000 4900
Entry Wire Line
	8100 4700 8000 4800
Entry Wire Line
	8100 4600 8000 4700
Text Label 8650 4600 2    50   ~ 0
~NMOTOR_FAULT
Text Label 8100 4700 0    50   ~ 0
~NBTN_ESTOP
Text Label 8100 4800 0    50   ~ 0
~NWOW_ESTOP
Wire Wire Line
	8100 4600 8900 4600
Wire Wire Line
	8100 4700 8900 4700
Wire Wire Line
	8100 4800 8900 4800
$Comp
L HR2:LIDAR_ADAPTER;2xF1x4_F1x3 CN5
U 1 1 5FD6A981
P 4500 5700
F 0 "CN5" H 4750 5850 50  0000 C CNN
F 1 "LIDAR_ADAPTER;2xF1x4_F1x3" H 5000 5050 50  0000 C CNN
F 2 "HR2:LIDAR_ADAPTER_2xF1x4_F1x3" H 4700 5750 60  0001 L CNN
F 3 "Lidar Adapter Connectors" H 4700 5550 60  0001 L CNN
F 4 "Lidar Adapter Connectors" H 4700 5450 60  0001 L CNN "desc"
	1    4500 5700
	1    0    0    -1  
$EndComp
Text Label 7900 6100 2    50   ~ 0
LDR_ID1
Text Label 7900 6200 2    50   ~ 0
LDR_ID2
Wire Wire Line
	8800 4200 8800 4900
Wire Wire Line
	8100 4300 8900 4300
Wire Wire Line
	8100 4400 8900 4400
Entry Wire Line
	8100 4300 8000 4400
Wire Wire Line
	8100 4500 8900 4500
Entry Wire Line
	8000 4500 8100 4400
Entry Wire Line
	8000 4600 8100 4500
Text Label 8100 4300 0    50   ~ 0
LDR_ID0
Text Label 8100 4400 0    50   ~ 0
LDR_ID1
Text Label 8100 4500 0    50   ~ 0
LDR_ID2
Text Notes 8300 4200 0    50   ~ 0
D0/D0 are\nPCB Rev. ID
Wire Bus Line
	4100 6500 6000 6500
Connection ~ 4100 6500
Wire Bus Line
	10400 6500 6000 6500
Connection ~ 6000 6500
Wire Wire Line
	3700 6000 4500 6000
Wire Wire Line
	3700 5900 4500 5900
Connection ~ 3800 7000
Wire Wire Line
	4500 6100 4200 6100
Wire Wire Line
	4200 6100 4200 5400
Wire Wire Line
	4500 5800 4300 5800
Wire Wire Line
	4300 5800 4300 5300
Wire Wire Line
	4500 5700 4400 5700
Wire Wire Line
	4400 5700 4400 5200
Wire Bus Line
	4900 6700 8000 6700
Text Label 7900 6000 2    50   ~ 0
LDR_ID0
$Comp
L Device:R_US R?
U 1 1 5FF762CF
P 6350 5550
AR Path="/5F52F39E/5FF762CF" Ref="R?"  Part="1" 
AR Path="/5F3221A2/5FF762CF" Ref="R?"  Part="1" 
AR Path="/5F4CEE66/5FF762CF" Ref="R1"  Part="1" 
F 0 "R1" H 6200 5650 50  0000 L CNN
F 1 "100KΩ;1005" H 6350 5400 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6390 5540 50  0001 C CNN
F 3 "~" H 6350 5550 50  0001 C CNN
	1    6350 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5FF76BFD
P 6950 5650
AR Path="/5F52F39E/5FF76BFD" Ref="R?"  Part="1" 
AR Path="/5F3221A2/5FF76BFD" Ref="R?"  Part="1" 
AR Path="/5F4CEE66/5FF76BFD" Ref="R2"  Part="1" 
F 0 "R2" H 6800 5750 50  0000 L CNN
F 1 "100KΩ;1005" H 6950 5500 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6990 5640 50  0001 C CNN
F 3 "~" H 6950 5650 50  0001 C CNN
	1    6950 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 5FF771F3
P 7550 5500
AR Path="/5F52F39E/5FF771F3" Ref="R?"  Part="1" 
AR Path="/5F3221A2/5FF771F3" Ref="R?"  Part="1" 
AR Path="/5F4CEE66/5FF771F3" Ref="R3"  Part="1" 
F 0 "R3" H 7400 5600 50  0000 L CNN
F 1 "100KΩ;1005" H 7050 5350 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7590 5490 50  0001 C CNN
F 3 "~" H 7550 5500 50  0001 C CNN
	1    7550 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 5350 7550 5250
Wire Wire Line
	7550 5250 6950 5250
Entry Wire Line
	7900 6200 8000 6100
Entry Wire Line
	7900 6100 8000 6000
Entry Wire Line
	7900 6000 8000 5900
Wire Wire Line
	6350 5700 6350 6000
Connection ~ 6350 6000
Wire Wire Line
	6350 6000 7900 6000
Wire Wire Line
	6950 5800 6950 6100
Wire Wire Line
	6950 6100 7900 6100
Wire Wire Line
	7550 5650 7550 6200
Connection ~ 7550 6200
Wire Wire Line
	7550 6200 7900 6200
Wire Wire Line
	6950 5500 6950 5250
Connection ~ 6950 5250
Wire Wire Line
	6950 5250 6350 5250
Wire Wire Line
	6350 5400 6350 5250
Connection ~ 6350 5250
Wire Wire Line
	6350 5250 6100 5250
Entry Wire Line
	6100 5250 6000 5150
Text Label 6100 5250 0    50   ~ 0
3.3V
Wire Wire Line
	8800 5200 8800 4900
Connection ~ 8800 5200
Connection ~ 8800 4900
Wire Wire Line
	8100 3250 10000 3250
Wire Wire Line
	10100 2600 10100 3350
Wire Wire Line
	5500 6200 7550 6200
Connection ~ 6950 6100
Wire Wire Line
	5500 6100 6950 6100
Wire Wire Line
	5500 6000 6350 6000
$Comp
L Transistor_FET:2N7002 Q25
U 1 1 5FE994E8
P 3200 3800
F 0 "Q25" H 3050 3950 50  0000 L CNN
F 1 "2N7002;SOT23" H 3350 3700 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3400 3725 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 3200 3800 50  0001 L CNN
	1    3200 3800
	1    0    0    -1  
$EndComp
$Comp
L HR2:SN74HC595;HTSSOP16EP3.4x5 U5
U 1 1 5FE359DB
P 2600 5800
F 0 "U5" H 2850 5950 50  0000 C CNN
F 1 "SN74HC595;HTSSOP16EP3.4x5" H 3200 4650 50  0000 C CNN
F 2 "Package_SO:HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm" H 2800 5850 60  0001 L CNN
F 3 "Tri-state 8-Bit Shift Register" H 2800 5650 60  0001 L CNN
F 4 "Tri-State 8-Bit Shift Register" H 2800 5550 60  0001 L CNN "desc"
	1    2600 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 2600 10100 2600
Wire Wire Line
	10000 2400 10000 3250
Wire Wire Line
	9900 2300 10300 2300
$Comp
L HR2:SN74HC165;HTSSOP16EP3.4x5 U6
U 1 1 5FE18228
P 8900 1900
F 0 "U6" H 9150 2050 50  0000 C CNN
F 1 "SN74HC165;HTSSOP16EP3.4x5" H 9400 650 50  0000 C CNN
F 2 "Package_SO:HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm" H 9100 1950 60  0001 L CNN
F 3 "8-Bit Parallel In Serial Out Shift Register" H 9100 1750 60  0001 L CNN
F 4 "8-Bit Parallel In Serial Out Shift Register" H 9100 1650 60  0001 L CNN "desc"
	1    8900 1900
	1    0    0    -1  
$EndComp
$Comp
L HR2:SN74HC165;HTSSOP16EP3.4x5 U7
U 1 1 5FE19F67
P 8900 4100
F 0 "U7" H 9150 4250 50  0000 C CNN
F 1 "SN74HC165;HTSSOP16EP3.4x5" H 9400 2850 50  0000 C CNN
F 2 "Package_SO:HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm" H 9100 4150 60  0001 L CNN
F 3 "8-Bit Parallel In Serial Out Shift Register" H 9100 3950 60  0001 L CNN
F 4 "8-Bit Parallel In Serial Out Shift Register" H 9100 3850 60  0001 L CNN "desc"
	1    8900 4100
	1    0    0    -1  
$EndComp
Wire Bus Line
	4100 5600 4100 6500
Wire Bus Line
	4100 6500 4100 7100
Wire Bus Line
	2700 700  6000 700 
Wire Bus Line
	2700 700  2700 4100
Wire Bus Line
	2400 700  2400 1200
Wire Bus Line
	10400 700  10400 6500
Wire Bus Line
	2150 3800 2150 7300
Wire Bus Line
	8000 2800 8000 6700
Wire Bus Line
	6000 700  6000 6500
$EndSCHEMATC
