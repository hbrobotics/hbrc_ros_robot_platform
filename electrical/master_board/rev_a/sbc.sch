EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 3 10
Title "HR2 Single Board Computer"
Date "2020-11-22"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
NoConn ~ 3900 2600
NoConn ~ 3900 3100
NoConn ~ 3900 3200
NoConn ~ 3900 3400
NoConn ~ 3900 3500
NoConn ~ 3900 3700
NoConn ~ 3900 3800
NoConn ~ 3900 4000
NoConn ~ 3900 4100
NoConn ~ 3900 4800
NoConn ~ 3900 5000
NoConn ~ 3900 5100
NoConn ~ 3900 5900
NoConn ~ 3900 5700
NoConn ~ 3900 5600
NoConn ~ 3900 5500
NoConn ~ 3900 5400
NoConn ~ 3900 5200
$Comp
L Device:Crystal X1
U 1 1 5F4F5A10
P 6650 3200
AR Path="/5F4877FA/5F4F5A10" Ref="X1"  Part="1" 
AR Path="/5FA587FE/5F4F5A10" Ref="X?"  Part="1" 
F 0 "X1" H 6650 3350 50  0000 C CNN
F 1 "32.768kHz9pF;3.2x1.5" H 6600 3050 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_5032-2Pin_5.0x3.2mm" H 6650 3200 50  0001 C CNN
F 3 "~" H 6650 3200 50  0001 C CNN
	1    6650 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5F5097A4
P 7300 3450
AR Path="/5F4877FA/5F5097A4" Ref="C9"  Part="1" 
AR Path="/5FA587FE/5F5097A4" Ref="C?"  Part="1" 
F 0 "C9" H 7150 3550 50  0000 L CNN
F 1 "16pF;1608" H 7300 3350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7338 3300 50  0001 C CNN
F 3 "~" H 7300 3450 50  0001 C CNN
	1    7300 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5F509B6B
P 6000 3450
AR Path="/5F4877FA/5F509B6B" Ref="C7"  Part="1" 
AR Path="/5FA587FE/5F509B6B" Ref="C?"  Part="1" 
F 0 "C7" H 5850 3550 50  0000 L CNN
F 1 "16pF;1608" H 6000 3350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6038 3300 50  0001 C CNN
F 3 "~" H 6000 3450 50  0001 C CNN
	1    6000 3450
	1    0    0    -1  
$EndComp
Text Notes 5600 1500 0    50   ~ 0
Real Time Clock Notes:\n1. Real Time Clock is needed to set the operatiing system time need for\n   ROS (Robot Operating System.)\n2. The crystal load capacitor calculations are:\n   A. Cld = (C1 x C2) / (C1 + C2), where:\n      Cld = crystal load capacitence (i.e. 7-9pF)  assume 8pF\n      C1 and C2 are two capactors connected to ground, one to each crystal side.\n   B. Assume C1 = C2 and call it C.\n   C. Cld = (C1 x C2) / (C1 + C2)  = (C x C) / (C + C)  = C² / 2C = C / 2.\n   D. C = 2 ✕ Cload  = 2 ✕ 8pF = 16pF.\n
Wire Wire Line
	6800 3200 6900 3200
Wire Wire Line
	7300 3200 7300 3300
Wire Wire Line
	6500 3200 6400 3200
Wire Wire Line
	6000 3200 6000 3300
Wire Wire Line
	6000 3600 6000 3700
Wire Wire Line
	6000 3700 7300 3700
Wire Wire Line
	7300 3700 7300 3600
Wire Wire Line
	7000 2900 6900 2900
Wire Wire Line
	6900 2900 6900 3200
Connection ~ 6900 3200
Wire Wire Line
	6900 3200 7300 3200
Wire Wire Line
	6400 3200 6400 2800
Wire Wire Line
	6400 2800 7000 2800
Connection ~ 6400 3200
Wire Wire Line
	6400 3200 6000 3200
Connection ~ 6000 3700
$Comp
L Device:R_US R8
U 1 1 5F510131
P 5500 1950
AR Path="/5F4877FA/5F510131" Ref="R8"  Part="1" 
AR Path="/5FA587FE/5F510131" Ref="R?"  Part="1" 
F 0 "R8" H 5350 2050 50  0000 L CNN
F 1 "3.9KΩ;1608" H 5550 1850 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5540 1940 50  0001 C CNN
F 3 "~" H 5500 1950 50  0001 C CNN
	1    5500 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R10
U 1 1 5F510F46
P 6100 1950
AR Path="/5F4877FA/5F510F46" Ref="R10"  Part="1" 
AR Path="/5FA587FE/5F510F46" Ref="R?"  Part="1" 
F 0 "R10" H 5950 2050 50  0000 L CNN
F 1 "3.9KΩ;1608" H 6150 1850 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6140 1940 50  0001 C CNN
F 3 "~" H 6100 1950 50  0001 C CNN
	1    6100 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2200 6900 2200
Wire Wire Line
	6900 2200 6900 1700
Wire Wire Line
	6100 1700 6100 1800
Wire Wire Line
	6100 2400 6100 2100
Wire Wire Line
	7000 2500 5500 2500
Wire Wire Line
	5500 2500 5500 2200
Wire Wire Line
	6100 1700 5500 1700
Wire Wire Line
	5500 1700 5500 1800
Connection ~ 6100 1700
NoConn ~ 7000 2300
$Comp
L HR2:CAT24C32;SOIC8 U3
U 1 1 5F50901B
P 8900 6000
AR Path="/5F4877FA/5F50901B" Ref="U3"  Part="1" 
AR Path="/5FA587FE/5F50901B" Ref="U?"  Part="1" 
F 0 "U3" H 9400 5150 50  0000 R CNN
F 1 "CAT24C32;SOIC8" H 9550 6150 50  0000 R CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 9100 6050 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/CAT24C32-D.PDF" H 9100 5850 60  0001 L CNN
F 4 "https://www.onsemi.com/pub/Collateral/CAT24C32-D.PDF" H 9100 5950 60  0001 L CNN "manf#"
F 5 "32Kb I2C EEPROM" H 9100 5750 60  0001 L CNN "Field5"
	1    8900 6000
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R13
U 1 1 5F50DB19
P 6900 5050
AR Path="/5F4877FA/5F50DB19" Ref="R13"  Part="1" 
AR Path="/5FA587FE/5F50DB19" Ref="R?"  Part="1" 
F 0 "R13" H 6750 5150 50  0000 L CNN
F 1 "4.7KΩ;1608" H 6950 4950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6940 5040 50  0001 C CNN
F 3 "~" H 6900 5050 50  0001 C CNN
	1    6900 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R14
U 1 1 5F50E10E
P 7500 5050
AR Path="/5F4877FA/5F50E10E" Ref="R14"  Part="1" 
AR Path="/5FA587FE/5F50E10E" Ref="R?"  Part="1" 
F 0 "R14" H 7350 5150 50  0000 L CNN
F 1 "100KΩ;1608" H 7550 4950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7540 5040 50  0001 C CNN
F 3 "~" H 7500 5050 50  0001 C CNN
	1    7500 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 5300 8100 5300
Wire Wire Line
	8100 5300 8100 4200
Wire Wire Line
	8100 4200 7500 4200
Wire Wire Line
	6900 4200 6900 4900
Wire Wire Line
	7500 4900 7500 4200
Connection ~ 7500 4200
Wire Wire Line
	7500 4200 7300 4200
$Comp
L Device:R_US R12
U 1 1 5F50FCF9
P 6300 5050
AR Path="/5F4877FA/5F50FCF9" Ref="R12"  Part="1" 
AR Path="/5FA587FE/5F50FCF9" Ref="R?"  Part="1" 
F 0 "R12" H 6150 5150 50  0000 L CNN
F 1 "4.7KΩ;1608" H 6350 4950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6340 5040 50  0001 C CNN
F 3 "~" H 6300 5050 50  0001 C CNN
	1    6300 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 4200 6900 4200
Connection ~ 6900 4200
Wire Wire Line
	8200 5400 7500 5400
Wire Wire Line
	7500 5400 7500 5200
Wire Wire Line
	8200 5500 6900 5500
Wire Wire Line
	6900 5500 6900 5200
Wire Wire Line
	8200 5600 6300 5600
Wire Wire Line
	6300 5600 6300 5200
Wire Wire Line
	8200 5700 8000 5700
Wire Wire Line
	8000 5700 8000 5800
Wire Wire Line
	8000 6000 8200 6000
Wire Wire Line
	8200 5900 8000 5900
Connection ~ 8000 5900
Wire Wire Line
	8000 5900 8000 6000
Wire Wire Line
	8200 5800 8000 5800
Connection ~ 8000 5800
Wire Wire Line
	8000 5800 8000 5900
Text Notes 8200 4600 0    50   ~ 0
EEPROM Notes:\n1. As per Raspberry PI I2C ID specification, the EEPROM\n     address is 0x50 (i.e. A0=A1=A2=0=Ground.)\n2. To enable the storage into the EEPROM s, short jumper.\n
Wire Wire Line
	3900 4600 4200 4600
Wire Wire Line
	4200 4600 4200 5600
Connection ~ 6300 5600
Wire Wire Line
	6900 5500 4300 5500
Wire Wire Line
	4300 5500 4300 4700
Wire Wire Line
	4300 4700 3900 4700
Connection ~ 6900 5500
Wire Wire Line
	3900 2500 4000 2500
Wire Wire Line
	4000 2500 4000 2800
Wire Wire Line
	3900 5300 4000 5300
Connection ~ 4000 5300
Wire Wire Line
	4000 5300 4000 5800
Wire Wire Line
	3900 4900 4000 4900
Connection ~ 4000 4900
Wire Wire Line
	4000 4900 4000 5300
Wire Wire Line
	3900 2100 4000 2100
Wire Wire Line
	4000 2100 4000 2300
Wire Wire Line
	4000 2300 3900 2300
Wire Wire Line
	8000 5800 5500 5800
Connection ~ 4000 5800
Connection ~ 4000 3700
Wire Wire Line
	4000 3700 4000 3900
Text HLabel 5000 2700 2    50   Output ~ 0
SBC_TX
Wire Wire Line
	3900 2700 4800 2700
Text HLabel 5000 2900 2    50   Output ~ 0
SBC_RX
Wire Wire Line
	3900 2900 4800 2900
Wire Wire Line
	3900 2800 4000 2800
Connection ~ 4000 2800
Wire Wire Line
	4000 2800 4000 3300
Text HLabel 5000 2100 2    50   Input ~ 0
5V
Wire Wire Line
	4000 2100 5000 2100
Connection ~ 4000 2100
Text HLabel 8400 6300 2    50   Input ~ 0
GND
Wire Wire Line
	4000 5800 4000 6300
Wire Wire Line
	4000 6300 8400 6300
Text Label 5950 6300 0    50   ~ 0
GND
Wire Wire Line
	3900 4400 4000 4400
Connection ~ 4000 4400
Wire Wire Line
	4000 4400 4000 4900
NoConn ~ 3900 4500
Wire Wire Line
	3900 3300 4000 3300
Connection ~ 4000 3300
Wire Wire Line
	4000 3300 4000 3700
Wire Wire Line
	4200 5600 6300 5600
Wire Wire Line
	8100 1700 6900 1700
Connection ~ 8100 4200
Connection ~ 6900 1700
Text HLabel 8400 1700 2    50   Input ~ 0
3.3V
Wire Wire Line
	8100 1700 8400 1700
Connection ~ 8100 1700
Text Label 8400 1700 2    50   ~ 0
3.3V
$Comp
L Device:Jumper_NO_Small JP1
U 1 1 5F563C70
P 7700 6000
AR Path="/5F4877FA/5F563C70" Ref="JP1"  Part="1" 
AR Path="/5FA587FE/5F563C70" Ref="JP?"  Part="1" 
F 0 "JP1" H 7700 6100 50  0000 C CNN
F 1 "JUMPER;M1x2" H 7700 5900 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7700 6000 50  0001 C CNN
F 3 "~" H 7700 6000 50  0001 C CNN
	1    7700 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 6000 8000 6000
Connection ~ 8000 6000
Wire Wire Line
	7500 5400 7500 6000
Wire Wire Line
	7500 6000 7600 6000
Connection ~ 7500 5400
Text Notes 8250 5100 0    50   ~ 0
32Kb I2C Serial Memory\nUsed for Raspberry Pi Discovery
Wire Wire Line
	3900 3900 4000 3900
Connection ~ 4000 3900
Wire Wire Line
	4000 3900 4000 4400
Text HLabel 5600 4500 2    50   Output ~ 0
SBC_ALIVE
Wire Wire Line
	3900 5800 4000 5800
$Comp
L HR2:RASPI;F2X20 CN94
U 1 1 5F488CF3
P 3100 2000
AR Path="/5F4877FA/5F488CF3" Ref="CN94"  Part="1" 
AR Path="/5FA587FE/5F488CF3" Ref="CN?"  Part="1" 
F 0 "CN94" H 3550 2150 50  0000 C CNN
F 1 "RASPI;F2X20" H 3550 -2050 50  0000 C CNN
F 2 "HR2:RASPI_F2X20" H 3300 2050 60  0001 L CNN
F 3 "Raspberry Pi F2x20 Connector (Row Swappped)" H 3300 1850 60  0001 L CNN
	1    3100 2000
	1    0    0    -1  
$EndComp
Text Notes 7050 1950 0    50   ~ 0
Real Time Clock\nwith Battery Backup
Text Label 4050 4200 0    50   ~ 0
SBC_STOP
Text Label 4050 4300 0    50   ~ 0
SBC_ALIVE
Text Notes 1900 1150 0    50   ~ 0
SBC (Single Board Computer) Connector Notes:\n1. Any SBC that is compatible with the defacto 40-pin Raspbery Pi Standard is allowed.\n2. 5V power is provided by the 2 5V pins.\n3. The SBC_ALIVE is High (3.3V) when the operating system is up and running.\n4. The SBC_SHUTDOWN is pulled High to ask the operating system to shutdown.\n5. The SBC_TX/SBC_RX provide a dedicated serial connection to the Nucleo.
$Comp
L Device:R_US R6
U 1 1 5F58D7BC
P 4600 4850
AR Path="/5F4877FA/5F58D7BC" Ref="R6"  Part="1" 
AR Path="/5FA587FE/5F58D7BC" Ref="R?"  Part="1" 
F 0 "R6" H 4450 4950 50  0000 L CNN
F 1 "100KΩ;1608" H 4650 4750 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4640 4840 50  0001 C CNN
F 3 "~" H 4600 4850 50  0001 C CNN
	1    4600 4850
	1    0    0    -1  
$EndComp
Connection ~ 4600 5800
Text Label 7750 5800 0    50   ~ 0
GND
Text Label 4500 3700 0    50   ~ 0
GND
Text Label 7600 4200 0    50   ~ 0
3.3V
Text Label 7750 5500 0    50   ~ 0
ID_SCL
Text Label 7750 5600 0    50   ~ 0
ID_SDA
Connection ~ 5500 3700
Wire Wire Line
	5500 3700 5500 3600
Wire Wire Line
	7000 2700 5500 2700
$Comp
L Device:Battery_Cell BT1
U 1 1 5F4E022E
P 5500 3500
AR Path="/5F4877FA/5F4E022E" Ref="BT1"  Part="1" 
AR Path="/5FA587FE/5F4E022E" Ref="BT?"  Part="1" 
F 0 "BT1" H 5300 3650 50  0000 L CNN
F 1 "COIN_HOLDER6.8R;TE1376164-1" H 4250 3450 50  0000 L CNN
F 2 "HR2:TE1376164-1_COINHOLDER6.8" V 5500 3560 50  0001 C CNN
F 3 "~" V 5500 3560 50  0001 C CNN
	1    5500 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 2700 5500 3300
Wire Wire Line
	5500 3700 4000 3700
Wire Wire Line
	5750 2600 6700 2600
Wire Wire Line
	5750 3700 5500 3700
Connection ~ 5750 3700
Wire Wire Line
	6000 3700 5750 3700
Wire Wire Line
	5750 3700 5750 2600
Wire Wire Line
	6100 2400 3900 2400
Connection ~ 6100 2400
Wire Wire Line
	3900 2200 5500 2200
Connection ~ 5500 2200
Wire Wire Line
	5500 2200 5500 2100
Text Label 4100 2400 0    50   ~ 0
SBC_SCL
Text Label 4100 2200 0    50   ~ 0
SBC_SDA
Text Label 5550 2700 0    50   ~ 0
VBAT
Text Label 6900 2800 0    50   ~ 0
X2
Text Label 6900 2900 0    50   ~ 0
X1
Wire Wire Line
	3900 4200 4600 4200
$Comp
L HR2:MCP7940;SOIC8 U2
U 1 1 5F5D2D12
P 7700 2900
AR Path="/5F4877FA/5F5D2D12" Ref="U2"  Part="1" 
AR Path="/5FA587FE/5F5D2D12" Ref="U?"  Part="1" 
F 0 "U2" H 8200 2050 50  0000 R CNN
F 1 "MCP7940;SOIC8" H 8350 3050 50  0000 R CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7900 2950 60  0001 L CNN
F 3 "I2C Real-time Clock" H 7900 2750 60  0001 L CNN
F 4 "https://ww1.microchip.com/downloads/en/DeviceDoc/20005010H.pdf" H 7900 2850 60  0001 L CNN "manf#"
F 5 "I2C Real Time Clock" H 7900 2650 60  0001 L CNN "Field5"
	1    7700 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	4600 4700 4600 4200
Connection ~ 4600 4200
$Comp
L Switch:SW_Push SW?
U 1 1 5F8FBAFA
P 5900 4200
AR Path="/5F52F39E/5F8FBAFA" Ref="SW?"  Part="1" 
AR Path="/5F4877FA/5F8FBAFA" Ref="SW2"  Part="1" 
F 0 "SW2" H 5750 4300 50  0000 C CNN
F 1 "BUTTON;6x3.5" H 5900 4100 50  0000 C CNN
F 2 "HR2:BUTTON_6x3.5" H 5900 4400 50  0001 C CNN
F 3 "~" H 5900 4400 50  0001 C CNN
	1    5900 4200
	1    0    0    -1  
$EndComp
Text Label 4100 2900 0    50   ~ 0
SBC_RX
Text Label 4100 2700 0    50   ~ 0
SBC_TX
Text Label 4100 2100 0    50   ~ 0
5V
NoConn ~ 3900 2000
Wire Wire Line
	4600 4200 4750 4200
$Comp
L Device:R_US R11
U 1 1 5FAF8B92
P 4900 4200
AR Path="/5F4877FA/5FAF8B92" Ref="R11"  Part="1" 
AR Path="/5FA587FE/5FAF8B92" Ref="R?"  Part="1" 
F 0 "R11" V 4800 4150 50  0000 L CNN
F 1 "1KΩ;1608" V 5000 4050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4940 4190 50  0001 C CNN
F 3 "~" H 4900 4200 50  0001 C CNN
	1    4900 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	5050 4200 5700 4200
$Comp
L Device:LED D?
U 1 1 5FB21992
P 5500 5150
AR Path="/5F79BC00/5FB21992" Ref="D?"  Part="1" 
AR Path="/5FA4A874/5FB21992" Ref="D?"  Part="1" 
AR Path="/5F4877FA/5FB21992" Ref="D2"  Part="1" 
F 0 "D2" V 5600 5250 50  0000 R CNN
F 1 "GRNLED;1608" V 5400 5050 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5500 5150 50  0001 C CNN
F 3 "~" H 5500 5150 50  0001 C CNN
	1    5500 5150
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R9
U 1 1 5FB254F3
P 5500 4750
AR Path="/5F4877FA/5FB254F3" Ref="R9"  Part="1" 
AR Path="/5FA587FE/5FB254F3" Ref="R?"  Part="1" 
F 0 "R9" H 5350 4850 50  0000 L CNN
F 1 "470KΩ;1608" H 5550 4650 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5540 4740 50  0001 C CNN
F 3 "~" H 5500 4750 50  0001 C CNN
	1    5500 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 4300 4500 4500
Wire Wire Line
	4500 4500 5300 4500
Wire Wire Line
	5500 4500 5500 4600
Wire Wire Line
	5500 5000 5500 4900
Connection ~ 5500 5800
Wire Wire Line
	5500 5800 5300 5800
Text Notes 4850 4000 0    50   ~ 0
SBC Shutdown Request Button
Text Notes 5600 5200 0    50   ~ 0
SBC Alive LED
Wire Wire Line
	3900 4300 4500 4300
Wire Wire Line
	4000 5800 4600 5800
Wire Wire Line
	4600 5000 4600 5800
NoConn ~ 3900 3600
Text Notes 3250 1800 0    50   ~ 0
SBC Connector
$Comp
L Device:R_US R7
U 1 1 5F917643
P 5300 5150
AR Path="/5F4877FA/5F917643" Ref="R7"  Part="1" 
AR Path="/5FA587FE/5F917643" Ref="R?"  Part="1" 
F 0 "R7" H 5150 5250 50  0000 L CNN
F 1 "100KΩ;1608" H 4800 5000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5340 5140 50  0001 C CNN
F 3 "~" H 5300 5150 50  0001 C CNN
	1    5300 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5000 5300 4500
Connection ~ 5300 4500
Wire Wire Line
	5300 4500 5500 4500
$Comp
L Device:C C10
U 1 1 5FA44A8F
P 7300 3950
AR Path="/5F4877FA/5FA44A8F" Ref="C10"  Part="1" 
AR Path="/5FA587FE/5FA44A8F" Ref="C?"  Part="1" 
F 0 "C10" H 7150 4050 50  0000 L CNN
F 1 "10pF;1608" H 7300 3850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7338 3800 50  0001 C CNN
F 3 "~" H 7300 3950 50  0001 C CNN
	1    7300 3950
	1    0    0    -1  
$EndComp
Connection ~ 7300 4200
Wire Wire Line
	7300 4200 6900 4200
Wire Wire Line
	7300 3800 7300 3700
Connection ~ 7300 3700
Text Notes 8250 3050 0    50   ~ 0
Example Crystal Part:\nCM7V-T1A-LOW-ESR-32.768KHZ-9PF-20-TA-QC
Text Label 7750 5400 0    50   ~ 0
WP
Wire Wire Line
	6700 1700 6100 1700
Wire Wire Line
	6900 1700 6700 1700
Connection ~ 6700 1700
Wire Wire Line
	6700 2000 6700 1700
Wire Wire Line
	8100 4200 8100 1700
$Comp
L Device:C C8
U 1 1 5FA24EA3
P 6700 2150
AR Path="/5F4877FA/5FA24EA3" Ref="C8"  Part="1" 
AR Path="/5FA587FE/5FA24EA3" Ref="C?"  Part="1" 
F 0 "C8" H 6700 2250 50  0000 L CNN
F 1 "10pF;1608" H 6250 2050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6738 2000 50  0001 C CNN
F 3 "~" H 6700 2150 50  0001 C CNN
	1    6700 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2400 7000 2400
Wire Wire Line
	6700 2300 6700 2600
Connection ~ 6700 2600
Wire Wire Line
	6700 2600 7000 2600
Wire Wire Line
	5500 4500 5600 4500
Connection ~ 5500 4500
Wire Wire Line
	6300 4200 6300 4900
Wire Wire Line
	6100 4200 6300 4200
Connection ~ 6300 4200
Wire Wire Line
	7300 4100 7300 4200
Connection ~ 5300 5800
Wire Wire Line
	5300 5800 4600 5800
Text Notes 6200 5950 0    50   ~ 0
EEPROM Program Enable Jumper
Wire Wire Line
	5300 5300 5300 5800
Wire Wire Line
	5500 5300 5500 5800
Text Label 5100 4200 0    50   ~ 0
SBC_REQ
$Comp
L Connector:TestPoint TP16
U 1 1 5FB4ED7C
P 4800 2600
F 0 "TP16" H 4850 2750 50  0000 L CNN
F 1 "SPC_TX;M1x2" H 4858 2627 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 5000 2600 50  0001 C CNN
F 3 "~" H 5000 2600 50  0001 C CNN
	1    4800 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2100 4000 1600
$Comp
L Jumper:Jumper_3_Open JP9
U 1 1 5FB67673
P 750 3400
F 0 "JP9" V 800 3250 50  0000 C CNN
F 1 "RPI_DISP_EN;M1x3" V 650 3000 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 750 3400 50  0001 C CNN
F 3 "~" H 750 3400 50  0001 C CNN
	1    750  3400
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R65
U 1 1 5FB68DCD
P 750 1750
AR Path="/5F4877FA/5FB68DCD" Ref="R65"  Part="1" 
AR Path="/5FA587FE/5FB68DCD" Ref="R?"  Part="1" 
F 0 "R65" H 600 1850 50  0000 L CNN
F 1 "100KΩ;1608" H 750 1650 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 790 1740 50  0001 C CNN
F 3 "~" H 750 1750 50  0001 C CNN
	1    750  1750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5FB7067A
P 1600 3400
AR Path="/5F79BC00/5FB7067A" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5FB7067A" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5FB7067A" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5FB7067A" Ref="Q?"  Part="1" 
AR Path="/5F4877FA/5FB7067A" Ref="Q24"  Part="1" 
F 0 "Q24" H 1500 3550 50  0000 L CNN
F 1 "2N7000;SOT23" H 1750 3250 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1800 3325 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 1600 3400 50  0001 L CNN
	1    1600 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R67
U 1 1 5FB84C0D
P 1700 2750
AR Path="/5F4877FA/5FB84C0D" Ref="R67"  Part="1" 
AR Path="/5FA587FE/5FB84C0D" Ref="R?"  Part="1" 
F 0 "R67" H 1500 2850 50  0000 L CNN
F 1 "100KΩ;1608" H 1200 2600 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1740 2740 50  0001 C CNN
F 3 "~" H 1700 2750 50  0001 C CNN
	1    1700 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 CN3
U 1 1 5FB885F0
P 2900 2500
F 0 "CN3" H 2800 2600 50  0000 L CNN
F 1 "RPI_DSP_PWR;M1x2" H 2550 2300 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 2900 2500 50  0001 C CNN
F 3 "~" H 2900 2500 50  0001 C CNN
	1    2900 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  3150 750  1900
Wire Wire Line
	1400 3400 1100 3400
Wire Wire Line
	2500 4200 2500 2600
Wire Wire Line
	2500 2600 2700 2600
Wire Wire Line
	1800 2500 1700 2500
Wire Wire Line
	1700 1600 4000 1600
Wire Wire Line
	750  4200 1700 4200
Wire Wire Line
	1700 3600 1700 4200
Connection ~ 1700 4200
Wire Wire Line
	1700 4200 2500 4200
Wire Wire Line
	4000 6300 2500 6300
Wire Wire Line
	2500 6300 2500 4200
Connection ~ 4000 6300
Connection ~ 2500 4200
Wire Wire Line
	5500 1700 5500 1400
Wire Wire Line
	5500 1400 750  1400
Connection ~ 5500 1700
Wire Wire Line
	750  1400 750  1600
Wire Wire Line
	1700 2600 1700 2500
Connection ~ 1700 2500
Wire Wire Line
	1700 1600 1700 2500
Wire Wire Line
	1700 2900 1700 3000
Connection ~ 1700 3000
Wire Wire Line
	1700 3000 1700 3200
Wire Wire Line
	750  4100 750  4200
Wire Wire Line
	750  3800 750  3650
$Comp
L Device:R_US R66
U 1 1 5FB69F2A
P 750 3950
AR Path="/5F4877FA/5FB69F2A" Ref="R66"  Part="1" 
AR Path="/5FA587FE/5FB69F2A" Ref="R?"  Part="1" 
F 0 "R66" H 600 4050 50  0000 L CNN
F 1 "100KΩ;1608" H 800 3850 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 790 3940 50  0001 C CNN
F 3 "~" H 750 3950 50  0001 C CNN
	1    750  3950
	1    0    0    -1  
$EndComp
Text Notes 1900 2300 0    50   ~ 0
Raspberry Pi Display\nPower Enable Circuit
Wire Wire Line
	1100 3400 1100 2500
Wire Wire Line
	1100 2500 1300 2500
Wire Wire Line
	1300 2500 1300 1500
Wire Wire Line
	1300 1500 4600 1500
Wire Wire Line
	4600 1500 4600 3000
Wire Wire Line
	4600 3000 3900 3000
Connection ~ 1100 3400
Wire Wire Line
	1100 3400 900  3400
Text Notes 500  4650 0    50   ~ 0
Notes:\n1. Jumper controlls default display Power On/Off.\n2. Transistor shifts from 3.3V to 5V Gate On/Off.\n3. Jumper provides switched 5V to Ras Pi Display.\n4. Raspberry Pi can override jumper.
Text Notes 800  3700 0    50   ~ 0
ON
Text Notes 800  3150 0    50   ~ 0
OFF
Text Label 4100 3000 0    50   ~ 0
RPI_DSP_EN
$Comp
L Connector:TestPoint TP17
U 1 1 5FC4D458
P 4800 3000
F 0 "TP17" H 4600 3200 50  0000 L CNN
F 1 "SBC_RX;M1x2" H 4250 3050 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 5000 3000 50  0001 C CNN
F 3 "~" H 5000 3000 50  0001 C CNN
	1    4800 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 2600 4800 2700
Connection ~ 4800 2700
Wire Wire Line
	4800 2700 5000 2700
Wire Wire Line
	4800 3000 4800 2900
Connection ~ 4800 2900
Wire Wire Line
	4800 2900 5000 2900
$Comp
L HR2_MISC:Q_PWR_PMOS_GSD Q23
U 1 1 5FF1459E
P 2000 2600
F 0 "Q23" V 1900 2500 50  0000 C CNN
F 1 "PFET_6A;SOT23" V 2250 2600 50  0000 C CNN
F 2 "" H 2200 2700 50  0001 C CNN
F 3 "~" H 2000 2600 50  0001 C CNN
	1    2000 2600
	0    1    -1   0   
$EndComp
Wire Wire Line
	1700 3000 2000 3000
Wire Wire Line
	2000 3000 2000 2800
Wire Wire Line
	2200 2500 2700 2500
$EndSCHEMATC
