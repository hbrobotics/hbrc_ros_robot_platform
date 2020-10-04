EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 7 9
Title "HR2 Master Board"
Date "2020-10-03"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
NoConn ~ 3900 1950
NoConn ~ 3900 2550
NoConn ~ 3900 2950
NoConn ~ 3900 3050
NoConn ~ 3900 3150
NoConn ~ 3900 3350
NoConn ~ 3900 3450
NoConn ~ 3900 3550
NoConn ~ 3900 3650
NoConn ~ 3900 3750
NoConn ~ 3900 3950
NoConn ~ 3900 4050
NoConn ~ 3900 4750
NoConn ~ 3900 4950
NoConn ~ 3900 5050
NoConn ~ 3900 5850
NoConn ~ 3900 5650
NoConn ~ 3900 5550
NoConn ~ 3900 5450
NoConn ~ 3900 5350
NoConn ~ 3900 5150
$Comp
L Device:Crystal X1
U 1 1 5F4F5A10
P 6650 3150
F 0 "X1" H 6650 3300 50  0000 C CNN
F 1 "32.768kHZ;3.2x1.5MM" H 6650 3000 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm_HandSoldering" H 6650 3150 50  0001 C CNN
F 3 "~" H 6650 3150 50  0001 C CNN
	1    6650 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5F5097A4
P 7300 3400
F 0 "C9" H 7150 3500 50  0000 L CNN
F 1 "16pF;1608" H 7415 3355 50  0000 L CNN
F 2 "" H 7338 3250 50  0001 C CNN
F 3 "~" H 7300 3400 50  0001 C CNN
	1    7300 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5F509B6B
P 6000 3400
F 0 "C8" H 5850 3500 50  0000 L CNN
F 1 "16pF;1608" H 6115 3355 50  0000 L CNN
F 2 "" H 6038 3250 50  0001 C CNN
F 3 "~" H 6000 3400 50  0001 C CNN
	1    6000 3400
	1    0    0    -1  
$EndComp
Text Notes 5600 1500 0    50   ~ 0
Real Time Clock Notes:\n1. Real Time Clock is needed to set the operatiing system time need for\n     ROS (Robot Operating System.)\n2. The crystal load capacitor calculations are:\n     A. Cld = (C1 x C2) / (C1 + C2), where:\n          Cld = crystal load capacitence (i.e. 7-9pF)  assume 8pF\n          C1 and C2 are two capactors connected to ground, one to each crystal side.\n     B. Assume C1 = C2 and call it C.\n     C. Cld = (C1 x C2) / (C1 + C2)  = (C x C) / (C + C)  = C² / 2C = C / 2.\n     D. C = 2 ✕ Cload  = 2 ✕ 8pF = 16pF.\n
Wire Wire Line
	6800 3150 6900 3150
Wire Wire Line
	7300 3150 7300 3250
Wire Wire Line
	6500 3150 6400 3150
Wire Wire Line
	6000 3150 6000 3250
Wire Wire Line
	6000 3550 6000 3650
Wire Wire Line
	6000 3650 7300 3650
Wire Wire Line
	7300 3650 7300 3550
Wire Wire Line
	7000 2850 6900 2850
Wire Wire Line
	6900 2850 6900 3150
Connection ~ 6900 3150
Wire Wire Line
	6900 3150 7300 3150
Wire Wire Line
	6400 3150 6400 2750
Wire Wire Line
	6400 2750 7000 2750
Connection ~ 6400 3150
Wire Wire Line
	6400 3150 6000 3150
Connection ~ 6000 3650
$Comp
L Device:R_US R15
U 1 1 5F510131
P 5700 1900
F 0 "R15" H 5550 2000 50  0000 L CNN
F 1 "3.9KΩ;1608" H 5750 1800 50  0000 L CNN
F 2 "" V 5740 1890 50  0001 C CNN
F 3 "~" H 5700 1900 50  0001 C CNN
	1    5700 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R17
U 1 1 5F510F46
P 6300 1900
F 0 "R17" H 6150 2000 50  0000 L CNN
F 1 "3.9KΩ;1608" H 6350 1800 50  0000 L CNN
F 2 "" V 6340 1890 50  0001 C CNN
F 3 "~" H 6300 1900 50  0001 C CNN
	1    6300 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2150 6900 2150
Wire Wire Line
	6900 2150 6900 1650
Wire Wire Line
	6900 1650 6300 1650
Wire Wire Line
	6300 1650 6300 1750
Wire Wire Line
	7000 2350 6300 2350
Wire Wire Line
	6300 2350 6300 2050
Wire Wire Line
	7000 2450 5700 2450
Wire Wire Line
	5700 2450 5700 2150
Wire Wire Line
	6300 1650 5700 1650
Wire Wire Line
	5700 1650 5700 1750
Connection ~ 6300 1650
NoConn ~ 7000 2250
$Comp
L HR2:CAT24C32;SOIC8 U3
U 1 1 5F50901B
P 8100 5950
F 0 "U3" H 8600 5100 50  0000 R CNN
F 1 "CAT24C32;SOIC8" H 8750 6100 50  0000 R CNN
F 2 "IPC7351:SOIC127P600X175-8AN" H 8300 6000 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/CAT24C32-D.PDF" H 8300 5800 60  0001 L CNN
F 4 "https://www.onsemi.com/pub/Collateral/CAT24C32-D.PDF" H 8300 5900 60  0001 L CNN "manf#"
F 5 "32Kb I2C EEPROM" H 8300 5700 60  0001 L CNN "Field5"
	1    8100 5950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R16
U 1 1 5F50DB19
P 6100 5000
F 0 "R16" H 5950 5100 50  0000 L CNN
F 1 "4.7KΩ;1608" H 6150 4900 50  0000 L CNN
F 2 "" V 6140 4990 50  0001 C CNN
F 3 "~" H 6100 5000 50  0001 C CNN
	1    6100 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R18
U 1 1 5F50E10E
P 6700 5000
F 0 "R18" H 6550 5100 50  0000 L CNN
F 1 "100KΩ;1608" H 6750 4900 50  0000 L CNN
F 2 "" V 6740 4990 50  0001 C CNN
F 3 "~" H 6700 5000 50  0001 C CNN
	1    6700 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 5250 7300 5250
Wire Wire Line
	7300 5250 7300 4750
Wire Wire Line
	7300 4750 6700 4750
Wire Wire Line
	6100 4750 6100 4850
Wire Wire Line
	6700 4850 6700 4750
Connection ~ 6700 4750
Wire Wire Line
	6700 4750 6100 4750
$Comp
L Device:R_US R14
U 1 1 5F50FCF9
P 5500 5000
F 0 "R14" H 5350 5100 50  0000 L CNN
F 1 "4.7KΩ;1608" H 5550 4900 50  0000 L CNN
F 2 "" V 5540 4990 50  0001 C CNN
F 3 "~" H 5500 5000 50  0001 C CNN
	1    5500 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 4850 5500 4750
Wire Wire Line
	5500 4750 6100 4750
Connection ~ 6100 4750
Wire Wire Line
	7400 5350 6700 5350
Wire Wire Line
	6700 5350 6700 5150
Wire Wire Line
	7400 5450 6100 5450
Wire Wire Line
	6100 5450 6100 5150
Wire Wire Line
	7400 5550 5500 5550
Wire Wire Line
	5500 5550 5500 5150
Wire Wire Line
	7400 5650 7300 5650
Wire Wire Line
	7300 5650 7300 5750
Wire Wire Line
	7300 5950 7400 5950
Wire Wire Line
	7400 5850 7300 5850
Connection ~ 7300 5850
Wire Wire Line
	7300 5850 7300 5950
Wire Wire Line
	7400 5750 7300 5750
Connection ~ 7300 5750
Wire Wire Line
	7300 5750 7300 5850
Text Notes 6450 6500 0    50   ~ 0
EEPROM Notes:\n1. As per Raspberry PI I2C ID specification, the EEPROM\n     address is 0x50 (i.e. A0=A1=A2=0=Ground.)\n2. To enable the storage into the EEPROM s, short jumper.\n
Wire Wire Line
	3900 4550 4200 4550
Wire Wire Line
	4200 4550 4200 5550
Connection ~ 5500 5550
Wire Wire Line
	6100 5450 4300 5450
Wire Wire Line
	4300 5450 4300 4650
Wire Wire Line
	4300 4650 3900 4650
Connection ~ 6100 5450
Wire Wire Line
	3900 2450 4000 2450
Wire Wire Line
	4000 2450 4000 2750
Wire Wire Line
	3900 5250 4000 5250
Connection ~ 4000 5250
Wire Wire Line
	4000 5250 4000 5750
Wire Wire Line
	3900 4850 4000 4850
Connection ~ 4000 4850
Wire Wire Line
	4000 4850 4000 5250
Wire Wire Line
	3900 2050 4000 2050
Wire Wire Line
	4000 2050 4000 2250
Wire Wire Line
	4000 2250 3900 2250
Wire Wire Line
	7300 5750 5100 5750
Connection ~ 4000 5750
Connection ~ 4000 3650
Wire Wire Line
	4000 3650 4000 3850
Text HLabel 4500 2650 2    50   Output ~ 0
SBC_TX
Wire Wire Line
	3900 2650 4500 2650
Text Label 4100 2650 0    50   ~ 0
SBC_TX
Text HLabel 4500 2850 2    50   Output ~ 0
SBC_RX
Wire Wire Line
	3900 2850 4500 2850
Text Label 4100 2850 0    50   ~ 0
SBC_RX
Wire Wire Line
	3900 2750 4000 2750
Connection ~ 4000 2750
Wire Wire Line
	4000 2750 4000 3250
Text HLabel 4500 2050 2    50   Input ~ 0
5V
Wire Wire Line
	4000 2050 4500 2050
Connection ~ 4000 2050
Text Label 4100 2050 0    50   ~ 0
5V
Text HLabel 4500 5950 2    50   Input ~ 0
GND
Wire Wire Line
	4000 5750 4000 5950
Wire Wire Line
	4000 5950 4500 5950
Text Label 4100 5950 0    50   ~ 0
GND
Wire Wire Line
	3900 4350 4000 4350
Connection ~ 4000 4350
Wire Wire Line
	4000 4350 4000 4850
NoConn ~ 3900 4450
Wire Wire Line
	3900 3250 4000 3250
Connection ~ 4000 3250
Wire Wire Line
	4000 3250 4000 3650
Wire Wire Line
	4200 5550 5500 5550
Wire Wire Line
	7300 4750 8000 4750
Wire Wire Line
	8000 4750 8000 1650
Wire Wire Line
	8000 1650 6900 1650
Connection ~ 7300 4750
Connection ~ 6900 1650
Text HLabel 8400 1650 2    50   Input ~ 0
3.3V
Wire Wire Line
	8000 1650 8400 1650
Connection ~ 8000 1650
Text Label 7200 1650 0    50   ~ 0
3.3V
$Comp
L Device:Jumper_NO_Small JP1
U 1 1 5F563C70
P 7000 5950
F 0 "JP1" H 7000 6050 50  0000 C CNN
F 1 "JUMPER;M1x2" H 7000 5850 50  0000 C CNN
F 2 "" H 7000 5950 50  0001 C CNN
F 3 "~" H 7000 5950 50  0001 C CNN
	1    7000 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 5950 7300 5950
Connection ~ 7300 5950
Wire Wire Line
	6700 5350 6700 5950
Wire Wire Line
	6700 5950 6900 5950
Connection ~ 6700 5350
Text Notes 7450 5000 0    50   ~ 0
32Kb I2C Serial Memory
Wire Wire Line
	3900 3850 4000 3850
Connection ~ 4000 3850
Wire Wire Line
	4000 3850 4000 4350
Text HLabel 4900 4250 2    50   Output ~ 0
SBC_ALIVE
Text HLabel 4900 4150 2    50   Input ~ 0
SBC_SHUTDOWN
$Comp
L Device:R_US R19
U 1 1 5F57561C
P 4500 4900
F 0 "R19" H 4350 5000 50  0000 L CNN
F 1 "100KΩ;1608" H 4550 4800 50  0000 L CNN
F 2 "" V 4540 4890 50  0001 C CNN
F 3 "~" H 4500 4900 50  0001 C CNN
	1    4500 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 5750 4000 5750
$Comp
L HR2:RASPI;F2X20 CN57
U 1 1 5F488CF3
P 3100 1950
F 0 "CN57" H 3550 2100 50  0000 C CNN
F 1 "RASPI;F2X20" H 3550 -2100 50  0000 C CNN
F 2 "HR2:RASPI_F2X20" H 3300 2000 60  0001 L CNN
F 3 "Raspberry Pi F2x20 Connector (Row Swappped)" H 3300 1800 60  0001 L CNN
	1    3100 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4250 4500 4250
Wire Wire Line
	4500 4750 4500 4250
Connection ~ 4500 4250
Wire Wire Line
	4500 5050 4500 5750
Connection ~ 4500 5750
Wire Wire Line
	4500 5750 4000 5750
Text Notes 7050 1900 0    50   ~ 0
Real Time Clock\nwith Battery Backup
Text Label 4050 4150 0    50   ~ 0
SBC_SHUTDOWN
Text Label 4050 4250 0    50   ~ 0
SBC_ALIVE
Text Notes 1900 1550 0    50   ~ 0
SPBC Connector Notes:\n1. Any SBC that is compatible with the defacto 40-pin Raspbery Pi Standard is allowed.\n2. 5V power is provided by the 2 5V pins.\n3. The SBC_ALIVE is High (3.3V) when the operating system is up and running.\n4. The SBC_SHUTDOWN is pulled High to ask the operating system to shutdown.\n5. The SBC_TX/SBC_RX provide a dedicated connection to the Nucleo.
$Comp
L Device:R_US R20
U 1 1 5F58D7BC
P 4700 4600
F 0 "R20" H 4550 4700 50  0000 L CNN
F 1 "100KΩ;1608" H 4750 4500 50  0000 L CNN
F 2 "" V 4740 4590 50  0001 C CNN
F 3 "~" H 4700 4600 50  0001 C CNN
	1    4700 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 4750 4700 4850
Wire Wire Line
	4700 4850 5100 4850
Wire Wire Line
	5100 4850 5100 5750
Connection ~ 5100 5750
Wire Wire Line
	5100 5750 4500 5750
Text Label 7050 5750 0    50   ~ 0
GND
Text Label 6450 3650 0    50   ~ 0
GND
Text Label 7600 4750 0    50   ~ 0
3.3V
Text Label 7100 5450 0    50   ~ 0
ID_SCL
Text Label 7100 5550 0    50   ~ 0
ID_SDA
Connection ~ 5500 3650
Wire Wire Line
	5500 3650 5500 3550
Wire Wire Line
	7000 2650 5500 2650
$Comp
L Device:Battery_Cell BT1
U 1 1 5F4E022E
P 5500 3450
F 0 "BT1" H 5300 3600 50  0000 L CNN
F 1 "3V_LITH_BAT;KEYSTONE1669_CR1632" H 4100 3400 50  0000 L CNN
F 2 "" V 5500 3510 50  0001 C CNN
F 3 "~" V 5500 3510 50  0001 C CNN
	1    5500 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 2650 5500 3250
Wire Wire Line
	5500 3650 4000 3650
Wire Wire Line
	5750 2550 7000 2550
Wire Wire Line
	5750 3650 5500 3650
Connection ~ 5750 3650
Wire Wire Line
	6000 3650 5750 3650
Wire Wire Line
	5750 3650 5750 2550
Wire Wire Line
	6300 2350 3900 2350
Connection ~ 6300 2350
Wire Wire Line
	3900 2150 5700 2150
Connection ~ 5700 2150
Wire Wire Line
	5700 2150 5700 2050
Text Label 4800 2350 0    50   ~ 0
SBC_SCL
Text Label 4800 2150 0    50   ~ 0
SBC_SDA
Text Label 5550 2650 0    50   ~ 0
VBAT
Text Label 6200 3150 0    50   ~ 0
X2
Text Label 7050 3150 0    50   ~ 0
X1
Wire Wire Line
	3900 4150 4700 4150
$Comp
L HR2:MCP7940;SOIC8 U2
U 1 1 5F5D2D12
P 7700 2850
F 0 "U2" H 8200 2000 50  0000 R CNN
F 1 "MCP7940;SOIC8" H 8350 3000 50  0000 R CNN
F 2 "IPC7351:SOIC127P600X175-8AN" H 7900 2900 60  0001 L CNN
F 3 "I2C Real-time Clock" H 7900 2700 60  0001 L CNN
F 4 "https://ww1.microchip.com/downloads/en/DeviceDoc/20005010H.pdf" H 7900 2800 60  0001 L CNN "manf#"
F 5 "I2C Real Time Clock" H 7900 2600 60  0001 L CNN "Field5"
	1    7700 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 4250 4900 4250
Wire Wire Line
	4700 4450 4700 4150
Connection ~ 4700 4150
Wire Wire Line
	4700 4150 4900 4150
$EndSCHEMATC
