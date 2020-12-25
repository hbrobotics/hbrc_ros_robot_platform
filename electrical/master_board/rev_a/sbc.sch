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
NoConn ~ 3900 2900
NoConn ~ 3900 3400
NoConn ~ 3900 3500
NoConn ~ 3900 3700
NoConn ~ 3900 3800
NoConn ~ 3900 4000
NoConn ~ 3900 4100
NoConn ~ 3900 4300
NoConn ~ 3900 4400
NoConn ~ 3900 5100
NoConn ~ 3900 5300
NoConn ~ 3900 5400
NoConn ~ 3900 6000
NoConn ~ 3900 5900
NoConn ~ 3900 5800
NoConn ~ 3900 5700
NoConn ~ 3900 5500
$Comp
L Device:Crystal X1
U 1 1 5F4F5A10
P 6650 3500
AR Path="/5F4877FA/5F4F5A10" Ref="X1"  Part="1" 
AR Path="/5FA587FE/5F4F5A10" Ref="X?"  Part="1" 
F 0 "X1" H 6650 3650 50  0000 C CNN
F 1 "32.768kHz12.5pF;3.2x1.5" H 6600 3350 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_3215-2Pin_3.2x1.5mm" H 6650 3500 50  0001 C CNN
F 3 "~" H 6650 3500 50  0001 C CNN
	1    6650 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5F5097A4
P 7300 3750
AR Path="/5F4877FA/5F5097A4" Ref="C9"  Part="1" 
AR Path="/5FA587FE/5F5097A4" Ref="C?"  Part="1" 
F 0 "C9" H 7150 3850 50  0000 L CNN
F 1 "15pF;1608" H 7300 3650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7338 3600 50  0001 C CNN
F 3 "~" H 7300 3750 50  0001 C CNN
	1    7300 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5F509B6B
P 6000 3750
AR Path="/5F4877FA/5F509B6B" Ref="C7"  Part="1" 
AR Path="/5FA587FE/5F509B6B" Ref="C?"  Part="1" 
F 0 "C7" H 5850 3850 50  0000 L CNN
F 1 "15pF;1608" H 6000 3650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6038 3600 50  0001 C CNN
F 3 "~" H 6000 3750 50  0001 C CNN
	1    6000 3750
	1    0    0    -1  
$EndComp
Text Notes 5600 1800 0    50   ~ 0
Real Time Clock Notes:\n1. Reference: https://suntsu.com/crystal-load-capacitance/\n2. Real Time Clock is needed to set the operatiing system time need for\n   ROS (Robot Operating System.)\n3. The crystal load capacitor calculations are:\n   A. Cld = (C1 x C2) / (C1 + C2)  + 5pF, where:\n     5pF = stray PCB trace capacitance,\n      Cld = desired crystal load capacitence and\n      C1 and C2 are two capactors connected to ground, one on each crystal side.\n   B. Assume C1 = C2 and call it C.\n   C. Cld = (C1 x C2) / (C1 + C2) + 5pF = (C x C) / (C + C) + 5pF = C² / 2C + 5pF = C / 2 + 5pF .\n   D. Cld = C/2 + 5pF => C = 2 (Cld - 5pF)\n4. The most available crystal from JLCPCB is Q13FC1350000400 (LCSC C32346)\n   which is an Epson FC-135 32.7680KA-A5, which is a 32.7680KHz 12.5pF.\n5. Using C = 12.5pF, C = 2(Cld - 5pF) = 2(12.5pF - 5pF) = 2(7.5pF) = 15pF.\n6. Thus, C7 and C9 are 15pF.\n
Wire Wire Line
	6800 3500 6900 3500
Wire Wire Line
	7300 3500 7300 3600
Wire Wire Line
	6500 3500 6400 3500
Wire Wire Line
	6000 3500 6000 3600
Wire Wire Line
	6000 3900 6000 4000
Wire Wire Line
	6000 4000 7300 4000
Wire Wire Line
	7300 4000 7300 3900
Wire Wire Line
	7000 3200 6900 3200
Wire Wire Line
	6900 3200 6900 3500
Connection ~ 6900 3500
Wire Wire Line
	6900 3500 7300 3500
Wire Wire Line
	6400 3500 6400 3100
Wire Wire Line
	6400 3100 7000 3100
Connection ~ 6400 3500
Wire Wire Line
	6400 3500 6000 3500
Connection ~ 6000 4000
$Comp
L Device:R_US R8
U 1 1 5F510131
P 5500 2250
AR Path="/5F4877FA/5F510131" Ref="R8"  Part="1" 
AR Path="/5FA587FE/5F510131" Ref="R?"  Part="1" 
F 0 "R8" H 5350 2350 50  0000 L CNN
F 1 "3.9KΩ;1005" H 5550 2150 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5540 2240 50  0001 C CNN
F 3 "~" H 5500 2250 50  0001 C CNN
	1    5500 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R10
U 1 1 5F510F46
P 6100 2250
AR Path="/5F4877FA/5F510F46" Ref="R10"  Part="1" 
AR Path="/5FA587FE/5F510F46" Ref="R?"  Part="1" 
F 0 "R10" H 5950 2350 50  0000 L CNN
F 1 "3.9KΩ;1005" H 6150 2150 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6140 2240 50  0001 C CNN
F 3 "~" H 6100 2250 50  0001 C CNN
	1    6100 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2500 6900 2500
Wire Wire Line
	6900 2500 6900 2000
Wire Wire Line
	6100 2000 6100 2100
Wire Wire Line
	6100 2700 6100 2400
Wire Wire Line
	7000 2800 5500 2800
Wire Wire Line
	5500 2800 5500 2500
Wire Wire Line
	6100 2000 5500 2000
Wire Wire Line
	5500 2000 5500 2100
Connection ~ 6100 2000
NoConn ~ 7000 2600
$Comp
L HR2:CAT24C32;SOIC8 U3
U 1 1 5F50901B
P 9050 5600
AR Path="/5F4877FA/5F50901B" Ref="U3"  Part="1" 
AR Path="/5FA587FE/5F50901B" Ref="U?"  Part="1" 
F 0 "U3" H 9550 4750 50  0000 R CNN
F 1 "CAT24C32;SOIC8" H 9700 5750 50  0000 R CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 9250 5650 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/CAT24C32-D.PDF" H 9250 5450 60  0001 L CNN
F 4 "https://www.onsemi.com/pub/Collateral/CAT24C32-D.PDF" H 9250 5550 60  0001 L CNN "manf#"
F 5 "32Kb I2C EEPROM" H 9250 5350 60  0001 L CNN "Field5"
	1    9050 5600
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R13
U 1 1 5F50DB19
P 6900 4750
AR Path="/5F4877FA/5F50DB19" Ref="R13"  Part="1" 
AR Path="/5FA587FE/5F50DB19" Ref="R?"  Part="1" 
F 0 "R13" H 6750 4850 50  0000 L CNN
F 1 "4.7KΩ;1005" H 6950 4650 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6940 4740 50  0001 C CNN
F 3 "~" H 6900 4750 50  0001 C CNN
	1    6900 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R14
U 1 1 5F50E10E
P 7500 4750
AR Path="/5F4877FA/5F50E10E" Ref="R14"  Part="1" 
AR Path="/5FA587FE/5F50E10E" Ref="R?"  Part="1" 
F 0 "R14" H 7350 4850 50  0000 L CNN
F 1 "100KΩ;1005" H 7550 4650 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 7540 4740 50  0001 C CNN
F 3 "~" H 7500 4750 50  0001 C CNN
	1    7500 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 4900 8100 4900
Wire Wire Line
	8100 4900 8100 4500
Wire Wire Line
	8100 4500 7500 4500
Wire Wire Line
	6900 4500 6900 4600
Wire Wire Line
	7500 4600 7500 4500
Connection ~ 7500 4500
Wire Wire Line
	7500 4500 7300 4500
$Comp
L Device:R_US R12
U 1 1 5F50FCF9
P 6300 4750
AR Path="/5F4877FA/5F50FCF9" Ref="R12"  Part="1" 
AR Path="/5FA587FE/5F50FCF9" Ref="R?"  Part="1" 
F 0 "R12" H 6150 4850 50  0000 L CNN
F 1 "4.7KΩ;1005" H 6350 4650 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6340 4740 50  0001 C CNN
F 3 "~" H 6300 4750 50  0001 C CNN
	1    6300 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 4500 6900 4500
Connection ~ 6900 4500
Wire Wire Line
	8350 5000 7500 5000
Wire Wire Line
	7500 5000 7500 4900
Wire Wire Line
	8350 5100 6900 5100
Wire Wire Line
	6900 5100 6900 4900
Wire Wire Line
	8350 5200 6300 5200
Wire Wire Line
	6300 5200 6300 4900
Wire Wire Line
	8350 5300 8000 5300
Wire Wire Line
	8000 5300 8000 5400
Wire Wire Line
	8000 5600 8350 5600
Wire Wire Line
	8350 5500 8000 5500
Connection ~ 8000 5500
Wire Wire Line
	8000 5500 8000 5600
Wire Wire Line
	8350 5400 8000 5400
Connection ~ 8000 5400
Wire Wire Line
	8000 5400 8000 5500
Text Notes 8200 3950 0    50   ~ 0
EEPROM Notes:\n1. As per Raspberry PI I2C ID specification, the EEPROM\n     address is 0x50 (i.e. A0=A1=A2=0=Ground.)\n2. To enable the storage into the EEPROM s, short jumper.\n
Wire Wire Line
	3900 4900 4200 4900
Connection ~ 6300 5200
Wire Wire Line
	6900 5100 4300 5100
Wire Wire Line
	4300 5100 4300 5000
Wire Wire Line
	4300 5000 3900 5000
Connection ~ 6900 5100
Wire Wire Line
	3900 2800 4000 2800
Wire Wire Line
	4000 2800 4000 3100
Wire Wire Line
	3900 5600 4000 5600
Connection ~ 4000 5600
Wire Wire Line
	4000 5600 4000 6100
Wire Wire Line
	3900 5200 4000 5200
Connection ~ 4000 5200
Wire Wire Line
	4000 5200 4000 5600
Wire Wire Line
	3900 2400 4000 2400
Wire Wire Line
	4000 2400 4000 2600
Wire Wire Line
	4000 2600 3900 2600
Connection ~ 4000 6100
Connection ~ 4000 4000
Wire Wire Line
	4000 4000 4000 4200
Text HLabel 5000 3000 2    50   Output ~ 0
SBC_TX
Wire Wire Line
	3900 3000 4800 3000
Text HLabel 5000 3200 2    50   Output ~ 0
SBC_RX
Wire Wire Line
	3900 3200 4800 3200
Wire Wire Line
	3900 3100 4000 3100
Connection ~ 4000 3100
Wire Wire Line
	4000 3100 4000 3600
Text HLabel 5000 2400 2    50   Input ~ 0
5V
Wire Wire Line
	4000 2400 5000 2400
Connection ~ 4000 2400
Text HLabel 8400 6600 2    50   Input ~ 0
GND
Wire Wire Line
	4000 6100 4000 6600
Text Label 4850 6100 0    50   ~ 0
GND
Wire Wire Line
	3900 4700 4000 4700
Connection ~ 4000 4700
Wire Wire Line
	4000 4700 4000 5200
NoConn ~ 3900 4800
Wire Wire Line
	3900 3600 4000 3600
Connection ~ 4000 3600
Wire Wire Line
	4000 3600 4000 4000
Wire Wire Line
	8100 2000 6900 2000
Connection ~ 8100 4500
Connection ~ 6900 2000
Text HLabel 8400 2000 2    50   Input ~ 0
3.3V
Wire Wire Line
	8100 2000 8400 2000
Connection ~ 8100 2000
Text Label 8400 2000 2    50   ~ 0
3.3V
$Comp
L Device:Jumper_NO_Small JP1
U 1 1 5F563C70
P 7800 5400
AR Path="/5F4877FA/5F563C70" Ref="JP1"  Part="1" 
AR Path="/5FA587FE/5F563C70" Ref="JP?"  Part="1" 
F 0 "JP1" H 7800 5500 50  0000 C CNN
F 1 "JUMPER;M1x2" H 7650 5300 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7800 5400 50  0001 C CNN
F 3 "~" H 7800 5400 50  0001 C CNN
	1    7800 5400
	1    0    0    -1  
$EndComp
Connection ~ 8000 5600
Wire Wire Line
	7500 5000 7500 5400
Wire Wire Line
	7500 5400 7700 5400
Connection ~ 7500 5000
Text Notes 8250 4450 0    50   ~ 0
32Kb I2C Serial Memory\nUsed for Raspberry Pi Discovery
Wire Wire Line
	3900 4200 4000 4200
Connection ~ 4000 4200
Wire Wire Line
	4000 4200 4000 4700
Text HLabel 8400 5900 2    50   Output ~ 0
SBC_ALIVE
Wire Wire Line
	3900 6100 4000 6100
$Comp
L HR2:RASPI;F2X20 CN94
U 1 1 5F488CF3
P 3100 2300
AR Path="/5F4877FA/5F488CF3" Ref="CN94"  Part="1" 
AR Path="/5FA587FE/5F488CF3" Ref="CN?"  Part="1" 
F 0 "CN94" H 3550 2450 50  0000 C CNN
F 1 "RASPI;F2X20" H 3550 -1750 50  0000 C CNN
F 2 "HR2:RASPI_F2X20" H 3300 2350 60  0001 L CNN
F 3 "Raspberry Pi F2x20 Connector (Row Swappped)" H 3300 2150 60  0001 L CNN
	1    3100 2300
	1    0    0    -1  
$EndComp
Text Notes 7050 2250 0    50   ~ 0
Real Time Clock\nwith Battery Backup
Text Label 4050 4500 0    50   ~ 0
SBC_STOP
Text Label 8400 5900 2    50   ~ 0
SBC_ALIVE
Text Notes 1900 1450 0    50   ~ 0
SBC (Single Board Computer) Connector Notes:\n1. Any SBC that is compatible with the defacto 40-pin Raspbery Pi Standard is allowed.\n2. 5V power is provided by the 2 5V pins.\n3. The SBC_ALIVE is High (3.3V) when the operating system is up and running.\n4. The SBC_SHUTDOWN is pulled High to ask the operating system to shutdown.\n5. The SBC_TX/SBC_RX provide a dedicated serial connection to the Nucleo.
$Comp
L Device:R_US R6
U 1 1 5F58D7BC
P 4600 5650
AR Path="/5F4877FA/5F58D7BC" Ref="R6"  Part="1" 
AR Path="/5FA587FE/5F58D7BC" Ref="R?"  Part="1" 
F 0 "R6" H 4450 5750 50  0000 L CNN
F 1 "100KΩ;1005" H 4650 5550 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4640 5640 50  0001 C CNN
F 3 "~" H 4600 5650 50  0001 C CNN
	1    4600 5650
	1    0    0    -1  
$EndComp
Connection ~ 4600 6100
Text Label 8400 6600 2    50   ~ 0
GND
Text Label 4500 4000 0    50   ~ 0
GND
Text Label 7600 4500 0    50   ~ 0
3.3V
Text Label 8350 5100 2    50   ~ 0
ID_SCL
Text Label 8350 5200 2    50   ~ 0
ID_SDA
Connection ~ 5500 4000
Wire Wire Line
	5500 4000 5500 3900
Wire Wire Line
	7000 3000 5500 3000
$Comp
L Device:Battery_Cell BT1
U 1 1 5F4E022E
P 5500 3800
AR Path="/5F4877FA/5F4E022E" Ref="BT1"  Part="1" 
AR Path="/5FA587FE/5F4E022E" Ref="BT?"  Part="1" 
F 0 "BT1" H 5300 3950 50  0000 L CNN
F 1 "COIN_HOLDER6.8R;TE1376164-1" H 4250 3750 50  0000 L CNN
F 2 "HR2:TE1376164-1_COINHOLDER6.8" V 5500 3860 50  0001 C CNN
F 3 "~" V 5500 3860 50  0001 C CNN
	1    5500 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3000 5500 3600
Wire Wire Line
	5500 4000 4000 4000
Wire Wire Line
	5750 2900 6700 2900
Wire Wire Line
	5750 4000 5500 4000
Connection ~ 5750 4000
Wire Wire Line
	6000 4000 5750 4000
Wire Wire Line
	5750 4000 5750 2900
Wire Wire Line
	6100 2700 3900 2700
Connection ~ 6100 2700
Wire Wire Line
	3900 2500 5500 2500
Connection ~ 5500 2500
Wire Wire Line
	5500 2500 5500 2400
Text Label 4100 2700 0    50   ~ 0
SBC_SCL
Text Label 4100 2500 0    50   ~ 0
SBC_SDA
Text Label 5550 3000 0    50   ~ 0
VBAT
Text Label 6900 3100 0    50   ~ 0
X2
Text Label 6900 3200 0    50   ~ 0
X1
Wire Wire Line
	3900 4500 4600 4500
$Comp
L HR2:MCP7940;SOIC8 U2
U 1 1 5F5D2D12
P 7700 3200
AR Path="/5F4877FA/5F5D2D12" Ref="U2"  Part="1" 
AR Path="/5FA587FE/5F5D2D12" Ref="U?"  Part="1" 
F 0 "U2" H 8200 2350 50  0000 R CNN
F 1 "MCP7940;SOIC8" H 8350 3350 50  0000 R CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7900 3250 60  0001 L CNN
F 3 "I2C Real-time Clock" H 7900 3050 60  0001 L CNN
F 4 "https://ww1.microchip.com/downloads/en/DeviceDoc/20005010H.pdf" H 7900 3150 60  0001 L CNN "manf#"
F 5 "I2C Real Time Clock" H 7900 2950 60  0001 L CNN "Field5"
	1    7700 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	4600 5500 4600 4500
Connection ~ 4600 4500
$Comp
L Switch:SW_Push SW?
U 1 1 5F8FBAFA
P 5900 4500
AR Path="/5F52F39E/5F8FBAFA" Ref="SW?"  Part="1" 
AR Path="/5F4877FA/5F8FBAFA" Ref="SW2"  Part="1" 
F 0 "SW2" H 5750 4600 50  0000 C CNN
F 1 "BUTTON;6x6" H 5900 4400 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 5900 4700 50  0001 C CNN
F 3 "~" H 5900 4700 50  0001 C CNN
	1    5900 4500
	1    0    0    -1  
$EndComp
Text Label 4100 3200 0    50   ~ 0
SBC_RX
Text Label 4100 3000 0    50   ~ 0
SBC_TX
Text Label 4100 2400 0    50   ~ 0
5V
NoConn ~ 3900 2300
Wire Wire Line
	4600 4500 4750 4500
$Comp
L Device:R_US R11
U 1 1 5FAF8B92
P 4900 4500
AR Path="/5F4877FA/5FAF8B92" Ref="R11"  Part="1" 
AR Path="/5FA587FE/5FAF8B92" Ref="R?"  Part="1" 
F 0 "R11" V 4800 4450 50  0000 L CNN
F 1 "1KΩ;1005" V 5000 4350 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4940 4490 50  0001 C CNN
F 3 "~" H 4900 4500 50  0001 C CNN
	1    4900 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	5050 4500 5700 4500
$Comp
L Device:R_US R9
U 1 1 5FB254F3
P 5500 5450
AR Path="/5F4877FA/5FB254F3" Ref="R9"  Part="1" 
AR Path="/5FA587FE/5FB254F3" Ref="R?"  Part="1" 
F 0 "R9" H 5350 5550 50  0000 L CNN
F 1 "33Ω;1005" H 5550 5350 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5540 5440 50  0001 C CNN
F 3 "~" H 5500 5450 50  0001 C CNN
	1    5500 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 4600 4500 4900
Wire Wire Line
	4500 4900 5300 4900
Wire Wire Line
	5500 4900 5500 5300
Wire Wire Line
	5500 6100 5300 6100
Text Notes 4850 4300 0    50   ~ 0
SBC Shutdown Request Button
Text Notes 5600 5500 0    50   ~ 0
SBC Alive LED
Wire Wire Line
	3900 4600 4500 4600
Wire Wire Line
	4000 6100 4600 6100
Wire Wire Line
	4600 5800 4600 6100
NoConn ~ 3900 3900
Text Notes 3250 2100 0    50   ~ 0
SBC Connector
$Comp
L Device:R_US R7
U 1 1 5F917643
P 5300 5450
AR Path="/5F4877FA/5F917643" Ref="R7"  Part="1" 
AR Path="/5FA587FE/5F917643" Ref="R?"  Part="1" 
F 0 "R7" H 5150 5550 50  0000 L CNN
F 1 "100KΩ;1005" H 4800 5300 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5340 5440 50  0001 C CNN
F 3 "~" H 5300 5450 50  0001 C CNN
	1    5300 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5300 5300 4900
Connection ~ 5300 4900
Wire Wire Line
	5300 4900 5500 4900
$Comp
L Device:C C10
U 1 1 5FA44A8F
P 7300 4250
AR Path="/5F4877FA/5FA44A8F" Ref="C10"  Part="1" 
AR Path="/5FA587FE/5FA44A8F" Ref="C?"  Part="1" 
F 0 "C10" H 7150 4350 50  0000 L CNN
F 1 "0.1µF;1005" H 7300 4150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7338 4100 50  0001 C CNN
F 3 "~" H 7300 4250 50  0001 C CNN
	1    7300 4250
	1    0    0    -1  
$EndComp
Connection ~ 7300 4500
Wire Wire Line
	7300 4500 6900 4500
Wire Wire Line
	7300 4100 7300 4000
Connection ~ 7300 4000
Text Notes 8250 3350 0    50   ~ 0
Example Crystal Part:\nCM7V-T1A-LOW-ESR-32.768KHZ-9PF-20-TA-QC
Text Label 8350 5000 2    50   ~ 0
WP
Wire Wire Line
	6700 2000 6100 2000
Wire Wire Line
	6900 2000 6700 2000
Connection ~ 6700 2000
Wire Wire Line
	6700 2300 6700 2000
Wire Wire Line
	8100 4500 8100 2000
$Comp
L Device:C C8
U 1 1 5FA24EA3
P 6700 2450
AR Path="/5F4877FA/5FA24EA3" Ref="C8"  Part="1" 
AR Path="/5FA587FE/5FA24EA3" Ref="C?"  Part="1" 
F 0 "C8" H 6700 2550 50  0000 L CNN
F 1 "0.1µF;1005" H 6250 2350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6738 2300 50  0001 C CNN
F 3 "~" H 6700 2450 50  0001 C CNN
	1    6700 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2700 7000 2700
Wire Wire Line
	6700 2600 6700 2900
Connection ~ 6700 2900
Wire Wire Line
	6700 2900 7000 2900
Connection ~ 5500 4900
Wire Wire Line
	6300 4500 6300 4600
Wire Wire Line
	6100 4500 6300 4500
Connection ~ 6300 4500
Wire Wire Line
	7300 4400 7300 4500
Connection ~ 5300 6100
Wire Wire Line
	5300 6100 4600 6100
Text Notes 6700 5600 0    50   ~ 0
EEPROM Program Enable Jumper
Wire Wire Line
	5300 5600 5300 6100
Text Label 5100 4500 0    50   ~ 0
SBC_REQ
$Comp
L Connector:TestPoint TP16
U 1 1 5FB4ED7C
P 4800 2900
F 0 "TP16" H 4850 3050 50  0000 L CNN
F 1 "SPC_TX;M1x2" H 4858 2927 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 5000 2900 50  0001 C CNN
F 3 "~" H 5000 2900 50  0001 C CNN
	1    4800 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2400 4000 1900
$Comp
L Jumper:Jumper_3_Open JP9
U 1 1 5FB67673
P 750 3700
F 0 "JP9" V 800 3550 50  0000 C CNN
F 1 "RPI_DISP_EN;M1x3" V 650 3300 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 750 3700 50  0001 C CNN
F 3 "~" H 750 3700 50  0001 C CNN
	1    750  3700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R65
U 1 1 5FB68DCD
P 750 2050
AR Path="/5F4877FA/5FB68DCD" Ref="R65"  Part="1" 
AR Path="/5FA587FE/5FB68DCD" Ref="R?"  Part="1" 
F 0 "R65" H 600 2150 50  0000 L CNN
F 1 "100KΩ;1608" H 750 1950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 790 2040 50  0001 C CNN
F 3 "~" H 750 2050 50  0001 C CNN
	1    750  2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R67
U 1 1 5FB84C0D
P 1700 3050
AR Path="/5F4877FA/5FB84C0D" Ref="R67"  Part="1" 
AR Path="/5FA587FE/5FB84C0D" Ref="R?"  Part="1" 
F 0 "R67" H 1500 3150 50  0000 L CNN
F 1 "100KΩ;1005" H 1200 2900 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 1740 3040 50  0001 C CNN
F 3 "~" H 1700 3050 50  0001 C CNN
	1    1700 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 CN3
U 1 1 5FB885F0
P 2900 2800
F 0 "CN3" H 2800 2900 50  0000 L CNN
F 1 "RPI_DSP_PWR;M1x2" H 2550 2600 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 2900 2800 50  0001 C CNN
F 3 "~" H 2900 2800 50  0001 C CNN
	1    2900 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  3450 750  2200
Wire Wire Line
	1400 3700 1100 3700
Wire Wire Line
	2500 2900 2700 2900
Wire Wire Line
	1800 2800 1700 2800
Wire Wire Line
	1700 1900 4000 1900
Wire Wire Line
	750  4500 1700 4500
Wire Wire Line
	1700 3900 1700 4500
Connection ~ 1700 4500
Wire Wire Line
	1700 4500 2500 4500
Wire Wire Line
	4000 6600 2500 6600
Wire Wire Line
	2500 6600 2500 4500
Connection ~ 4000 6600
Connection ~ 2500 4500
Wire Wire Line
	5500 2000 5500 1700
Wire Wire Line
	5500 1700 750  1700
Connection ~ 5500 2000
Wire Wire Line
	750  1700 750  1900
Wire Wire Line
	1700 2900 1700 2800
Connection ~ 1700 2800
Wire Wire Line
	1700 1900 1700 2800
Wire Wire Line
	1700 3200 1700 3300
Connection ~ 1700 3300
Wire Wire Line
	1700 3300 1700 3500
Wire Wire Line
	750  4400 750  4500
Wire Wire Line
	750  4100 750  3950
$Comp
L Device:R_US R66
U 1 1 5FB69F2A
P 750 4250
AR Path="/5F4877FA/5FB69F2A" Ref="R66"  Part="1" 
AR Path="/5FA587FE/5FB69F2A" Ref="R?"  Part="1" 
F 0 "R66" H 600 4350 50  0000 L CNN
F 1 "100KΩ;1005" H 800 4150 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 790 4240 50  0001 C CNN
F 3 "~" H 750 4250 50  0001 C CNN
	1    750  4250
	1    0    0    -1  
$EndComp
Text Notes 1750 2550 0    50   ~ 0
Raspberry Pi Display\nPower Enable Circuit
Wire Wire Line
	1100 3700 1100 2800
Wire Wire Line
	1100 2800 1300 2800
Wire Wire Line
	1300 2800 1300 1800
Wire Wire Line
	1300 1800 4600 1800
Wire Wire Line
	4600 1800 4600 3300
Wire Wire Line
	4600 3300 3900 3300
Connection ~ 1100 3700
Wire Wire Line
	1100 3700 900  3700
Text Notes 500  4950 0    50   ~ 0
Notes:\n1. Jumper controlls default display Power On/Off.\n2. Transistor shifts from 3.3V to 5V Gate On/Off.\n3. Jumper provides switched 5V to Ras Pi Display.\n4. Raspberry Pi can override jumper.
Text Notes 800  4000 0    50   ~ 0
ON
Text Notes 800  3450 0    50   ~ 0
OFF
Text Label 4100 3300 0    50   ~ 0
RPI_DSP_EN
$Comp
L Connector:TestPoint TP17
U 1 1 5FC4D458
P 4800 3300
F 0 "TP17" H 4600 3500 50  0000 L CNN
F 1 "SBC_RX;M1x2" H 4250 3350 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 5000 3300 50  0001 C CNN
F 3 "~" H 5000 3300 50  0001 C CNN
	1    4800 3300
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 2900 4800 3000
Connection ~ 4800 3000
Wire Wire Line
	4800 3000 5000 3000
Wire Wire Line
	4800 3300 4800 3200
Connection ~ 4800 3200
Wire Wire Line
	4800 3200 5000 3200
$Comp
L HR2_MISC:Q_PWR_PMOS_GSD Q23
U 1 1 5FF1459E
P 2000 2900
F 0 "Q23" V 1900 2800 50  0000 C CNN
F 1 "PFET_6A_GSD;SOT23" V 2250 3000 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2200 3000 50  0001 C CNN
F 3 "~" H 2000 2900 50  0001 C CNN
	1    2000 2900
	0    1    -1   0   
$EndComp
Wire Wire Line
	1700 3300 2000 3300
Wire Wire Line
	2000 3300 2000 3100
Wire Wire Line
	2200 2800 2700 2800
Wire Wire Line
	2500 2900 2500 4500
$Comp
L Transistor_FET:2N7002 Q24
U 1 1 5FE943A9
P 1600 3700
F 0 "Q24" H 1450 3850 50  0000 L CNN
F 1 "2N7002;SOT23" H 1750 3600 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1800 3625 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 1600 3700 50  0001 L CNN
	1    1600 3700
	1    0    0    -1  
$EndComp
Text HLabel 8400 6200 2    50   Input ~ 0
~NESTOP
Wire Wire Line
	5500 5700 5500 5600
Wire Wire Line
	5500 6000 5500 6100
$Comp
L Device:LED D?
U 1 1 5FB21992
P 5500 5850
AR Path="/5F79BC00/5FB21992" Ref="D?"  Part="1" 
AR Path="/5FA4A874/5FB21992" Ref="D?"  Part="1" 
AR Path="/5F4877FA/5FB21992" Ref="D2"  Part="1" 
F 0 "D2" V 5600 5950 50  0000 R CNN
F 1 "LEDGRN;1608" V 5400 5750 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5500 5850 50  0001 C CNN
F 3 "~" H 5500 5850 50  0001 C CNN
	1    5500 5850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4200 5200 6300 5200
Wire Wire Line
	4200 4900 4200 5200
Wire Wire Line
	8000 5600 8000 6600
Wire Wire Line
	4000 6600 8000 6600
Connection ~ 8000 6600
Wire Wire Line
	8000 6600 8400 6600
Wire Wire Line
	7900 5400 8000 5400
Wire Wire Line
	8400 5900 6200 5900
Wire Wire Line
	6200 5900 6200 4900
Wire Wire Line
	5500 4900 6200 4900
Wire Wire Line
	3900 6200 8400 6200
Text Label 8400 6200 2    50   ~ 0
~NESTOP
$EndSCHEMATC
