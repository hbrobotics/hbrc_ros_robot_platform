EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 10 10
Title "HR2: WOWBus"
Date "2020-11-22"
Rev "A"
Comp "HomeBrew Robotics Club"
Comment1 "Copyright © 2020 by HomeBrew Robotics Club "
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R_US R59
U 1 1 5F846FDA
P 7400 2150
F 0 "R59" H 7250 2250 50  0000 L CNN
F 1 "120Ω.25W;1608" H 7450 2050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7440 2140 50  0001 C CNN
F 3 "~" H 7400 2150 50  0001 C CNN
	1    7400 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 2400 7200 2000
Wire Wire Line
	7400 2000 7400 1900
Wire Wire Line
	7400 2500 7400 2300
$Comp
L Device:R_US R55
U 1 1 5F869783
P 3900 1750
F 0 "R55" H 3750 1850 50  0000 L CNN
F 1 "100KΩ;1005" H 3950 1650 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3940 1740 50  0001 C CNN
F 3 "~" H 3900 1750 50  0001 C CNN
	1    3900 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1100 3900 1100
Wire Wire Line
	3900 1100 3900 1600
Wire Wire Line
	3900 2000 4500 2000
Text HLabel 1200 1100 0    50   Input ~ 0
9V
Wire Wire Line
	1200 1100 3900 1100
Connection ~ 3900 1100
Text HLabel 1200 6100 0    50   Input ~ 0
GND
Text HLabel 1200 5100 0    50   Input ~ 0
WOW_EN
$Comp
L Device:R_US R57
U 1 1 5F873ADD
P 4700 3450
F 0 "R57" H 4550 3550 50  0000 L CNN
F 1 "100KΩ;1608" H 4750 3350 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4740 3440 50  0001 C CNN
F 3 "~" H 4700 3450 50  0001 C CNN
	1    4700 3450
	1    0    0    -1  
$EndComp
$Comp
L HR2:WOW_OUT;M2x6S CN10
U 1 1 5F848F57
P 9700 4800
F 0 "CN10" H 9900 4950 50  0000 L CNN
F 1 "WOW_OUT;M2x6S" H 9850 3550 50  0000 L CNN
F 2 "HR2:PinHeader_2x06_P2.54mm_Vertical_Shrouded" H 9900 4850 60  0001 L CNN
F 3 "WOW Output Connector" H 9900 4650 60  0001 L CNN
F 4 "WOW Output Connector" H 9900 4550 60  0001 L CNN "Field5"
	1    9700 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4400 5500 4400
$Comp
L Device:R_US R60
U 1 1 5F84B29E
P 7400 3850
F 0 "R60" H 7250 3950 50  0000 L CNN
F 1 "120Ω.25W;1608" H 7450 3750 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7440 3840 50  0001 C CNN
F 3 "~" H 7400 3850 50  0001 C CNN
	1    7400 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 4100 7200 4100
Wire Wire Line
	6600 4200 7400 4200
Wire Wire Line
	7400 4200 7400 4000
Wire Wire Line
	7200 4100 7200 3400
Wire Wire Line
	7400 3500 7500 3500
Wire Wire Line
	7200 4100 9000 4100
Connection ~ 7200 4100
Wire Wire Line
	7400 4200 8900 4200
Connection ~ 7400 4200
Connection ~ 7400 2500
Wire Wire Line
	4500 1400 4500 2000
Text HLabel 1200 3200 0    50   Input ~ 0
3.3V
Wire Wire Line
	6600 4400 6900 4400
Text HLabel 1200 1500 0    50   Input ~ 0
5V
Wire Wire Line
	1200 1500 6700 1500
Wire Wire Line
	6700 1500 6700 2200
Wire Wire Line
	6600 4300 6700 4300
Wire Wire Line
	6700 4300 6700 2200
Connection ~ 6700 2200
Wire Wire Line
	6600 4500 7000 4500
Connection ~ 6900 3200
Wire Wire Line
	6900 3200 6900 2300
Wire Wire Line
	7000 6100 7000 5500
Connection ~ 7000 4500
Connection ~ 4700 6100
$Comp
L Device:R_US R56
U 1 1 5F98DD9D
P 2000 5750
F 0 "R56" H 1850 5850 50  0000 L CNN
F 1 "100KΩ;1005" H 1500 5650 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 2040 5740 50  0001 C CNN
F 3 "~" H 2000 5750 50  0001 C CNN
	1    2000 5750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F9A0944
P 3800 4200
AR Path="/5F79BC00/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5F9A0944" Ref="Q20"  Part="1" 
F 0 "Q20" H 3700 4350 50  0000 L CNN
F 1 "2N7000;SOT23" H 3950 4050 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4000 4125 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 3800 4200 50  0001 L CNN
	1    3800 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 2000 3900 4000
Text HLabel 1200 2100 0    50   Input ~ 0
WOW_TX
Text HLabel 1200 2200 0    50   Output ~ 0
WOW_RX
Wire Wire Line
	1200 2200 2600 2200
Wire Wire Line
	9500 1800 9500 1900
$Comp
L Device:R_US R61
U 1 1 5F8EAE9D
P 9500 2050
F 0 "R61" H 9350 2150 50  0000 L CNN
F 1 "100KΩ;1608" H 9000 1900 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9540 2040 50  0001 C CNN
F 3 "~" H 9500 2050 50  0001 C CNN
	1    9500 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 2200 9500 2900
$Comp
L HR2:74LVC1G74;TSSOP8 U16
U 1 1 5F8C5AF5
P 5500 5200
F 0 "U16" H 5750 5350 50  0000 C CNN
F 1 "74LVC1G74;TSSOP8" H 6050 4750 50  0000 C CNN
F 2 "Package_SO:TSSOP-8_3x3mm_P0.65mm" H 5700 5250 60  0001 L CNN
F 3 "D-type flip-flop edge-trigger/set/reset" H 5700 5050 60  0001 L CNN
F 4 "D-type flip-flop edge-trigger/set/reset" H 5700 4950 60  0001 L CNN "Field5"
	1    5500 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5200 6600 5200
Wire Wire Line
	6600 5500 7000 5500
Connection ~ 7000 5500
Wire Wire Line
	7000 5500 7000 4500
Text HLabel 1200 5500 0    50   Input ~ 0
ESTOP_CLR
Text HLabel 1200 5300 0    50   Input ~ 0
~NRST
Wire Wire Line
	5400 5300 5500 5300
Text Notes 7300 5800 0    50   ~ 0
Red ESTOP LED
Text Notes 5350 1750 0    50   ~ 0
Communication CANBus Transceiver
Text Notes 5650 3900 0    50   ~ 0
ESTOP CANBus Transceiver
Text Notes 3800 850  0    50   ~ 0
WOW Power P-FET (6A)
Text Notes 5500 5000 0    50   ~ 0
E-Stop x74 D Style Flip-Flop
Text Notes 7050 1300 0    50   ~ 0
Opto-isolator
Text Notes 7750 3200 0    50   ~ 0
Solid State Relay
Text Notes 9400 6300 0    50   ~ 0
WOWBus Output Connector
Wire Wire Line
	3500 4200 3600 4200
$Comp
L Device:R_US R53
U 1 1 5FC61F4D
P 2800 4350
F 0 "R53" H 2650 4450 50  0000 L CNN
F 1 "100KΩ;1005" H 2300 4200 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 2840 4340 50  0001 C CNN
F 3 "~" H 2800 4350 50  0001 C CNN
	1    2800 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4200 2800 4100
$Comp
L Device:LED D?
U 1 1 5FC8C292
P 7200 5650
AR Path="/5F79BC00/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5FA4A874/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5FAD82EE/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5F8C2F57/5FC8C292" Ref="D3"  Part="1" 
F 0 "D3" V 7300 5750 50  0000 R CNN
F 1 "REDLED;1608" V 7150 5550 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7200 5650 50  0001 C CNN
F 3 "~" H 7200 5650 50  0001 C CNN
	1    7200 5650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R58
U 1 1 5FC946D0
P 7200 5250
F 0 "R58" H 7050 5350 50  0000 L CNN
F 1 "470Ω;1608" H 7250 5150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7240 5240 50  0001 C CNN
F 3 "~" H 7200 5250 50  0001 C CNN
	1    7200 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 6100 7200 6100
Wire Wire Line
	7200 6100 7200 5800
Connection ~ 7000 6100
Text Label 8900 2400 0    50   ~ 0
WOW_COMH
Text Label 8900 2500 0    50   ~ 0
WOW_COML
Text Label 7700 4100 0    50   ~ 0
WOW_STOPH
Text Label 7700 4200 0    50   ~ 0
WOW_STOPL
Text Label 1200 1100 0    50   ~ 0
9V
Text Label 1200 1500 0    50   ~ 0
5V
Text Label 1200 2100 0    50   ~ 0
WOW_TX
Text Label 1200 2200 0    50   ~ 0
WOW_RX
Text Label 1200 3200 0    50   ~ 0
3.3V
Text Label 1200 5100 0    50   ~ 0
WOW_EN
Text Label 1200 5500 0    50   ~ 0
ESTOP_CLR
Text Label 1200 5300 0    50   ~ 0
~NRST
Text Label 1200 6100 0    50   ~ 0
GND
Text Label 5400 4800 0    50   ~ 0
~NHR2_ESTOP
Text Label 4750 4400 0    50   ~ 0
WOW_STDBY
Connection ~ 3500 4700
Text Label 6900 1100 0    50   ~ 0
WOW_POW_OUT
Text Label 8800 3500 0    50   ~ 0
WOW_ESTOP_OPTO
Text Label 8900 1800 0    50   ~ 0
WOW_COM_OPTO
Text Notes 800  7900 0    50   ~ 0
WOWBus Notes:\n\n1. WOWBus is an an expansion bus.\n2. WOWBus stands for Wayne's Omnipotent Wonderful Bus (heavy sarcasm.)\n3. WOWBus uses 2x6 ribbon daisy chain cables for interconnect.\n4. WOWBus has a nominal 9V rail, but this can droop due to resistive losses down the daisy chain.\n5. The WOWBus connector is designed to mirror image power, ground, and signaling.\n6. If a WOWBus connector is accidently reversed on a daisy cable, nothing shorts out.\n7. However, an accidentally reversed cable, will basically make the bus non-fuctional until corrected.\n8. The WOWBus has two "CAN bus" (ISO-11898-2) physical differential pairs for siginalling.\n9. The first differential pair is used as a multi-drop bidirectional data transfer.\n10. The second differential pair is used to signal Emergency Stop.\n11. When the WOWBus is enabled, it powers up both the transceivers and the differential\n    pair termination resistors.\n12. The expansions boards are responsible for providing the bus terminator resistors for\n    for the other end of the differential pairs.\n13. The Bus powers up in the EStop conndition and must be explicitly cleared out.\n14. When ESTOP occurs, it latches up and must be explicitly cleared.\n
Connection ~ 5300 4400
Wire Wire Line
	5400 4800 5400 4300
Wire Wire Line
	5400 4300 5500 4300
Wire Wire Line
	4700 3600 4700 4400
Connection ~ 4700 4400
Wire Wire Line
	4700 4400 5300 4400
Text HLabel 1200 5200 0    50   Input ~ 0
~NMOTOR_FAULT
Text Label 1200 5200 0    50   ~ 0
~NMOTOR_FAULT
Wire Wire Line
	5400 4900 5400 5200
Wire Wire Line
	6700 4900 6700 5200
Wire Wire Line
	6700 4900 5400 4900
Text Label 4200 5500 0    50   ~ 0
~NESTOP_SET
Connection ~ 2800 5200
Wire Wire Line
	2800 4500 2800 5200
Wire Wire Line
	1200 5200 2800 5200
Wire Wire Line
	2800 5200 3200 5200
Wire Wire Line
	7200 5400 7200 5500
Wire Wire Line
	4700 6100 7000 6100
Wire Wire Line
	4700 4400 4700 4500
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F86962A
P 4600 4700
AR Path="/5F79BC00/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5F86962A" Ref="Q22"  Part="1" 
F 0 "Q22" H 4500 4850 50  0000 L CNN
F 1 "2N7000;SOT23" H 4100 4500 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4800 4625 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 4600 4700 50  0001 L CNN
	1    4600 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3200 6900 3200
Connection ~ 4700 3200
Wire Wire Line
	4700 3200 4700 3300
Wire Wire Line
	6900 3200 6900 4400
$Comp
L Switch:SW_Push SW?
U 1 1 5FC73CCB
P 2500 5900
AR Path="/5F52F39E/5FC73CCB" Ref="SW?"  Part="1" 
AR Path="/5F8C2F57/5FC73CCB" Ref="SW3"  Part="1" 
F 0 "SW3" H 2350 6000 50  0000 C CNN
F 1 "BUTTON;6x3.5" H 2500 5800 50  0000 C CNN
F 2 "HR2:BUTTON_6x3.5" H 2500 6100 50  0001 C CNN
F 3 "~" H 2500 6100 50  0001 C CNN
	1    2500 5900
	1    0    0    -1  
$EndComp
Connection ~ 3900 6100
$Comp
L Device:R_US R54
U 1 1 5FF6B902
P 3000 4350
F 0 "R54" H 3000 4500 50  0000 L CNN
F 1 "100KΩ;1005" H 3000 4200 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3040 4340 50  0001 C CNN
F 3 "~" H 3000 4350 50  0001 C CNN
	1    3000 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 4200 3000 4100
Connection ~ 3000 3200
Wire Wire Line
	3900 2000 3900 1900
Connection ~ 3900 2000
Wire Wire Line
	3200 5400 3000 5400
Wire Wire Line
	1200 5100 2000 5100
Wire Wire Line
	1200 5300 3200 5300
Wire Wire Line
	1200 6100 2000 6100
Wire Wire Line
	3000 5900 2700 5900
Connection ~ 3000 5400
Wire Wire Line
	2300 5900 2200 5900
Wire Wire Line
	2200 5900 2200 6100
Connection ~ 2200 6100
Text Notes 3950 4650 0    50   ~ 0
E-Stop Set\n3-Input\nAND Gate
$Comp
L Device:C C?
U 1 1 6036DBC8
P 1650 4600
AR Path="/5F48CAAD/6036DBC8" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6036DBC8" Ref="C24"  Part="1" 
F 0 "C24" V 1550 4400 50  0000 L CNN
F 1 "0.1µF;1005" V 1750 4650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1688 4450 50  0001 C CNN
F 3 "~" H 1650 4600 50  0001 C CNN
	1    1650 4600
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 60430375
P 1650 4850
AR Path="/5F48CAAD/60430375" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/60430375" Ref="C25"  Part="1" 
F 0 "C25" V 1550 4650 50  0000 L CNN
F 1 "0.1µF;1005" V 1700 4900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1688 4700 50  0001 C CNN
F 3 "~" H 1650 4850 50  0001 C CNN
	1    1650 4850
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 6043088F
P 1650 4350
AR Path="/5F48CAAD/6043088F" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6043088F" Ref="C23"  Part="1" 
F 0 "C23" V 1550 4150 50  0000 L CNN
F 1 "0.1µF;1005" V 1700 4400 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1688 4200 50  0001 C CNN
F 3 "~" H 1650 4350 50  0001 C CNN
	1    1650 4350
	0    1    1    0   
$EndComp
Wire Wire Line
	1200 3200 1400 3200
Wire Wire Line
	1500 4850 1400 4850
Wire Wire Line
	1400 4850 1400 4600
Connection ~ 1400 3200
Wire Wire Line
	1500 4350 1400 4350
Connection ~ 1400 4350
Wire Wire Line
	1400 4350 1400 4100
Wire Wire Line
	1500 4600 1400 4600
Connection ~ 1400 4600
Wire Wire Line
	1400 4600 1400 4350
Wire Wire Line
	2200 5900 2200 4850
Wire Wire Line
	2200 4850 1800 4850
Connection ~ 2200 5900
Wire Wire Line
	1800 4600 2200 4600
Wire Wire Line
	2200 4600 2200 4850
Connection ~ 2200 4850
Wire Wire Line
	1800 4350 2200 4350
Wire Wire Line
	2200 4350 2200 4600
Connection ~ 2200 4600
Text Notes 3950 4000 0    50   ~ 0
3.3 to 9V\nLevel Shift
Text Notes 4750 5000 0    50   ~ 0
WOW Standby\nInverter
Text Label 3900 2000 0    50   ~ 0
~NWOW_EN
$Comp
L Device:C C?
U 1 1 6057FE20
P 1650 4100
AR Path="/5F48CAAD/6057FE20" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6057FE20" Ref="C22"  Part="1" 
F 0 "C22" V 1550 3900 50  0000 L CNN
F 1 "0.1µF;1005" V 1700 4150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1688 3950 50  0001 C CNN
F 3 "~" H 1650 4100 50  0001 C CNN
	1    1650 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	2200 4350 2200 4100
Wire Wire Line
	2200 4100 1800 4100
Connection ~ 2200 4350
Wire Wire Line
	1500 4100 1400 4100
Connection ~ 1400 4100
Wire Wire Line
	1400 4100 1400 3800
Wire Wire Line
	9500 3500 9500 3300
$Comp
L Device:R_US R62
U 1 1 5F8E9E6C
P 9500 3150
F 0 "R62" H 9350 3250 50  0000 L CNN
F 1 "100KΩ;1608" H 9000 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9540 3140 50  0001 C CNN
F 3 "~" H 9500 3150 50  0001 C CNN
	1    9500 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 5400 6800 4800
Wire Wire Line
	6800 5400 6600 5400
Wire Wire Line
	6800 4800 5400 4800
Wire Wire Line
	8900 4200 8900 5500
Wire Wire Line
	8900 5500 9700 5500
Wire Wire Line
	9700 5400 9000 5400
Wire Wire Line
	9000 5400 9000 4100
Wire Wire Line
	9700 5200 9300 5200
Wire Wire Line
	9300 5200 9300 4200
Wire Wire Line
	9300 4200 9700 4200
Wire Wire Line
	9700 4200 9700 2500
Wire Wire Line
	7400 2500 9700 2500
Wire Wire Line
	9700 5300 9200 5300
Wire Wire Line
	9200 5300 9200 4100
Wire Wire Line
	9200 4100 9600 4100
Wire Wire Line
	9600 4100 9600 2400
Wire Wire Line
	7200 2400 9600 2400
Wire Wire Line
	7200 6100 8700 6100
Wire Wire Line
	9600 6100 9600 5700
Wire Wire Line
	9600 5700 9700 5700
Connection ~ 7200 6100
Wire Wire Line
	9700 5600 9600 5600
Wire Wire Line
	9600 5600 9600 5700
Connection ~ 9600 5700
Wire Wire Line
	9600 5600 9600 5100
Wire Wire Line
	9600 5100 9700 5100
Connection ~ 9600 5600
Wire Wire Line
	9600 5100 9600 5000
Wire Wire Line
	9600 5000 9700 5000
Connection ~ 9600 5100
Wire Wire Line
	9700 5900 9500 5900
Wire Wire Line
	9500 5900 9500 5800
Wire Wire Line
	9500 5800 9700 5800
Wire Wire Line
	9500 5800 9500 4900
Wire Wire Line
	9500 4900 9700 4900
Connection ~ 9500 5800
Wire Wire Line
	9500 4900 9500 4800
Wire Wire Line
	9500 4800 9700 4800
Connection ~ 9500 4900
Wire Wire Line
	9500 4800 9500 4400
Wire Wire Line
	9500 4400 9900 4400
Connection ~ 9500 4800
Connection ~ 9500 2900
Wire Wire Line
	9500 2900 9500 3000
Wire Wire Line
	4700 4900 4700 6100
Wire Wire Line
	2200 6100 3900 6100
$Comp
L HR2:MCP2542;SOIC8 U15
U 1 1 5F9AA16D
P 5500 4100
F 0 "U15" H 5750 4250 50  0000 C CNN
F 1 "MCP2542;SOIC8" H 6050 3550 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5700 4150 60  0001 L CNN
F 3 "MCP2542 CAN Transceiver" H 5700 3950 60  0001 L CNN
F 4 "MCP2542" H 5700 3850 60  0001 L CNN "Field5"
	1    5500 4100
	1    0    0    -1  
$EndComp
Text Notes 7150 750  0    50   ~ 0
Selectable CAN Termination: https://www.ti.com/lit/ug/tiducf3/tiducf3.pdf
$Comp
L HR2:CPC1017N;SOP4W3.8WL4.1 U18
U 1 1 5F9EEF8D
P 8600 3400
F 0 "U18" H 9450 3550 50  0000 C CNN
F 1 "CPC1017N;SOP4W3.8L4.1" H 9150 3150 50  0000 C CNN
F 2 "Package_SO:SOP-4_3.8x4.1mm_P2.54mm" H 8800 3450 60  0001 L CNN
F 3 "CPC1017 Solid State Relay" H 8800 3250 60  0001 L CNN
F 4 "CPC1017 Solid State Relay" H 8800 3150 60  0001 L CNN "Field5"
	1    8600 3400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7400 3500 7400 3700
Wire Wire Line
	7200 3400 7500 3400
Wire Wire Line
	7000 2400 7000 4500
Wire Wire Line
	8600 3500 9500 3500
Wire Wire Line
	7200 1800 7500 1800
Wire Wire Line
	7400 1900 7500 1900
$Comp
L HR2:CPC1017N;SOP4W3.8WL4.1 U17
U 1 1 5F9ECFD5
P 8600 1800
F 0 "U17" H 8850 1950 50  0000 C CNN
F 1 "CPC1017N;SOP4W3.8L4.1" H 9150 1550 50  0000 C CNN
F 2 "Package_SO:SOP-4_3.8x4.1mm_P2.54mm" H 8800 1850 60  0001 L CNN
F 3 "CPC1017 Solid State Relay" H 8800 1650 60  0001 L CNN
F 4 "CPC1017 Solid State Relay" H 8800 1550 60  0001 L CNN "Field5"
	1    8600 1800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8600 1800 9500 1800
Wire Wire Line
	8600 3400 8700 3400
Wire Wire Line
	8700 3400 8700 6100
Connection ~ 8700 6100
Wire Wire Line
	8700 6100 9600 6100
Wire Wire Line
	8600 1900 8700 1900
Wire Wire Line
	8700 1900 8700 3400
Connection ~ 8700 3400
Wire Wire Line
	5300 2300 5300 4400
Wire Wire Line
	3500 2900 3500 4200
Connection ~ 3500 4200
Wire Wire Line
	3500 4200 3500 4700
Wire Wire Line
	6900 4400 6900 4900
Wire Wire Line
	6900 4900 6700 4900
Connection ~ 6900 4400
Connection ~ 6700 4900
Text Notes 850  1400 0    50   ~ 0
TO DO:\n1. Add Test Points.\n
Wire Wire Line
	5700 700  5700 1100
Wire Wire Line
	6000 700  5700 700 
Wire Wire Line
	6800 1100 6700 1100
Wire Wire Line
	6800 700  6800 1100
Wire Wire Line
	6400 700  6800 700 
$Comp
L Jumper:Jumper_2_Open JP?
U 1 1 5FB530EF
P 6200 700
AR Path="/5F52F39E/5FB530EF" Ref="JP?"  Part="1" 
AR Path="/5F8C2F57/5FB530EF" Ref="JP8"  Part="1" 
F 0 "JP8" H 6000 750 50  0000 C CNN
F 1 "SHUNT;M1x2" H 6200 600 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6200 700 50  0001 C CNN
F 3 "~" H 6200 700 50  0001 C CNN
	1    6200 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 1100 5700 1100
Connection ~ 5700 1100
Wire Wire Line
	6800 1100 9900 1100
Connection ~ 6800 1100
Wire Wire Line
	9900 1100 9900 4400
$Comp
L Jumper:Jumper_2_Open JP7
U 1 1 5FB85177
P 3500 5900
F 0 "JP7" H 3350 6000 50  0000 C CNN
F 1 "EXT_ESTOP;M1x2" H 3500 5800 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3500 5900 50  0001 C CNN
F 3 "~" H 3500 5900 50  0001 C CNN
	1    3500 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 5900 3300 5900
Connection ~ 3000 5900
Wire Wire Line
	3700 5900 3900 5900
Wire Wire Line
	3900 5900 3900 6100
$Comp
L Connector:TestPoint TP13
U 1 1 5FB52BFF
P 2600 2000
F 0 "TP13" H 2658 2118 50  0000 L CNN
F 1 "WOW_RX;M1x1" H 2658 2027 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 2800 2000 50  0001 C CNN
F 3 "~" H 2800 2000 50  0001 C CNN
	1    2600 2000
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP12
U 1 1 5FB54150
P 1900 2000
F 0 "TP12" H 1958 2118 50  0000 L CNN
F 1 "WOW_TX;M1x1" H 1958 2027 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 2100 2000 50  0001 C CNN
F 3 "~" H 2100 2000 50  0001 C CNN
	1    1900 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2000 1900 2100
Connection ~ 1900 2100
Wire Wire Line
	1900 2100 1200 2100
Wire Wire Line
	2600 2000 2600 2200
Connection ~ 2600 2200
$Comp
L Connector:TestPoint TP14
U 1 1 5FB6945D
P 4100 5200
F 0 "TP14" H 4150 5400 50  0000 R CNN
F 1 "~nESTOP_SET" H 4600 5300 50  0000 R CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 4300 5200 50  0001 C CNN
F 3 "~" H 4300 5200 50  0001 C CNN
	1    4100 5200
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP15
U 1 1 5FB83E8F
P 4400 2600
F 0 "TP15" H 4150 2750 50  0000 L CNN
F 1 "~nWOW_ESTOP;M1x1" H 4450 2650 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 4600 2600 50  0001 C CNN
F 3 "~" H 4600 2600 50  0001 C CNN
	1    4400 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 4500 3000 5400
$Comp
L HR2:SHUNT R64
U 1 1 5FD3322D
P 5800 1100
F 0 "R64" H 6050 1250 50  0000 C CNN
F 1 "0Ω;1608" H 6250 950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6000 1150 60  0001 L CNN
F 3 "0 Ohm Power Shunt" H 6000 950 60  0001 L CNN
F 4 "0 Ohm Power Shunt" H 6000 850 60  0001 L CNN "Field5"
	1    5800 1100
	1    0    0    -1  
$EndComp
$Comp
L HR2_MISC:Q_PWR_PMOS_GSD Q21
U 1 1 5FD48C48
P 4500 1200
F 0 "Q21" V 4400 1100 50  0000 C CNN
F 1 "PFET_6A;SOT23" V 4751 1200 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4700 1300 50  0001 C CNN
F 3 "~" H 4500 1200 50  0001 C CNN
	1    4500 1200
	0    1    -1   0   
$EndComp
Wire Wire Line
	4700 1100 5700 1100
$Comp
L HR2_MISC:74x11G1 U13
U 1 2 5FD5DBFB
P 3500 5300
F 0 "U13" H 3500 5625 50  0000 C CNN
F 1 "74x11G1" H 3500 5534 50  0000 C CNN
F 2 "" H 3500 5300 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS11" H 3500 5300 50  0001 C CNN
	1    3500 5300
	1    0    0    -1  
$EndComp
$Comp
L HR2_MISC:74x11G1 U13
U 4 2 5FD5FD90
P 1900 3800
F 0 "U13" H 1800 3950 50  0000 C CNN
F 1 "74x11G1" H 1900 3650 50  0000 C CNN
F 2 "" H 1900 3800 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS11" H 1900 3800 50  0001 C CNN
	4    1900 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 5200 5400 5200
Connection ~ 5400 5200
Wire Wire Line
	5400 5200 5400 5300
Wire Wire Line
	3800 5300 4100 5300
Wire Wire Line
	4100 5300 4100 5500
Wire Wire Line
	3900 4400 3900 5900
Wire Wire Line
	1200 5500 4000 5500
Wire Wire Line
	4000 5500 4000 5400
Wire Wire Line
	2000 5600 2000 5100
Wire Wire Line
	2000 5900 2000 6100
Connection ~ 2000 6100
Wire Wire Line
	2000 6100 2200 6100
Wire Wire Line
	3500 4700 4400 4700
Wire Wire Line
	3900 6100 4700 6100
$Comp
L HR2_MISC:74x11G1 U22
U 1 2 5FDF7C11
P 2600 2800
F 0 "U22" H 2800 2700 50  0000 C CNN
F 1 "74x11G1" H 2600 3000 50  0000 C CNN
F 2 "" H 2600 2800 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS11" H 2600 2800 50  0001 C CNN
	1    2600 2800
	-1   0    0    1   
$EndComp
Wire Wire Line
	5300 2300 5500 2300
Wire Wire Line
	6900 2300 6600 2300
Wire Wire Line
	2600 2200 5500 2200
Wire Wire Line
	6700 2200 6600 2200
Wire Wire Line
	7000 2400 6600 2400
Wire Wire Line
	5500 2100 1900 2100
$Comp
L HR2:MCP2542;SOIC8 U14
U 1 1 5F9A7EB1
P 5500 2000
F 0 "U14" H 5750 2150 50  0000 C CNN
F 1 "MCP2542;SOIC8" H 6050 1450 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5700 2050 60  0001 L CNN
F 3 "MCP2542 CAN Transceiver" H 5700 1850 60  0001 L CNN
F 4 "MCP2542" H 5700 1750 60  0001 L CNN "Field5"
	1    5500 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2000 7200 2000
Connection ~ 7200 2000
Wire Wire Line
	7200 2000 7200 1800
Wire Wire Line
	7400 2500 7100 2500
Wire Wire Line
	7100 2500 7100 2100
Wire Wire Line
	7100 2100 6600 2100
Wire Wire Line
	2000 5100 2500 5100
Wire Wire Line
	2500 5100 2500 4700
Connection ~ 2000 5100
Wire Wire Line
	2500 4700 3500 4700
Wire Wire Line
	3000 5900 3000 5400
Connection ~ 3900 5900
Wire Wire Line
	2800 4100 3000 4100
Connection ~ 3000 4100
Wire Wire Line
	3000 4100 3000 3200
Wire Wire Line
	1400 3200 2400 3200
Wire Wire Line
	1500 3800 1400 3800
Connection ~ 1400 3800
Wire Wire Line
	1400 3800 1400 3400
Wire Wire Line
	1400 3400 1500 3400
Connection ~ 1400 3400
Wire Wire Line
	1400 3400 1400 3200
Wire Wire Line
	2300 3800 2400 3800
Wire Wire Line
	2400 3800 2400 3400
Connection ~ 2400 3200
Wire Wire Line
	2400 3200 3000 3200
Wire Wire Line
	2300 3400 2400 3400
Connection ~ 2400 3400
Wire Wire Line
	2400 3400 2400 3200
$Comp
L HR2_MISC:74x11G1 U22
U 4 2 6013227A
P 1900 3400
F 0 "U22" H 2000 3250 50  0000 C CNN
F 1 "74x11G1" H 1900 3550 50  0000 C CNN
F 2 "" H 1900 3400 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS11" H 1900 3400 50  0001 C CNN
	4    1900 3400
	-1   0    0    1   
$EndComp
Wire Wire Line
	6800 4800 6800 2700
Connection ~ 6800 4800
Wire Wire Line
	2900 2800 3000 2800
Wire Wire Line
	2900 2900 3000 2900
Wire Wire Line
	3000 2900 3000 3200
Wire Wire Line
	3000 3200 4700 3200
Text HLabel 1200 2800 0    50   Output ~ 0
~NESTOP
Wire Wire Line
	1200 2800 2200 2800
Text Label 3100 2800 0    50   ~ 0
~NWOW_ESTOP
Text Label 3100 2700 0    50   ~ 0
~NHR2_ESTOP
Text HLabel 1200 5400 0    50   Output ~ 0
~NBTN_ESTOP
Wire Wire Line
	1200 5400 3000 5400
Text Notes 700  4500 0    50   ~ 0
Filter Capacitors
Text Notes 950  3650 0    50   ~ 0
AND Gate\nPower Pins
Text Notes 2000 2450 0    50   ~ 0
E-Stop AND Gate
Text Label 1200 5400 0    50   ~ 0
~NHBTN_ESTOP
Text Label 1200 2800 0    50   ~ 0
~NESTOP
Text Notes 7750 1550 0    50   ~ 0
Solid State Relay
Wire Wire Line
	4100 5500 5500 5500
Wire Wire Line
	4000 5400 5500 5400
Wire Wire Line
	4100 5300 4100 5200
Connection ~ 4100 5300
Wire Wire Line
	3500 2900 9500 2900
Wire Wire Line
	2900 2700 6800 2700
Text HLabel 1200 2500 0    50   Output ~ 0
~NWOW_ESTOP
Wire Wire Line
	1200 2500 3000 2500
Wire Wire Line
	3000 2500 3000 2800
Connection ~ 3000 2800
Wire Wire Line
	3000 2800 4400 2800
Text Label 1200 2500 0    50   ~ 0
~NWOW_ESTOP
Wire Wire Line
	2200 2800 2200 3100
Wire Wire Line
	2200 3100 7100 3100
Wire Wire Line
	7100 3100 7100 4800
Wire Wire Line
	7100 4800 7200 4800
Connection ~ 2200 2800
Wire Wire Line
	2200 2800 2300 2800
Wire Wire Line
	7200 4800 7200 5100
NoConn ~ 6600 5300
Wire Wire Line
	4400 2800 4400 2600
Connection ~ 4400 2800
Wire Wire Line
	4400 2800 5400 2800
Wire Wire Line
	5400 2800 5400 4200
Wire Wire Line
	5400 4200 5500 4200
Text Notes 2300 5750 0    50   ~ 0
Manual\nE-Stop\nButton
Text Notes 3050 5750 0    50   ~ 0
External\nE-Stop\nButton
$EndSCHEMATC
