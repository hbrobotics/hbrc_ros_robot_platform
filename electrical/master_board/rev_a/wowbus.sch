EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 8 10
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
L Device:R_US R59
U 1 1 5F846FDA
P 7300 2150
F 0 "R59" H 7150 2250 50  0000 L CNN
F 1 "120Ω1/4W;1608" H 7350 2050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7340 2140 50  0001 C CNN
F 3 "~" H 7300 2150 50  0001 C CNN
	1    7300 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2400 7100 2400
Wire Wire Line
	7100 2400 7100 1800
Wire Wire Line
	7300 2000 7300 1900
Wire Wire Line
	6500 2500 7300 2500
Wire Wire Line
	7300 2500 7300 2300
$Comp
L Device:R_US R55
U 1 1 5F869783
P 3800 1400
F 0 "R55" H 3650 1500 50  0000 L CNN
F 1 "100KΩ;1608" H 3850 1300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3840 1390 50  0001 C CNN
F 3 "~" H 3800 1400 50  0001 C CNN
	1    3800 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1100 3800 1100
Wire Wire Line
	3800 1100 3800 1250
Wire Wire Line
	3800 1650 4400 1650
Wire Wire Line
	6900 2800 6500 2800
Wire Wire Line
	5200 2700 5400 2700
Text HLabel 1100 1100 0    50   Input ~ 0
9V
Wire Wire Line
	1100 1100 3800 1100
Connection ~ 3800 1100
Text HLabel 1100 6100 0    50   Input ~ 0
GND
Text HLabel 1100 4700 0    50   Input ~ 0
WOW_EN
$Comp
L Device:R_US R57
U 1 1 5F873ADD
P 4600 3450
F 0 "R57" H 4450 3550 50  0000 L CNN
F 1 "100KΩ;1608" H 4650 3350 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4640 3440 50  0001 C CNN
F 3 "~" H 4600 3450 50  0001 C CNN
	1    4600 3450
	1    0    0    -1  
$EndComp
$Comp
L HR2:WOW_OUT;M2x6S CN10
U 1 1 5F848F57
P 9600 4800
F 0 "CN10" H 9800 4950 50  0000 L CNN
F 1 "WOW_OUT;M2x6S" H 9750 3550 50  0000 L CNN
F 2 "HR2:PinHeader_2x06_P2.54mm_Vertical_Shrouded" H 9800 4850 60  0001 L CNN
F 3 "WOW Output Connector" H 9800 4650 60  0001 L CNN
F 4 "WOW Output Connector" H 9800 4550 60  0001 L CNN "Field5"
	1    9600 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4400 5400 4400
$Comp
L Device:R_US R60
U 1 1 5F84B29E
P 7300 3850
F 0 "R60" H 7150 3950 50  0000 L CNN
F 1 "120Ω1/4W;1608" H 7350 3750 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7340 3840 50  0001 C CNN
F 3 "~" H 7300 3850 50  0001 C CNN
	1    7300 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 4100 7100 4100
Wire Wire Line
	6500 4200 7300 4200
Wire Wire Line
	7300 4200 7300 4000
Wire Wire Line
	7100 4100 7100 3400
Wire Wire Line
	7300 3500 7400 3500
Wire Wire Line
	7100 4100 8900 4100
Connection ~ 7100 4100
Wire Wire Line
	7300 4200 8800 4200
Connection ~ 7300 4200
Connection ~ 7300 2500
Connection ~ 7100 2400
Wire Wire Line
	4500 1200 4400 1200
Wire Wire Line
	4400 1200 4400 1650
Text HLabel 1100 3200 0    50   Input ~ 0
3.3V
Wire Wire Line
	6800 2700 6500 2700
Wire Wire Line
	6500 4400 6800 4400
Text HLabel 1100 1800 0    50   Input ~ 0
5V
Wire Wire Line
	1100 1800 6600 1800
Wire Wire Line
	6600 1800 6600 2600
Wire Wire Line
	6600 2600 6500 2600
Wire Wire Line
	6500 4300 6600 4300
Wire Wire Line
	6600 4300 6600 2600
Connection ~ 6600 2600
Wire Wire Line
	6500 4500 6900 4500
Connection ~ 6800 3200
Wire Wire Line
	6800 3200 6800 2700
Wire Wire Line
	6900 6100 6900 5500
Connection ~ 6900 4500
Connection ~ 4600 6100
$Comp
L Device:R_US R56
U 1 1 5F98DD9D
P 4000 5850
F 0 "R56" H 3850 5950 50  0000 L CNN
F 1 "100KΩ;1608" H 4000 5700 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4040 5840 50  0001 C CNN
F 3 "~" H 4000 5850 50  0001 C CNN
	1    4000 5850
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F9A0944
P 3700 4200
AR Path="/5F79BC00/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5F9A0944" Ref="Q20"  Part="1" 
F 0 "Q20" H 3600 4350 50  0000 L CNN
F 1 "2N7000;SOT23" H 3850 4050 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3900 4125 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 3700 4200 50  0001 L CNN
	1    3700 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1650 3800 4000
Text HLabel 1100 2500 0    50   Input ~ 0
WOW_TX
Text HLabel 1100 2600 0    50   Output ~ 0
WOW_RX
Wire Wire Line
	5400 2500 1800 2500
Wire Wire Line
	1100 2600 2500 2600
Wire Wire Line
	9400 1800 9400 1900
$Comp
L Device:R_US R61
U 1 1 5F8EAE9D
P 9400 2050
F 0 "R61" H 9250 2150 50  0000 L CNN
F 1 "100KΩ;1608" H 8900 1900 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9440 2040 50  0001 C CNN
F 3 "~" H 9400 2050 50  0001 C CNN
	1    9400 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2200 9400 2900
$Comp
L HR2:74LVC1G74;TSSOP8 U16
U 1 1 5F8C5AF5
P 5400 5200
F 0 "U16" H 5650 5350 50  0000 C CNN
F 1 "74LVC1G74;TSSOP8" H 5950 4750 50  0000 C CNN
F 2 "Package_SO:TSSOP-8_3x3mm_P0.65mm" H 5600 5250 60  0001 L CNN
F 3 "D-type flip-flop edge-trigger/set/reset" H 5600 5050 60  0001 L CNN
F 4 "D-type flip-flop edge-trigger/set/reset" H 5600 4950 60  0001 L CNN "Field5"
	1    5400 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5200 6500 5200
Wire Wire Line
	6500 5500 6900 5500
Connection ~ 6900 5500
Wire Wire Line
	6900 5500 6900 4500
Wire Wire Line
	6700 6200 1100 6200
Wire Wire Line
	6800 6300 1100 6300
Text HLabel 1100 6300 0    50   Output ~ 0
ESTOP
Text HLabel 1100 6200 0    50   Output ~ 0
~ESTOP
Text HLabel 1100 5500 0    50   Input ~ 0
~ESTOP_CLR
Text HLabel 1100 5200 0    50   Input ~ 0
~NRST
Wire Wire Line
	5300 5300 5400 5300
Text Notes 7200 5800 0    50   ~ 0
Red ESTOP LED
Text Notes 5200 2200 0    50   ~ 0
Communication CANBus Transceiver
Text Notes 5550 3900 0    50   ~ 0
ESTOP CANBus Transceiver
Text Notes 4550 900  0    50   ~ 0
WOW Power P-FET (6A)
Text Notes 5400 5000 0    50   ~ 0
E-Stop x74 D Style Flip-Flop
Text Notes 6950 1300 0    50   ~ 0
Opto-isolator
Text Notes 7650 3200 0    50   ~ 0
Solid State Relay
Text Notes 9500 4600 0    50   ~ 0
WOWBus Output Connector
Wire Wire Line
	3400 4200 3500 4200
$Comp
L Device:R_US R53
U 1 1 5FC61F4D
P 2200 3450
F 0 "R53" H 2050 3550 50  0000 L CNN
F 1 "100KΩ;1608" H 2200 3300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2240 3440 50  0001 C CNN
F 3 "~" H 2200 3450 50  0001 C CNN
	1    2200 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 3300 2200 3200
Connection ~ 2200 3200
$Comp
L Device:LED D?
U 1 1 5FC8C292
P 7100 5650
AR Path="/5F79BC00/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5FA4A874/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5FAD82EE/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5F8C2F57/5FC8C292" Ref="D3"  Part="1" 
F 0 "D3" V 7200 5750 50  0000 R CNN
F 1 "REDLED;1608" V 7050 5550 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7100 5650 50  0001 C CNN
F 3 "~" H 7100 5650 50  0001 C CNN
	1    7100 5650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R58
U 1 1 5FC946D0
P 7100 5250
F 0 "R58" H 6950 5350 50  0000 L CNN
F 1 "470Ω;1608" H 7150 5150 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7140 5240 50  0001 C CNN
F 3 "~" H 7100 5250 50  0001 C CNN
	1    7100 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 6100 7100 6100
Wire Wire Line
	7100 6100 7100 5800
Connection ~ 6900 6100
Wire Wire Line
	6800 5000 7100 5000
Wire Wire Line
	7100 5000 7100 5100
Text Label 8800 2400 0    50   ~ 0
WOW_COMH
Text Label 8800 2500 0    50   ~ 0
WOW_COML
Text Label 7600 4100 0    50   ~ 0
WOW_STOPH
Text Label 7600 4200 0    50   ~ 0
WOW_STOPL
Text Label 1100 1100 0    50   ~ 0
9V
Text Label 1100 1800 0    50   ~ 0
5V
Text Label 1100 2500 0    50   ~ 0
WOW_TX
Text Label 1100 2600 0    50   ~ 0
WOW_RX
Text Label 1100 3200 0    50   ~ 0
3.3V
Text Label 1100 4700 0    50   ~ 0
WOW_EN
Text Label 1100 5500 0    50   ~ 0
~ESTOP_CLR
Text Label 1100 5200 0    50   ~ 0
~NRST
Text Label 1100 6200 0    50   ~ 0
~ESTOP
Text Label 1100 6300 0    50   ~ 0
ESTOP
Text Label 1100 6100 0    50   ~ 0
GND
Text Label 5000 4800 0    50   ~ 0
~ESTOP
Text Label 4650 3800 0    50   ~ 0
WOW_STDBY
Connection ~ 3400 4700
Text Label 7900 1200 0    50   ~ 0
WOW_POW_OUT
Text Label 8700 3500 0    50   ~ 0
WOW_ESTOP_OPTO
Text Label 8800 1800 0    50   ~ 0
WOW_COM_OPTO
Text Notes 700  7900 0    50   ~ 0
WOWBus Notes:\n\n1. WOWBus is an an expansion bus.\n2. WOWBus stands for Wayne's Omnipotent Wonderful Bus (heavy sarcasm.)\n3. WOWBus uses 2x6 ribbon daisy chain cables for interconnect.\n4. WOWBus has a nominal 9V rail, but this can droop due to resistive losses down the daisy chain.\n5. The WOWBus connector is designed to mirror image power, ground, and signaling.\n6. If a WOWBus connector is accidently reversed on a daisy cable, nothing shorts out.\n7. However, an accidentally reversed cable, will basically make the bus non-fuctional until corrected.\n8. The WOWBus has two "CAN bus" (ISO-11898-2) physical differential pairs for siginalling.\n9. The first differential pair is used as a multi-drop bidirectional data transfer.\n10. The second differential pair is used to signal Emergency Stop.\n11. When the WOWBus is enabled, it powers up both the transceivers and the differential\n    pair termination resistors.\n12. The expansions boards are responsible for providing the bus terminator resistors for\n    for the other end of the differential pairs.\n13. The Bus powers up in the EStop conndition and must be explicitly cleared out.\n14. When ESTOP occurs, it latches up and must be explicitly cleared.\n
Connection ~ 5200 3800
Wire Wire Line
	5200 3800 5200 4400
Wire Wire Line
	5000 4800 5000 4300
Wire Wire Line
	5000 4300 5400 4300
Wire Wire Line
	4600 3600 4600 3800
Connection ~ 4600 3800
Wire Wire Line
	4600 3800 5200 3800
Text HLabel 1100 5100 0    50   Input ~ 0
~MOTOR_FAULT
Text Label 1100 5100 0    50   ~ 0
~MOTOR_FAULT
Wire Wire Line
	4600 6100 4000 6100
Wire Wire Line
	5300 4900 5300 5300
Wire Wire Line
	6600 4900 6600 5200
Wire Wire Line
	6600 4900 5300 4900
Text Label 4850 5200 0    50   ~ 0
~ESTOP_SET
Text Label 4850 5400 0    50   ~ 0
ESTOP_TRIG
Connection ~ 2200 5100
Wire Wire Line
	2200 3600 2200 5100
Wire Wire Line
	1100 5100 2200 5100
Wire Wire Line
	2200 5100 2800 5100
Wire Wire Line
	7100 5400 7100 5500
Wire Wire Line
	4600 6100 6900 6100
Wire Wire Line
	4600 3800 4600 4500
Wire Wire Line
	3400 4700 4000 4700
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F86962A
P 4500 4700
AR Path="/5F79BC00/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5F86962A" Ref="Q22"  Part="1" 
F 0 "Q22" H 4400 4850 50  0000 L CNN
F 1 "2N7000;SOT23" H 4000 4500 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4700 4625 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 4500 4700 50  0001 L CNN
	1    4500 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3200 6800 3200
Connection ~ 4600 3200
Wire Wire Line
	4600 3200 4600 3300
Wire Wire Line
	6800 3200 6800 4400
$Comp
L Switch:SW_Push SW?
U 1 1 5FC73CCB
P 2400 5900
AR Path="/5F52F39E/5FC73CCB" Ref="SW?"  Part="1" 
AR Path="/5F8C2F57/5FC73CCB" Ref="SW3"  Part="1" 
F 0 "SW3" H 2250 6000 50  0000 C CNN
F 1 "BUTTON;6x3.5" H 2400 5800 50  0000 C CNN
F 2 "HR2:BUTTON_6x3.5" H 2400 6100 50  0001 C CNN
F 3 "~" H 2400 6100 50  0001 C CNN
	1    2400 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 5400 4800 5400
Wire Wire Line
	5400 4200 5300 4200
Wire Wire Line
	4800 4200 4800 5400
Connection ~ 3800 6100
$Comp
L Device:R_US R54
U 1 1 5FF6B902
P 2700 3450
F 0 "R54" H 2550 3550 50  0000 L CNN
F 1 "100KΩ;1608" H 2700 3300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2740 3440 50  0001 C CNN
F 3 "~" H 2700 3450 50  0001 C CNN
	1    2700 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 3300 2700 3200
Connection ~ 2700 3200
Wire Wire Line
	3800 1650 3800 1550
Connection ~ 3800 1650
Connection ~ 4000 4700
Wire Wire Line
	4000 4700 4300 4700
Wire Wire Line
	4000 6000 4000 6100
Connection ~ 4000 6100
Wire Wire Line
	4000 6100 3800 6100
Wire Wire Line
	2200 3200 2700 3200
Wire Wire Line
	2800 5300 2700 5300
Wire Wire Line
	1100 4700 3400 4700
Wire Wire Line
	5400 5200 3700 5200
Wire Wire Line
	1100 5200 2800 5200
Wire Wire Line
	1100 5500 4800 5500
Wire Wire Line
	1100 6100 2100 6100
Wire Wire Line
	2700 5300 2700 5900
Wire Wire Line
	2700 5900 2600 5900
Connection ~ 2700 5300
Wire Wire Line
	2200 5900 2100 5900
Wire Wire Line
	2100 5900 2100 6100
Connection ~ 2100 6100
Text Notes 2200 5750 0    50   ~ 0
Manual\nE-Stop\nButton
Text Notes 3150 5000 0    50   ~ 0
E-Stop Set\n3-Input\nAND Gate
$Comp
L Device:C C?
U 1 1 6036DBC8
P 1550 4200
AR Path="/5F48CAAD/6036DBC8" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6036DBC8" Ref="C24"  Part="1" 
F 0 "C24" V 1450 4000 50  0000 L CNN
F 1 "10pF;1608" V 1650 4250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1588 4050 50  0001 C CNN
F 3 "~" H 1550 4200 50  0001 C CNN
	1    1550 4200
	0    1    1    0   
$EndComp
Text Notes 1400 3150 0    50   ~ 0
Bypass Capacitors
$Comp
L Device:C C?
U 1 1 60430375
P 1550 4500
AR Path="/5F48CAAD/60430375" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/60430375" Ref="C25"  Part="1" 
F 0 "C25" V 1450 4300 50  0000 L CNN
F 1 "10pF;1608" V 1600 4550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1588 4350 50  0001 C CNN
F 3 "~" H 1550 4500 50  0001 C CNN
	1    1550 4500
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 6043088F
P 1550 3900
AR Path="/5F48CAAD/6043088F" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6043088F" Ref="C23"  Part="1" 
F 0 "C23" V 1450 3700 50  0000 L CNN
F 1 "10pF;1608" V 1600 3950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1588 3750 50  0001 C CNN
F 3 "~" H 1550 3900 50  0001 C CNN
	1    1550 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 3200 1300 3200
Wire Wire Line
	1400 4500 1300 4500
Wire Wire Line
	1300 4500 1300 4200
Connection ~ 1300 3200
Wire Wire Line
	1300 3200 2200 3200
Wire Wire Line
	1400 3900 1300 3900
Connection ~ 1300 3900
Wire Wire Line
	1300 3900 1300 3600
Wire Wire Line
	1400 4200 1300 4200
Connection ~ 1300 4200
Wire Wire Line
	1300 4200 1300 3900
Wire Wire Line
	2100 5900 2100 4500
Wire Wire Line
	2100 4500 1700 4500
Connection ~ 2100 5900
Wire Wire Line
	1700 4200 2100 4200
Wire Wire Line
	2100 4200 2100 4500
Connection ~ 2100 4500
Wire Wire Line
	1700 3900 2100 3900
Wire Wire Line
	2100 3900 2100 4200
Connection ~ 2100 4200
Text Notes 3850 4000 0    50   ~ 0
3.3 to 9V\nLevel Shift
Text Notes 4050 5100 0    50   ~ 0
WOW Standby\nInverter
Text Label 3800 1650 0    50   ~ 0
~WOW_EN
$Comp
L Device:C C?
U 1 1 6057FE20
P 1550 3600
AR Path="/5F48CAAD/6057FE20" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6057FE20" Ref="C22"  Part="1" 
F 0 "C22" V 1450 3400 50  0000 L CNN
F 1 "10pF;1608" V 1600 3650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1588 3450 50  0001 C CNN
F 3 "~" H 1550 3600 50  0001 C CNN
	1    1550 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	2100 3900 2100 3600
Wire Wire Line
	2100 3600 1700 3600
Connection ~ 2100 3900
Wire Wire Line
	1400 3600 1300 3600
Connection ~ 1300 3600
Wire Wire Line
	1300 3600 1300 3200
Wire Wire Line
	9400 3500 9400 3300
$Comp
L Device:R_US R62
U 1 1 5F8E9E6C
P 9400 3150
F 0 "R62" H 9250 3250 50  0000 L CNN
F 1 "100KΩ;1608" H 8900 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9440 3140 50  0001 C CNN
F 3 "~" H 9400 3150 50  0001 C CNN
	1    9400 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5400 6700 4800
Wire Wire Line
	6700 5400 6500 5400
Wire Wire Line
	6700 4800 5000 4800
Wire Wire Line
	6700 5400 6700 6200
Connection ~ 6700 5400
Wire Wire Line
	6500 5300 6800 5300
Wire Wire Line
	6800 5000 6800 5300
Wire Wire Line
	6800 5300 6800 6300
Connection ~ 6800 5300
Wire Wire Line
	8800 4200 8800 5500
Wire Wire Line
	8800 5500 9600 5500
Wire Wire Line
	9600 5400 8900 5400
Wire Wire Line
	8900 5400 8900 4100
Wire Wire Line
	9600 5200 9200 5200
Wire Wire Line
	9200 5200 9200 4200
Wire Wire Line
	9200 4200 9600 4200
Wire Wire Line
	9600 4200 9600 2500
Wire Wire Line
	7300 2500 9600 2500
Wire Wire Line
	9600 5300 9100 5300
Wire Wire Line
	9100 5300 9100 4100
Wire Wire Line
	9100 4100 9500 4100
Wire Wire Line
	9500 4100 9500 2400
Wire Wire Line
	7100 2400 9500 2400
Wire Wire Line
	7100 6100 8600 6100
Wire Wire Line
	9500 6100 9500 5700
Wire Wire Line
	9500 5700 9600 5700
Connection ~ 7100 6100
Wire Wire Line
	9600 5600 9500 5600
Wire Wire Line
	9500 5600 9500 5700
Connection ~ 9500 5700
Wire Wire Line
	9500 5600 9500 5100
Wire Wire Line
	9500 5100 9600 5100
Connection ~ 9500 5600
Wire Wire Line
	9500 5100 9500 5000
Wire Wire Line
	9500 5000 9600 5000
Connection ~ 9500 5100
Wire Wire Line
	9600 5900 9400 5900
Wire Wire Line
	9400 5900 9400 5800
Wire Wire Line
	9400 5800 9600 5800
Wire Wire Line
	9400 5800 9400 4900
Wire Wire Line
	9400 4900 9600 4900
Connection ~ 9400 5800
Wire Wire Line
	9400 4900 9400 4800
Wire Wire Line
	9400 4800 9600 4800
Connection ~ 9400 4900
Wire Wire Line
	9400 4800 9400 4400
Wire Wire Line
	9400 4400 9800 4400
Connection ~ 9400 4800
Wire Wire Line
	7000 3100 7000 2900
Wire Wire Line
	7000 2900 9400 2900
Connection ~ 9400 2900
Wire Wire Line
	9400 2900 9400 3000
$Comp
L HR2:POWER_PFET_GSD Q21
U 1 1 5F9BEDD5
P 4500 1100
F 0 "Q21" H 4750 1250 50  0000 C CNN
F 1 "PFET_6A_GSD;SOT23" H 4900 850 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4700 1150 60  0001 L CNN
F 3 "Power PFG (GSD pinout)" H 4700 950 60  0001 L CNN
F 4 "Power PFET (GSD pinout)" H 4700 850 60  0001 L CNN "Field5"
	1    4500 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 5700 4000 4700
Wire Wire Line
	4600 4900 4600 6100
Wire Wire Line
	2100 6100 3500 6100
$Comp
L HR2:MCP2542;SOIC8 U14
U 1 1 5F9A7EB1
P 5400 2400
F 0 "U14" H 5650 2550 50  0000 C CNN
F 1 "MCP2542;SOIC8" H 5950 1850 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5600 2450 60  0001 L CNN
F 3 "MCP2542 CAN Transceiver" H 5600 2250 60  0001 L CNN
F 4 "MCP2542" H 5600 2150 60  0001 L CNN "Field5"
	1    5400 2400
	1    0    0    -1  
$EndComp
$Comp
L HR2:MCP2542;SOIC8 U15
U 1 1 5F9AA16D
P 5400 4100
F 0 "U15" H 5650 4250 50  0000 C CNN
F 1 "MCP2542;SOIC8" H 5950 3550 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5600 4150 60  0001 L CNN
F 3 "MCP2542 CAN Transceiver" H 5600 3950 60  0001 L CNN
F 4 "MCP2542" H 5600 3850 60  0001 L CNN "Field5"
	1    5400 4100
	1    0    0    -1  
$EndComp
Text Notes 6400 1500 0    50   ~ 0
Selectable CAN Termination: https://www.ti.com/lit/ug/tiducf3/tiducf3.pdf
$Comp
L HR2:CPC1017N;SOP4W3.8WL4.1 U18
U 1 1 5F9EEF8D
P 8500 3400
F 0 "U18" H 9350 3550 50  0000 C CNN
F 1 "CPC1017N;SOP4W3.8L4.1" H 9050 3150 50  0000 C CNN
F 2 "Package_SO:SOP-4_3.8x4.1mm_P2.54mm" H 8700 3450 60  0001 L CNN
F 3 "CPC1017 Solid State Relay" H 8700 3250 60  0001 L CNN
F 4 "CPC1017 Solid State Relay" H 8700 3150 60  0001 L CNN "Field5"
	1    8500 3400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7300 3500 7300 3700
Wire Wire Line
	7100 3400 7400 3400
Wire Wire Line
	6900 2800 6900 4500
Wire Wire Line
	8500 3500 9400 3500
Wire Wire Line
	7100 1800 7400 1800
Wire Wire Line
	7300 1900 7400 1900
$Comp
L HR2:CPC1017N;SOP4W3.8WL4.1 U17
U 1 1 5F9ECFD5
P 8500 1800
F 0 "U17" H 8750 1950 50  0000 C CNN
F 1 "CPC1017N;SOP4W3.8L4.1" H 9050 1550 50  0000 C CNN
F 2 "Package_SO:SOP-4_3.8x4.1mm_P2.54mm" H 8700 1850 60  0001 L CNN
F 3 "CPC1017 Solid State Relay" H 8700 1650 60  0001 L CNN
F 4 "CPC1017 Solid State Relay" H 8700 1550 60  0001 L CNN "Field5"
	1    8500 1800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8500 1800 9400 1800
Wire Wire Line
	8500 3400 8600 3400
Wire Wire Line
	8600 3400 8600 6100
Connection ~ 8600 6100
Wire Wire Line
	8600 6100 9500 6100
Wire Wire Line
	8500 1900 8600 1900
Wire Wire Line
	8600 1900 8600 3400
Connection ~ 8600 3400
Wire Wire Line
	5200 2700 5200 3800
Wire Wire Line
	7000 3100 3400 3100
Wire Wire Line
	3400 3100 3400 4200
Connection ~ 3400 4200
Wire Wire Line
	3400 4200 3400 4700
$Comp
L HR2:74LVC1G11;SOT363 U13
U 1 1 5FA1469C
P 2800 5100
F 0 "U13" H 3050 5250 50  0000 C CNN
F 1 "74LVC1G11;SOT363" H 3250 4750 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 3000 5150 60  0001 L CNN
F 3 "1 3-Input AND Gate" H 3000 4950 60  0001 L CNN
F 4 "1 3-Input and Gate" H 3000 4850 60  0001 L CNN "Field5"
	1    2800 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 4400 3800 5300
Wire Wire Line
	3700 5300 3800 5300
Connection ~ 3800 5300
Wire Wire Line
	3800 5300 3800 6100
Wire Wire Line
	2700 3200 4400 3200
Wire Wire Line
	3700 5100 3900 5100
Wire Wire Line
	3900 5100 3900 4450
Wire Wire Line
	3900 4450 4400 4450
Wire Wire Line
	4400 4450 4400 3200
Connection ~ 4400 3200
Wire Wire Line
	4400 3200 4600 3200
Wire Wire Line
	6800 4400 6800 4900
Wire Wire Line
	6800 4900 6600 4900
Connection ~ 6800 4400
Connection ~ 6600 4900
Text Notes 750  1400 0    50   ~ 0
TO DO:\n1. Add Test Points.\n
Wire Wire Line
	5600 800  5600 1200
Wire Wire Line
	5900 800  5600 800 
Wire Wire Line
	6700 1200 6600 1200
Wire Wire Line
	6700 800  6700 1200
Wire Wire Line
	6300 800  6700 800 
$Comp
L Jumper:Jumper_2_Open JP?
U 1 1 5FB530EF
P 6100 800
AR Path="/5F52F39E/5FB530EF" Ref="JP?"  Part="1" 
AR Path="/5F8C2F57/5FB530EF" Ref="JP8"  Part="1" 
F 0 "JP8" H 5900 850 50  0000 C CNN
F 1 "SHUNT;M1x2" H 6100 700 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6100 800 50  0001 C CNN
F 3 "~" H 6100 800 50  0001 C CNN
	1    6100 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1200 5600 1200
Connection ~ 5600 1200
Wire Wire Line
	6700 1200 9800 1200
Connection ~ 6700 1200
Wire Wire Line
	5300 1200 5600 1200
Wire Wire Line
	9800 1200 9800 4400
$Comp
L Jumper:Jumper_2_Open JP7
U 1 1 5FB85177
P 3100 5900
F 0 "JP7" H 2950 6000 50  0000 C CNN
F 1 "EXT_ESTOP;M1x2" H 3100 5800 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3100 5900 50  0001 C CNN
F 3 "~" H 3100 5900 50  0001 C CNN
	1    3100 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 5900 2900 5900
Connection ~ 2700 5900
Wire Wire Line
	3300 5900 3500 5900
Wire Wire Line
	3500 5900 3500 6100
Connection ~ 3500 6100
Wire Wire Line
	3500 6100 3800 6100
$Comp
L Connector:TestPoint TP13
U 1 1 5FB52BFF
P 2500 2350
F 0 "TP13" H 2558 2468 50  0000 L CNN
F 1 "WOW_RX;M1x1" H 2558 2377 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 2700 2350 50  0001 C CNN
F 3 "~" H 2700 2350 50  0001 C CNN
	1    2500 2350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP12
U 1 1 5FB54150
P 1800 2350
F 0 "TP12" H 1858 2468 50  0000 L CNN
F 1 "WOW_TX;M1x1" H 1858 2377 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 2000 2350 50  0001 C CNN
F 3 "~" H 2000 2350 50  0001 C CNN
	1    1800 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2350 1800 2500
Connection ~ 1800 2500
Wire Wire Line
	1800 2500 1100 2500
Wire Wire Line
	2500 2350 2500 2600
Connection ~ 2500 2600
Wire Wire Line
	2500 2600 5400 2600
$Comp
L Connector:TestPoint TP14
U 1 1 5FB6945D
P 4800 5600
F 0 "TP14" H 4742 5620 50  0000 R CNN
F 1 "~ESTOP_CLR" H 4742 5718 50  0000 R CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 5000 5600 50  0001 C CNN
F 3 "~" H 5000 5600 50  0001 C CNN
	1    4800 5600
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 5600 4800 5500
Connection ~ 4800 5500
Wire Wire Line
	4800 5500 5400 5500
$Comp
L Connector:TestPoint TP15
U 1 1 5FB83E8F
P 5300 3700
F 0 "TP15" H 5358 3818 50  0000 L CNN
F 1 "ESTOP_TRIG;M1x1" H 5350 3750 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm" H 5500 3700 50  0001 C CNN
F 3 "~" H 5500 3700 50  0001 C CNN
	1    5300 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4200 5300 3700
Connection ~ 5300 4200
Wire Wire Line
	5300 4200 4800 4200
Wire Wire Line
	2700 3600 2700 5300
$Comp
L HR2:SHUNT R64
U 1 1 5FD3322D
P 5700 1200
F 0 "R64" H 5950 1350 50  0000 C CNN
F 1 "0Ω;1608" H 6150 1050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5900 1250 60  0001 L CNN
F 3 "0 Ohm Power Shunt" H 5900 1050 60  0001 L CNN
F 4 "0 Ohm Power Shunt" H 5900 950 60  0001 L CNN "Field5"
	1    5700 1200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
