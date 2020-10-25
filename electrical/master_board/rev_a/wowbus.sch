EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 10 10
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
L HR2:MCP2542;SOIC8 U14
U 1 1 5F846C97
P 5400 2200
F 0 "U14" H 5650 2350 50  0000 C CNN
F 1 "LTV-827" H 5950 1650 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5600 2250 60  0001 L CNN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 5600 2050 60  0001 L CNN
F 4 "MCP2542" H 5600 1950 60  0001 L CNN "Field5"
	1    5400 2200
	1    0    0    -1  
$EndComp
$Comp
L Isolator:LTV-827 U17
U 1 1 5F8476A1
P 7400 1400
F 0 "U17" H 7250 1600 50  0000 C CNN
F 1 "LTV-827" H 7400 1200 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 7200 1200 50  0001 L CIN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 7400 1400 50  0001 L CNN
	1    7400 1400
	1    0    0    -1  
$EndComp
$Comp
L Isolator:LTV-827 U18
U 1 1 5F849475
P 8400 1400
F 0 "U18" H 8550 1200 50  0000 C CNN
F 1 "LTV-827" H 8400 1600 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 8200 1200 50  0001 L CIN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 8400 1400 50  0001 L CNN
	1    8400 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	7700 1500 8000 1500
Wire Wire Line
	7700 1300 7800 1300
Wire Wire Line
	7100 1500 7000 1500
Wire Wire Line
	7000 1700 8800 1700
Wire Wire Line
	8800 1700 8800 1500
Wire Wire Line
	8800 1500 8700 1500
Wire Wire Line
	7000 1500 7000 1700
$Comp
L Device:R_US R41
U 1 1 5F846FDA
P 8000 1950
F 0 "R41" H 7850 2050 50  0000 L CNN
F 1 "120Ω1/4W;1608" H 8050 1850 50  0000 L CNN
F 2 "" V 8040 1940 50  0001 C CNN
F 3 "~" H 8000 1950 50  0001 C CNN
	1    8000 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2200 7800 2200
Wire Wire Line
	7800 2200 7800 1300
Connection ~ 7800 1300
Wire Wire Line
	7800 1300 8100 1300
Wire Wire Line
	8000 1800 8000 1500
Connection ~ 8000 1500
Wire Wire Line
	8000 1500 8100 1500
Wire Wire Line
	6500 2300 8000 2300
Wire Wire Line
	8000 2300 8000 2100
$Comp
L Device:R_US R37
U 1 1 5F869783
P 3800 1200
F 0 "R37" H 3650 1300 50  0000 L CNN
F 1 "100KΩ;1608" H 3850 1100 50  0000 L CNN
F 2 "" V 3840 1190 50  0001 C CNN
F 3 "~" H 3800 1200 50  0001 C CNN
	1    3800 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 900  3800 900 
Wire Wire Line
	3800 900  3800 1050
Wire Wire Line
	3800 1450 4400 1450
Wire Wire Line
	6900 2600 6500 2600
Wire Wire Line
	5300 2500 5400 2500
Text HLabel 1100 900  0    50   Input ~ 0
9V
Wire Wire Line
	1100 900  3800 900 
Connection ~ 3800 900 
Text HLabel 1100 5900 0    50   Input ~ 0
GND
Text HLabel 1100 4500 0    50   Input ~ 0
WOW_EN
$Comp
L Device:R_US R38
U 1 1 5F873ADD
P 4600 3250
F 0 "R38" H 4450 3350 50  0000 L CNN
F 1 "100KΩ;1608" H 4650 3150 50  0000 L CNN
F 2 "" V 4640 3240 50  0001 C CNN
F 3 "~" H 4600 3250 50  0001 C CNN
	1    4600 3250
	1    0    0    -1  
$EndComp
$Comp
L HR2:WOW_OUT;M2x6S CN8
U 1 1 5F848F57
P 9600 4600
F 0 "CN8" H 9800 4750 50  0000 L CNN
F 1 "WOW_OUT;M2x6S" H 9750 3350 50  0000 L CNN
F 2 "Connector_PinHeader_2.54:PinHeader_2x06_P2.54mm_Vertical" H 9800 4650 60  0001 L CNN
F 3 "WOW Output Connector" H 9800 4450 60  0001 L CNN
F 4 "WOW Output Connector" H 9800 4350 60  0001 L CNN "Field5"
	1    9600 4600
	1    0    0    -1  
$EndComp
$Comp
L HR2:MCP2542;SOIC8 U15
U 1 1 5F8496E2
P 5400 3900
F 0 "U15" H 5650 4050 50  0000 C CNN
F 1 "LTV-827" H 5950 3350 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5600 3950 60  0001 L CNN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 5600 3750 60  0001 L CNN
F 4 "MCP2542" H 5600 3650 60  0001 L CNN "Field5"
	1    5400 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2500 5300 2900
Wire Wire Line
	5300 4200 5400 4200
$Comp
L Isolator:LTV-827 U17
U 2 1 5F84B285
P 7400 3100
F 0 "U17" H 7250 3300 50  0000 C CNN
F 1 "LTV-827" H 7400 2900 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 7200 2900 50  0001 L CIN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 7400 3100 50  0001 L CNN
	2    7400 3100
	1    0    0    -1  
$EndComp
$Comp
L Isolator:LTV-827 U18
U 2 1 5F84B28F
P 8400 3100
F 0 "U18" H 8550 2900 50  0000 C CNN
F 1 "LTV-827" H 8400 3300 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 8200 2900 50  0001 L CIN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20201610%20.pdf" H 8400 3100 50  0001 L CNN
	2    8400 3100
	-1   0    0    1   
$EndComp
Wire Wire Line
	7100 3200 7000 3200
Wire Wire Line
	7000 3400 9100 3400
Wire Wire Line
	7000 3200 7000 3400
$Comp
L Device:R_US R42
U 1 1 5F84B29E
P 8000 3650
F 0 "R42" H 7850 3750 50  0000 L CNN
F 1 "120Ω1/4W;1608" H 8050 3550 50  0000 L CNN
F 2 "" V 8040 3640 50  0001 C CNN
F 3 "~" H 8000 3650 50  0001 C CNN
	1    8000 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 3900 7800 3900
Wire Wire Line
	8000 3500 8000 3200
Wire Wire Line
	6500 4000 8000 4000
Wire Wire Line
	8000 4000 8000 3800
Wire Wire Line
	7800 3900 7800 3000
Wire Wire Line
	8700 3200 9100 3200
Wire Wire Line
	9100 3200 9100 3400
Wire Wire Line
	7700 3200 8000 3200
Wire Wire Line
	7700 3000 7800 3000
Connection ~ 8000 3200
Wire Wire Line
	8000 3200 8100 3200
Connection ~ 7800 3000
Wire Wire Line
	7800 3000 8100 3000
Wire Wire Line
	7800 3900 8900 3900
Connection ~ 7800 3900
Wire Wire Line
	8000 4000 8800 4000
Connection ~ 8000 4000
Connection ~ 8000 2300
Connection ~ 7800 2200
Wire Wire Line
	4500 1000 4400 1000
Wire Wire Line
	4400 1000 4400 1450
Text HLabel 1100 3000 0    50   Input ~ 0
3.3V
Wire Wire Line
	6800 2500 6500 2500
Wire Wire Line
	6500 4200 6800 4200
Wire Wire Line
	8700 1300 9400 1300
Text HLabel 1100 1600 0    50   Input ~ 0
5V
Wire Wire Line
	1100 1600 6600 1600
Wire Wire Line
	6600 1600 6600 2400
Wire Wire Line
	6600 2400 6500 2400
Wire Wire Line
	6500 4100 6600 4100
Wire Wire Line
	6600 4100 6600 2400
Connection ~ 6600 2400
Wire Wire Line
	6500 4300 6900 4300
Wire Wire Line
	6800 1300 7100 1300
Connection ~ 6800 2500
Wire Wire Line
	7100 3000 6800 3000
Connection ~ 6800 3000
Wire Wire Line
	6800 3000 6800 2500
Wire Wire Line
	6900 5900 6900 5300
Connection ~ 6900 4300
Wire Wire Line
	6900 2600 6900 4300
Connection ~ 4600 5900
$Comp
L Device:R_US R35
U 1 1 5F98DD9D
P 4000 5650
F 0 "R35" H 3850 5750 50  0000 L CNN
F 1 "100KΩ;1608" H 4000 5500 50  0000 L CNN
F 2 "" V 4040 5640 50  0001 C CNN
F 3 "~" H 4000 5650 50  0001 C CNN
	1    4000 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 1300 6800 2500
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F9A0944
P 3700 4200
AR Path="/5F79BC00/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5F9A0944" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5F9A0944" Ref="Q4"  Part="1" 
F 0 "Q4" H 3600 4350 50  0000 L CNN
F 1 "2N7000;SOT23" H 3850 4050 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3900 4125 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 3700 4200 50  0001 L CNN
	1    3700 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1450 3800 4000
Text HLabel 1100 2300 0    50   Input ~ 0
WOW_TX
Text HLabel 1100 2400 0    50   Output ~ 0
WOW_RX
Wire Wire Line
	5400 2300 1100 2300
Wire Wire Line
	1100 2400 5400 2400
Wire Wire Line
	9400 1300 9400 1400
$Comp
L Device:R_US R60
U 1 1 5F8EAE9D
P 9400 1550
F 0 "R60" H 9250 1650 50  0000 L CNN
F 1 "100KΩ;1608" H 8900 1400 50  0000 L CNN
F 2 "" V 9440 1540 50  0001 C CNN
F 3 "~" H 9400 1550 50  0001 C CNN
	1    9400 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 1700 9400 2400
$Comp
L HR2:74LVC1G74;TSSOP8 U16
U 1 1 5F8C5AF5
P 5400 5000
F 0 "U16" H 5650 5150 50  0000 C CNN
F 1 "74LVC1G74;TSSOP8" H 5950 4550 50  0000 C CNN
F 2 "Package_SO:TSSOP-8_3x3mm_P0.65mm" H 5600 5050 60  0001 L CNN
F 3 "D-type flip-flop edge-trigger/set/reset" H 5600 4850 60  0001 L CNN
F 4 "D-type flip-flop edge-trigger/set/reset" H 5600 4750 60  0001 L CNN "Field5"
	1    5400 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5000 6500 5000
Connection ~ 6600 4100
Wire Wire Line
	6500 5300 6900 5300
Connection ~ 6900 5300
Wire Wire Line
	6900 5300 6900 4300
Wire Wire Line
	6700 6000 1100 6000
Wire Wire Line
	6800 6100 1100 6100
Text HLabel 1100 6100 0    50   Output ~ 0
ESTOP
Text HLabel 1100 6000 0    50   Output ~ 0
~ESTOP
Text HLabel 1100 5300 0    50   Input ~ 0
~ESTOP_CLR
Text HLabel 1100 5000 0    50   Input ~ 0
~NRST
Wire Wire Line
	5300 5100 5400 5100
Text Notes 7200 5600 0    50   ~ 0
Red ESTOP LED
Text Notes 5200 2000 0    50   ~ 0
Communication CANBus Transceiver
Text Notes 5550 3700 0    50   ~ 0
ESTOP CANBus Transceiver
Text Notes 4550 700  0    50   ~ 0
WOW Power P-FET (6A)
Text Notes 5450 4800 0    50   ~ 0
E-Stop x74 D Style Flip-Flop
Text Notes 6950 1100 0    50   ~ 0
Opto-isolator
Text Notes 7600 2850 0    50   ~ 0
Dual Opto-isolator
Text Notes 9500 4400 0    50   ~ 0
WOWBus Output Connector
Wire Wire Line
	3400 4200 3400 4500
Wire Wire Line
	3400 4200 3500 4200
$Comp
L Device:R_US R36
U 1 1 5FC61F4D
P 2300 3250
F 0 "R36" H 2150 3350 50  0000 L CNN
F 1 "100KΩ;1608" H 2300 3100 50  0000 L CNN
F 2 "" V 2340 3240 50  0001 C CNN
F 3 "~" H 2300 3250 50  0001 C CNN
	1    2300 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3100 2300 3000
Connection ~ 2300 3000
$Comp
L Device:LED D?
U 1 1 5FC8C292
P 7100 5450
AR Path="/5F79BC00/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5FA4A874/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5FAD82EE/5FC8C292" Ref="D?"  Part="1" 
AR Path="/5F8C2F57/5FC8C292" Ref="D2"  Part="1" 
F 0 "D2" V 7200 5550 50  0000 R CNN
F 1 "REDLED;1608" V 7050 5350 50  0000 R CNN
F 2 "HR2:LED_GRNRA" H 7100 5450 50  0001 C CNN
F 3 "~" H 7100 5450 50  0001 C CNN
	1    7100 5450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R40
U 1 1 5FC946D0
P 7100 5050
F 0 "R40" H 6950 5150 50  0000 L CNN
F 1 "470Ω;1608" H 7150 4950 50  0000 L CNN
F 2 "" V 7140 5040 50  0001 C CNN
F 3 "~" H 7100 5050 50  0001 C CNN
	1    7100 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 5900 7100 5900
Wire Wire Line
	7100 5900 7100 5600
Connection ~ 6900 5900
Wire Wire Line
	6800 4800 7100 4800
Wire Wire Line
	7100 4800 7100 4900
Text Label 8800 2200 0    50   ~ 0
WOW_COMH
Text Label 8800 2300 0    50   ~ 0
WOW_COML
Text Label 7100 3900 0    50   ~ 0
WOW_STOPH
Text Label 7100 4000 0    50   ~ 0
WOW_STOPL
Text Label 1100 900  0    50   ~ 0
9V
Text Label 1100 1600 0    50   ~ 0
5V
Text Label 1100 2300 0    50   ~ 0
WOW_TX
Text Label 1100 2400 0    50   ~ 0
WOW_RX
Text Label 1100 3000 0    50   ~ 0
3.3V
Text Label 1100 4500 0    50   ~ 0
WOW_EN
Text Label 1100 5300 0    50   ~ 0
~ESTOP_CLR
Text Label 1100 5000 0    50   ~ 0
~NRST
Text Label 1100 6000 0    50   ~ 0
~ESTOP
Text Label 1100 6100 0    50   ~ 0
ESTOP
Text Label 1100 5900 0    50   ~ 0
GND
Text Label 5000 4600 0    50   ~ 0
~ESTOP
Text Label 4800 3900 0    50   ~ 0
WOW_STDBY
Connection ~ 3400 4500
Text Label 7900 1000 0    50   ~ 0
WOW_POW_OUT
Text Label 8200 1700 0    50   ~ 0
WOW_COM_TERM
Text Label 8350 3400 0    50   ~ 0
WOW_ESTOP_TERM
Wire Wire Line
	5300 2900 6500 2900
Wire Wire Line
	6500 2900 6500 2700
Wire Wire Line
	6500 2700 7000 2700
Connection ~ 5300 2900
Wire Wire Line
	5300 2900 5300 3900
Text Label 8700 3000 0    50   ~ 0
WOW_ESTOP_OPTO
Text Label 8800 1300 0    50   ~ 0
WOW_COM_OPTO
Text Notes 600  7600 0    50   ~ 0
WOWBus Notes:\n\n1. WOWBus is an an expansion bus.\n2. WOWBus stands for Wayne's Omnipotent Wonderful Bus (heavy sarcasm.)\n3. WOWBus uses 2x6 ribbon daisy chain cables for interconnect.\n4. WOWBus has a nominal 9V rail, but this can droop due to resistive losses down the daisy chain.\n5. The WOWBus connector is designed to mirror image power, ground, and signaling.\n6. If a WOWBus connector is accidently reversed on a daisy cable, nothing shorts out.\n7. However, an accidentally reversed cable, will basically make the bus non-fuctional until corrected.\n8. The WOWBus has two "CAN bus" (ISO-11898-2) physical differential pairs for siginalling.\n9. The first differential pair is used as a multi-drop bidirectional data transfer.\n10. The second differential pair is used to signal Emergency Stop.\n11. When the WOWBus is enabled, it powers up both the transceivers and the differential\n    pair termination resistors.\n12. The expansions boards are responsible for providing the bus terminator resistors for\n    for the other end of the differential pairs.\n13. The Bus powers up in the EStop conndition and must be explicitly cleared out.\n14. When ESTOP occurs, it latches up and must be explicitly cleared.\n
Connection ~ 5300 3900
Wire Wire Line
	5300 3900 5300 4200
Wire Wire Line
	5000 4600 5000 4100
Wire Wire Line
	5000 4100 5400 4100
Wire Wire Line
	4600 3400 4600 3900
Connection ~ 4600 3900
Wire Wire Line
	4600 3900 5300 3900
Text HLabel 1100 4900 0    50   Input ~ 0
~MOTOR_FAULT
Text Label 1100 4900 0    50   ~ 0
~MOTOR_FAULT
Wire Wire Line
	4600 5900 4000 5900
Wire Wire Line
	5300 4700 5300 5100
Wire Wire Line
	6600 4700 6600 5000
Wire Wire Line
	6600 4100 6600 4700
Connection ~ 6600 4700
Wire Wire Line
	6600 4700 5300 4700
Text Label 4850 5000 0    50   ~ 0
~ESTOP_SET
Text Label 4850 5200 0    50   ~ 0
ESTOP_TRIG
Connection ~ 2300 4900
Wire Wire Line
	2300 3400 2300 4900
Wire Wire Line
	1100 4900 2300 4900
Wire Wire Line
	2300 4900 3000 4900
Wire Wire Line
	7100 5200 7100 5300
Wire Wire Line
	4600 5900 6900 5900
Wire Wire Line
	4600 3900 4600 4300
Wire Wire Line
	3400 4500 4000 4500
Wire Wire Line
	4600 4700 4600 5900
$Comp
L Transistor_FET:2N7000 Q?
U 1 1 5F86962A
P 4500 4500
AR Path="/5F79BC00/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5FA4A874/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5FAD82EE/5F86962A" Ref="Q?"  Part="1" 
AR Path="/5F8C2F57/5F86962A" Ref="Q5"  Part="1" 
F 0 "Q5" H 4400 4650 50  0000 L CNN
F 1 "2N7000;SOT23" H 4000 4300 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4700 4425 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 4500 4500 50  0001 L CNN
	1    4500 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3000 6800 3000
Connection ~ 4600 3000
Wire Wire Line
	4600 3000 4600 3100
Wire Wire Line
	6800 3000 6800 4200
$Comp
L Switch:SW_Push SW?
U 1 1 5FC73CCB
P 2500 5700
AR Path="/5F52F39E/5FC73CCB" Ref="SW?"  Part="1" 
AR Path="/5F8C2F57/5FC73CCB" Ref="SW3"  Part="1" 
F 0 "SW3" H 2350 5800 50  0000 C CNN
F 1 "PUSH_BUTTON;6x6x9.5H" H 2650 5600 50  0000 C CNN
F 2 "" H 2500 5900 50  0001 C CNN
F 3 "~" H 2500 5900 50  0001 C CNN
	1    2500 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 5200 4800 5200
Wire Wire Line
	5400 4000 4800 4000
Wire Wire Line
	4800 4000 4800 5200
Wire Wire Line
	3800 4400 3800 5900
Connection ~ 3800 5900
$Comp
L Device:R_US R62
U 1 1 5FF6B902
P 2800 3250
F 0 "R62" H 2650 3350 50  0000 L CNN
F 1 "100KΩ;1608" H 2800 3100 50  0000 L CNN
F 2 "" V 2840 3240 50  0001 C CNN
F 3 "~" H 2800 3250 50  0001 C CNN
	1    2800 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 3100 2800 3000
Connection ~ 2800 3000
Wire Wire Line
	3800 1450 3800 1350
Connection ~ 3800 1450
$Comp
L 74xGxx:74LVC1G11 U13
U 1 1 5FFD471D
P 3300 5000
F 0 "U13" H 3200 5200 50  0000 C CNN
F 1 "74LVC1G11" H 3550 4800 50  0000 C CNN
F 2 "" H 3300 5000 50  0001 C CNN
F 3 "http://www.ti.com/lit/sg/scyt129e/scyt129e.pdf" H 3300 5000 50  0001 C CNN
	1    3300 5000
	1    0    0    -1  
$EndComp
Text Notes 3300 4850 0    40   ~ 0
5
Text Notes 3250 5200 0    40   ~ 0
2\n
Wire Wire Line
	4000 5500 4000 4500
Connection ~ 4000 4500
Wire Wire Line
	4000 4500 4300 4500
Wire Wire Line
	4000 5800 4000 5900
Connection ~ 4000 5900
Wire Wire Line
	4000 5900 3800 5900
Wire Wire Line
	2300 3000 2800 3000
Wire Wire Line
	2800 3400 2800 5100
Wire Wire Line
	3000 5100 2800 5100
Wire Wire Line
	1100 4500 3400 4500
Wire Wire Line
	5400 5000 3600 5000
Wire Wire Line
	1100 5000 3000 5000
Wire Wire Line
	1100 5300 5400 5300
Wire Wire Line
	1100 5900 2100 5900
Wire Wire Line
	2800 5100 2800 5700
Wire Wire Line
	2800 5700 2700 5700
Connection ~ 2800 5100
Wire Wire Line
	2300 5700 2100 5700
Wire Wire Line
	2100 5700 2100 5900
Connection ~ 2100 5900
Wire Wire Line
	2100 5900 3300 5900
Wire Wire Line
	3300 5125 3300 5900
Connection ~ 3300 5900
Wire Wire Line
	3300 5900 3800 5900
Wire Wire Line
	2800 3000 3300 3000
Wire Wire Line
	3300 4875 3300 3000
Connection ~ 3300 3000
Wire Wire Line
	3300 3000 4600 3000
Text Notes 2400 5550 0    50   ~ 0
Manual\nE-Stop\nButton
Text Notes 2850 4750 0    50   ~ 0
E-Stop Set\n3-Input\nAND Gate
$Comp
L Device:C C?
U 1 1 6036DBC8
P 1550 4000
AR Path="/5F48CAAD/6036DBC8" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6036DBC8" Ref="C16"  Part="1" 
F 0 "C16" V 1450 3800 50  0000 L CNN
F 1 "10pF;1608" V 1650 4050 50  0000 L CNN
F 2 "" H 1588 3850 50  0001 C CNN
F 3 "~" H 1550 4000 50  0001 C CNN
	1    1550 4000
	0    1    1    0   
$EndComp
Text Notes 1400 3200 0    50   ~ 0
Bypass Capacitors
$Comp
L Device:C C?
U 1 1 60430375
P 1550 4300
AR Path="/5F48CAAD/60430375" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/60430375" Ref="C17"  Part="1" 
F 0 "C17" V 1450 4100 50  0000 L CNN
F 1 "10pF;1608" V 1600 4350 50  0000 L CNN
F 2 "" H 1588 4150 50  0001 C CNN
F 3 "~" H 1550 4300 50  0001 C CNN
	1    1550 4300
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 6043088F
P 1550 3700
AR Path="/5F48CAAD/6043088F" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6043088F" Ref="C15"  Part="1" 
F 0 "C15" V 1450 3500 50  0000 L CNN
F 1 "10pF;1608" V 1600 3750 50  0000 L CNN
F 2 "" H 1588 3550 50  0001 C CNN
F 3 "~" H 1550 3700 50  0001 C CNN
	1    1550 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 3000 1300 3000
Wire Wire Line
	1400 4300 1300 4300
Wire Wire Line
	1300 4300 1300 4000
Connection ~ 1300 3000
Wire Wire Line
	1300 3000 2300 3000
Wire Wire Line
	1400 3700 1300 3700
Connection ~ 1300 3700
Wire Wire Line
	1300 3700 1300 3400
Wire Wire Line
	1400 4000 1300 4000
Connection ~ 1300 4000
Wire Wire Line
	1300 4000 1300 3700
Wire Wire Line
	2100 5700 2100 4300
Wire Wire Line
	2100 4300 1700 4300
Connection ~ 2100 5700
Wire Wire Line
	1700 4000 2100 4000
Wire Wire Line
	2100 4000 2100 4300
Connection ~ 2100 4300
Wire Wire Line
	1700 3700 2100 3700
Wire Wire Line
	2100 3700 2100 4000
Connection ~ 2100 4000
Text Notes 3850 4050 0    50   ~ 0
3.3 to 9V\nLevel Shift
Text Notes 4050 4900 0    50   ~ 0
WOW Standby\nInverter
Text Label 3800 1450 0    50   ~ 0
~WOW_EN
$Comp
L Device:C C?
U 1 1 6057FE20
P 1550 3400
AR Path="/5F48CAAD/6057FE20" Ref="C?"  Part="1" 
AR Path="/5F8C2F57/6057FE20" Ref="C14"  Part="1" 
F 0 "C14" V 1450 3200 50  0000 L CNN
F 1 "10pF;1608" V 1600 3450 50  0000 L CNN
F 2 "" H 1588 3250 50  0001 C CNN
F 3 "~" H 1550 3400 50  0001 C CNN
	1    1550 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	2100 3700 2100 3400
Wire Wire Line
	2100 3400 1700 3400
Connection ~ 2100 3700
Wire Wire Line
	1400 3400 1300 3400
Connection ~ 1300 3400
Wire Wire Line
	1300 3400 1300 3000
Wire Wire Line
	9400 3000 9400 2850
$Comp
L Device:R_US R43
U 1 1 5F8E9E6C
P 9400 2700
F 0 "R43" H 9250 2800 50  0000 L CNN
F 1 "100KΩ;1608" H 8900 2550 50  0000 L CNN
F 2 "" V 9440 2690 50  0001 C CNN
F 3 "~" H 9400 2700 50  0001 C CNN
	1    9400 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5200 6700 4600
Wire Wire Line
	6700 5200 6500 5200
Wire Wire Line
	6700 4600 5000 4600
Wire Wire Line
	6700 5200 6700 6000
Connection ~ 6700 5200
Wire Wire Line
	6500 5100 6800 5100
Wire Wire Line
	6800 4800 6800 5100
Wire Wire Line
	6800 5100 6800 6100
Connection ~ 6800 5100
Wire Wire Line
	8800 4000 8800 5300
Wire Wire Line
	8800 5300 9600 5300
Wire Wire Line
	9600 5200 8900 5200
Wire Wire Line
	8900 5200 8900 3900
Wire Wire Line
	9600 5000 9200 5000
Wire Wire Line
	9200 5000 9200 4000
Wire Wire Line
	9200 4000 9600 4000
Wire Wire Line
	9600 4000 9600 2300
Wire Wire Line
	8000 2300 9600 2300
Wire Wire Line
	9600 5100 9100 5100
Wire Wire Line
	9100 5100 9100 3900
Wire Wire Line
	9100 3900 9500 3900
Wire Wire Line
	9500 3900 9500 2200
Wire Wire Line
	7800 2200 9500 2200
Wire Wire Line
	7100 5900 9500 5900
Wire Wire Line
	9500 5900 9500 5500
Wire Wire Line
	9500 5500 9600 5500
Connection ~ 7100 5900
Wire Wire Line
	9600 5400 9500 5400
Wire Wire Line
	9500 5400 9500 5500
Connection ~ 9500 5500
Wire Wire Line
	9500 5400 9500 4900
Wire Wire Line
	9500 4900 9600 4900
Connection ~ 9500 5400
Wire Wire Line
	9500 4900 9500 4800
Wire Wire Line
	9500 4800 9600 4800
Connection ~ 9500 4900
Wire Wire Line
	9600 5700 9400 5700
Wire Wire Line
	9400 5700 9400 5600
Wire Wire Line
	9400 5600 9600 5600
Wire Wire Line
	9400 5600 9400 4700
Wire Wire Line
	9400 4700 9600 4700
Connection ~ 9400 5600
Wire Wire Line
	9400 4700 9400 4600
Wire Wire Line
	9400 4600 9600 4600
Connection ~ 9400 4700
Wire Wire Line
	9400 4600 9400 4200
Wire Wire Line
	9400 4200 9800 4200
Connection ~ 9400 4600
Wire Wire Line
	5300 1000 9800 1000
Wire Wire Line
	9800 1000 9800 4200
Wire Wire Line
	7000 2700 7000 2400
Wire Wire Line
	7000 2400 9400 2400
Connection ~ 9400 2400
Wire Wire Line
	9400 2400 9400 2550
Wire Wire Line
	8700 3000 9400 3000
$Comp
L HR2:POWER_PFET_GSD Q6
U 1 1 5F9BEDD5
P 4500 900
F 0 "Q6" H 4750 1050 50  0000 C CNN
F 1 "PFET_6A_GSD;SOT23" H 4900 650 50  0000 C CNN
F 2 "HR2:POWER_PFET_GSD" H 4700 950 60  0001 L CNN
F 3 "Power PFG (GSD pinout)" H 4700 750 60  0001 L CNN
F 4 "Power PFET (GSD pinout)" H 4700 650 60  0001 L CNN "Field5"
	1    4500 900 
	1    0    0    -1  
$EndComp
$EndSCHEMATC
