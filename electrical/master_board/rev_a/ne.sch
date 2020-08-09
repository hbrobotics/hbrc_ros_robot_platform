EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "HR2 NE PCB"
Date "2020-06-13"
Rev "A"
Comp "Homebrew Robotics Club"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
NoConn ~ 5500 3250
NoConn ~ 5500 3350
NoConn ~ 5500 3450
NoConn ~ 5500 3550
$Comp
L HR2:HCSR04H;F1X4 CN60
U 1 1 5EE6BF38
P 4800 3250
F 0 "CN60" H 5100 3400 50  0000 C CNN
F 1 "HCSR04;F1x4H" H 5150 2800 50  0000 C CNN
F 2 "HR2:HCSR04_F1x4H" H 5000 3300 60  0001 L CNN
F 3 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5000 3100 60  0001 L CNN
F 4 "https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf" H 5000 3200 60  0001 L CNN "manf#"
F 5 "HC-SR04 Sonar (High Profile Receptacle)" H 5000 3000 60  0001 L CNN "Field5"
	1    4800 3250
	1    0    0    -1  
$EndComp
$Comp
L HR2:HOLE;M2.5 H5
U 1 1 5F2F7E7C
P 1800 4850
F 0 "H5" H 1700 4900 60  0000 L CNN
F 1 "HOLE;M2.5" H 1550 4800 60  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 2000 4900 60  0001 L CNN
F 3 "M2.5 Mounting Hole" H 2000 4700 60  0001 L CNN
	1    1800 4850
	1    0    0    -1  
$EndComp
$Comp
L HR2:HOLE;M2.5 H1
U 1 1 5F2F85AA
P 1800 4550
F 0 "H1" H 1700 4600 60  0000 L CNN
F 1 "HOLE;M2.5" H 1550 4500 60  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 2000 4600 60  0001 L CNN
F 3 "M2.5 Mounting Hole" H 2000 4400 60  0001 L CNN
	1    1800 4550
	1    0    0    -1  
$EndComp
$Comp
L HR2:GROVE;20x20 GV2
U 1 1 5F2F9731
P 1800 4250
F 0 "GV2" H 1700 4300 60  0000 L CNN
F 1 "GROVE;20x20" H 1500 4200 60  0000 L CNN
F 2 "HR2:GROVE20x20" H 2000 4300 60  0001 L CNN
F 3 "Grove 20x20 Module" H 2000 4100 60  0001 L CNN
F 4 "Grove 20x20 Module" H 2000 4200 60  0001 L CNN "manf#"
	1    1800 4250
	1    0    0    -1  
$EndComp
$EndSCHEMATC
