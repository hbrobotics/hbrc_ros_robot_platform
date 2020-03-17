EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 8 9
Title "Raspberry Pi / Real Time Clock"
Date "2020-03-16"
Rev "Rev. A"
Comp "HBRC ROS Robot (HR2) Platform"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 3950 2750 2    50   Output ~ 0
PI_TX
Text HLabel 3950 2900 2    50   Input ~ 0
PI_RX
Text HLabel 3950 3050 2    50   Output ~ 0
HALTED
Text HLabel 3950 3200 2    50   Input ~ 0
HALT_REQ
Text Notes 2300 4600 0    50   ~ 0
Notes:\n1. The power module can request that the Pi halt itself.\n2. The RTC is connected to one I2C pair of the Pi.\n3. The other I2C is connected to the Hat EEPROM.\n4. Other pins may eventually be connected to various peripherals.
$EndSCHEMATC
