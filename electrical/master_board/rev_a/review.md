# HR2 Review Notes

## General

* Do a 4 layer board.:

  * Top inner layer is ground.
  * Bottom inner layer is Power distribution.
  * Use a region based strategy for power distribution.

* Start working on BOM:

## Power

* Convert to USB-C.
* Configure to get 3A at 5V.
* Replace PFET with correct PFET symbol.
* Add filter capacitor for U1.
* Upgrade 0Ω resistors to larger footprint. (1206's)
* Select an actual PFET for Q2.
* Sprinkle in 1µF Capacitors in various places

## WOWBus

* Ask Kyle for recommended smaller package for small signel inverter FET's. (Q20/Q22)
* Add gate to "OR" in ~ESTOP to ~nWOW_ESTOP.
* Put in better symbol for Q21.
* Make R64 be in a larger package.
* Make sure there are enough filter capacitors.

## SBC

* Replace Q23 with a better symbol.
* Use a surface mount 40-pin connector.

## Motors and Encoders

* Make sure that there are filter capacitors on the encoder boards.
* Remove "250Ω  .4W Current Sense"

## ST-Link



* No comments.

## Nucleo-144

* Fix a few symbols in the STM32Cube to have new labels.
* Move Q29 inverter to Connectors page
* Use surface mount connectors.

## LED's

* In the future (Rev. B) use better LED Drivers.
* https://youtu.be/ssHkMWcGat4

## Sonars and Servos

* Consider removing Echo level shift resistors
* Think about removing U10 level shefter.
* Add an RC filter to VOUT of U10.

## Connectors

Sullins: Connector_Samtec_HLE_SMD: Samtec_HLE-120-02-xxx-DV-LC_2x20_P2.54mm_Horizontal
* Nuke Mikrobus Connectors (This frees up INT0 and INT1)
* U7 DIO_SCK and DIO_SS are swapped.
* Remove DIO_OUT_TO_IN wire.
* Hook up ESTOP signals to U7 (MOTOR_FAULT, ESTOP BUTTON, WOW_ESTOP, and ESTOP.)

