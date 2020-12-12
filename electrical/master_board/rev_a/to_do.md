# HR2 TO DO list

## Power Supply

* Downsize the 3.3V regulator to a SOT23 Package.
  * Find a SOT23 3.3V regulator that JLCPCB supports.
  * Move 3.3V regulator closer to users of 3.3V regulators?
  * Use 3.3V from Nucleo instead of regulator? Probably not.
* Are LED resistors the right number of Ohms and wattage?
* Are Shut resistor packages big enough? NO: Schematic says 1608. PCB uses 3216 

## WOWBus

* (DONE) Update WOWBus schematic.
* Resize 120Ω Resistors (requires .2W, prefer .25W)?:
  Does 0603 support .25W?
* Are solid state relay current limit resistors (R61, R62) are currently 100KΩ;1608.
  * Are the right value for Ohms to limit current?
  * Are the sized right, or can theye be down-sized?

## SBC

* Is nESTOP connected?
  Not yet.
* Are the I2C resistors the correct value?
  One is 3.9KΩ and the other is 4.7KΩ.  They should probably be the same.
* Is the SBC Alive LED current limit resistor the right value?
  Currently it is 470Ω in a 1608 (0603) package.
* Does JLCPCB support surface mount buttons?
  If so, change to that part and package footprint.
* Does JLCPCB have a 32.768kHz crystal?
  If so, change to that part and package footprint.

## Motors_Encoders

* (DONE) Does nSLEEP need a pull up/down?  No
* (DONE) Does nFAULT need a pull up/down?  Yes, 10K pull-up present
* Are bypass capacitors are 2.2µF and 10µF (C11 and C12)? No! C11 is 10pF!
* Resize 10K resistors to 1005 (i.e. 0402)?
* Is nSLEEP connected to nESTOP?
* Move LED (R58,D3) to be off of nESTOP.
* Is LED current limit (R58) properly sized?

## ST_link

## Nucleo144

* Figure out why 3.3V is not in PCB footprint (KiCube problem?)
* Switch PD7 to NESTOP in schematic symbol.

## LED's:

* Are current limit resistors the right size?
* Are transistors necessary?
* Add a NeoPixel connector?
  * Maybe add a private ground return for the LED's with a jumper to disable?
    This could go on the outer rim of the Ground plane?  It is "Ground like"
  * Alternatively, add a gate to disable the clock line to the shift registers.

## Servos and Sonars

* Add the low pass filter to the current sense.
* Clean up schematic sheet to remove white space where voltage dividers were.

## Connectors

* Make sure that 165's are 5V tollerant for sense connectors.
* Add Lidar id pins to lidar connector.
* Route Lidar id pins to U7.
* Add nWOW_ESTOP, nMOTOR_FAULT, and nBTN_ESTOP to U7
* The remaining bits are a PCB revision (00=>A, 01=>B, 10=>C, 11=>D).

## Encoder Boards

* Do a design Review.

## STAdatper Board.

* Do a design review.

## Lidar Adapater Board.

* Bang one or two out?
