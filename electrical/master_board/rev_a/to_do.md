# HR2 TO DO list

## Misc. Links

* libreoffice --headless --convert-to csv ./jlcpcb_smt_parts_library.xls --outdir /tmp/
* https://eepower.com/resistor-guide/resistor-standards-and-codes/resistor-sizes-and-packages/#
* https://support.jlcpcb.com/article/84-how-to-generate-the-bom-and-centroid-file-from-kicad
* https://docs.google.com/spreadsheets/d/1bSM_Ko6OLz7rW6MCQbw4Zv_Z4hOHcI7w7O6sczi2h5w/edit#gid=0

## Encoder:

* (DONE) 100Ω resistors should be 100KΩ.

## Power Supply

* (DONE) Downsize the 3.3V regulator to a SOT23 Package.
  * Find a SOT23 3.3V regulator that JLCPCB supports. YES.
  * Move 3.3V regulator closer to users of 3.3V regulators? YES.
  * Use 3.3V from Nucleo instead of regulator? Probably not. NO.
* (DONE) Are LED resistors the right number of Ohms and wattage?  3.3V and SBC_ALIVE?
* (DONE) Are Shunt resistor packages big enough?

## WOWBus

* (DONE) Update WOWBus schematic.
* (DONE) Resize 120Ω Resistors (requires .2W, prefer .25W)? 3216.
* (DONE) Are solid state relay current limit resistors (R61, R62) are currently 100KΩ;1608.
  * Resized 300Ω;1608.
* (DONE) Route LED (R58, D3) to be off of NESTOP.
* (DONE) Is LED current limit (R58) properly sized?

## SBC

* (DONE) Is NESTOP connected to SBC?
* Are the I2C resistors the correct value?
  One is 3.9KΩ and the other is 4.7KΩ.  They should probably be the same.
* (DONE) Is the SBC Alive LED current limit resistor the right value?
* (DONE) Does JLCPCB support surface mount buttons? 6x6mm
* (DONE) Does JLCPCB have a 32.768kHz crystal?
* (DONE) Update 12.5pF load in schematic symbol.

## Motors_Encoders

* (DONE) Does nSLEEP need a pull up/down?  Nod) Does nFAULT need a pull up/down?  Yes, 100K pull-up present
* (DONE) Are bypass capacitors C11 and C12, 10µF and 2.2µF (C11 and C12)? Yes.
* (DONE) Resize 10K resistors to 1005 (i.e. 0402)?
* (DONE) Is NSLEEP routed to NESTOP?

## Nucleo144

* Figure out why 3.3V is not in PCB footprint (KiCube problem?)
* Switch PD7 to NESTOP in schematic symbol.

## LED's:

* (DONE) Are current limit resistors the right size? Yes, 1005.
* (DONE) Are transistors necessary? Yes.
* (DONE) Add a private ground return for the LED's with a jumper to disable?
  * (DONE)This could go on the outer rim of the Ground plane?  It is "Ground like".
  * Add a NeoPixel connector? NO

## Servos and Sonars

* (DONE) Add the low pass filter to the current sense.
* (DONE) Clean up schematic sheet to remove white space where voltage dividers were.

## Connectors

* (DONE) Make sure that 165's are 5V tolerant for sense connectors.
  * Nexperia says yes (15V). This is the part available from JLCPCB.
* (DONE) Add Lidar id pins to lidar connector.
* (DONE) Add pull-up resistors to LDR_ID0, LDR_ID1, LDR_ID2.
* (DONE) Route LDR_ID0, LDR_ID1, LDR_ID2 to U7
* (DONE) Route NWOW_ESTOP, NMOTOR_FAULT, and NBTN_ESTOP to U7
* (DONE) The remaining bits are a PCB revision (00=>A, 01=>B, 10=>C, 11=>D).

## Misc.

* (DONE) Recenter the PCB center to (100, 100)
* Verify HTSSOP vs TTSOP
* The 2x6 shrouded connector is missing one side of shroud.
* Clean up traces.
* Finish ground stitching.
* Verify that driver chip footprint is truly ground.
* Reannote the PCB.
* Add board name, revision, date, etc to artwork.
