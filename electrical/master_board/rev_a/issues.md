<!-- ===================================== 100 characters ====================================== -->

# Master Board Rev. A Issues

1.  JLCPCB had issues with C1 footprint.  They were soldered in anyhow.
2.  JLCPCB had issues with SW1/SW2 footprint.  They were soldered in anyhow.
3.  JP1 and JP8 have simliar labels.  One is for the Ring LED's and the
    other is for the power indicator LED's.
4.  The encoder M1x2RA pins are offset by 1.in.
    Both need to be offset outward by 1. in.
5.  The assembly orders is:
    1. Empty motor holder installed into Romi Base.
    2. Raspberry Pi bolted to Romi Base.
    3. Master Board is plugged onto Romi Base and bolted onto
       appropriate stand-offs.
    4. The encoder board is soldered to the motor gear motor.
    5. The encoder/gearmotor sub-assembly is slide into the motor holder.

    The issue is that the Master PCB is too high to allow the
    motor holder tabs to deflect.
6.  The Pololu regulators problems:
    1. They are 12V instead of 9V.
7.  Power Indicator LED's are way too bright.
8.  The JLCPCB order of the ideal diodes got some part other than the LM5050.
9.  The LM5050 does not guarantee to work under 5V, so we may need to
    find another component.
10. There needs to be a little more clearance for the encoder magnets.
11. There is no clearance for the WOW connector on top.
    Move to a different location.
    For now, mount on the bottom.
12. There are solder bumps for the ST-link board.
13. Investigate through hole connectors for Sonars.
14. Get shorter eletrolytic capacitors.  For now tilt them on edge.
15. ST-Link...
16. WOW connector...
17. LD06 ...
18. Bottom Sonars...
19. Top Sonar ...
20. Boost converter clearance ...
21. Capacitors ...
22. The 5 top sonar footprints are all rototed 180 degrees and point the wrong directions.
    The 2 bottom sonar footprints are correct.
23. Wheel clearnce is tight.  The Nucleo spacers need to be lifted up by about 3/16 inch.
    In other words -- 1/2" plus 3/16".
24. The motor driver needs a .1µF capicitor between between pin-11 (VCP) and pin-12 (VM).
25. The ESTOP LED is inverted.  It should be ON for an ESTOP not OFF.
26. The RxD and TxD pins on U19 are swapped.
27. There is something weird going on with power coming in from the STLink USB connector
    event when the Nucleo is jumpered for E5V.
28. The right encoders is flipped and is incompatible with the the associated encoder.



## Wire Patches

1. Bridge E5V to 5V: (CN11:6 Nucleo) <=> (CN10:1 Sonar)
2. Bridge NW:3V to Center:3V: (Via near U19:5 and R59) <=> (Via near C23 and U20:7/8)
3. Bridge NW:P5V to SW:P5V: (P5V: NW Back Edge trace <=> P5V: SW Back Edge Trace)
4. Bridge NE:P5V to SE:P5V: (P5V: NE Back Edge trace <=> P5V: SE Back Edge Trace)
5. Bridge NW:9V to SW:9V: (P5V: NW Back Edge trace <=> P5V: SW Back Edge Trace)
6. Bridge NE:9V to SE:9V: (P5V: NE Back Edge trace <=> P5V: SE Back Edge Trace)
7. Bridge NW:LED_GND to GND: (LED_GND:Via Near SW motor well inside corner <=> GND:Via near D11:1)
8. Bridge NE:LED_GND to GND: (LED_GND:Via Near R40 <=> GND:Via radially in from R40 one.)
9. Bridge NW:LED_GND to GND: (LED_GND:Via Near SW motor well inside corner <=> GND:Via near D11:1)
10. Bridge NE:LED_GND to GND: (LED_GND:Via Near R40 <=> GND:Via radially in from R40 one.)

## Spacers and Standoffs

The various levels are held together using spacers and standoffs.
#2-56 threaded spacers and standoffs are used wish some occassional washers inserted
to do find adjustment of the height.
A spacer is threaded all of the way through.
A stand-off has a threaded hole on one side and screw thread on the other.

Shorter spacers and standoffs can be screwed together to make longer spacers and standoffs.
A longer standoff is made up entirely out of shorter standoffs.
A longer spacer is made up one spacer and one or more standoffs.
The goal is to figure out which spacers/standoffs to use.

### Robot Levels

There are 6 robot levelst are listed below from lowest to highest:

* Base Level:
  The base level is the lowest level of the Romi Base.

* Battery Level:
  The battery level is the top of the battery compartment of the Romi Base.

* SBC Level:
  The SPC level is the bottom surface of the SBC (Single Board Computer)

* Master Board:
  The master board has two surfaces that correspond to the top and bottom surfaces
  of the master board.

* Nucleo Board:
  The Nucleo board is attached to the top of the Master board.

* Arm Base:
  The Arm shelf also has two surfaces that correspond to the top/bottom of Arm base.

* Lidar/Battery Base:
  The LIDAR battery base two surfaces.  The Lidar and Battery are mounted on the top surface.

## Inter Level Requirements:

* Base to SBC Level Spacer (18.2mm - 2.5mm = 15.7): 
  2 spacers are needed.

* Battery to SBC Level Spacer (6.35mm = 1/4in):
  2 spacers are needed.

* Base to Master Bottom Spacer (28.40mm)
  4 spacers are needed.

* Master Top to Nucleo Bottom Spacer (13mm)
  4 spacers are needed
  1 standoff is needed (for alignment)

* Master Top to Arm Bottom Standoff * Spacer (33.3mm)
  There are two standoffs needed and 2 spacers needed per robot.

* Arm Top to Lidar/Battery bottom standoff: (3in = 3 * 25.4mm = 76.2mm)
  2 stand-offs are needed.

* Master Top to Lidar/Battery bottom standoff (33.3mm + 2.5mm + 76.2mm = 108.4mm)
  2 stand-offs are needed.

## Misc:

1. Jumper positions:
   JP1 1x3 SW Power indicator LED's Enable Near 3.3V reguator
   JP2 1x2 SW 5V Shunt Near power connectors
   JP3 1x2 NE 9V Shunt Near Camera Slot
   JP4 1x2 SW 3.3V Shunt Near 3.3V regulators and arm slot
   JP5 1x2 SW P5V Shunt above 5V Shunt JP2
   JP6 1x3 SW Rasp Pi Display control Near voltage test points
   JP7 1x2 NW I2C program Near Pi Processor Heat sink slot
   JP8 1x3 SW LED's enable Near D20 LED
   JP9 1x2 NW ESTOP Near button.
2. Test Point positions:
   TP1 USB5V SW Near RasPi
   TP2 GND NE C3
   TP3 GND NE RPi Camera slot
   TP4 GND NE Back corner
   TP5 GND NW Back corner
   TP6 GND SE Near Q3 Q6
   TP7 3.3V SW Power TP's
   TP8 GND SW Power TP's
   TP9 P5V SW Power TP's
   TP10 5V SW Power TP's
   TP11 9V SW Power TP's
   TP12 SBC_TX SW Near Raspi
   TP13 SBC_RX SW Near Raspi
   TP14 WOW_TX NW Near Inside edge
   TP15 WOW_RX NW Near Wheel Well
   TP16 ~NESTOP_SET~ NE Near inside edge
   TP17 ~NWOW_ESTOP~ NW Near Wheel Well


## Board Work

### Board A

* RTC Battery
* Nulceo Alignment spacer

### Board B

* Fix diode D14
* Pololu Boost Supply
* Wow headers
* I2C EEProm header
* Electrolytic capacitors
* Center Sonar
* Bottom Sonars
* 8 Edge detector sonars
* RTC Battery
* RasPi Display Power Header
* Missing Encoder F1x3

### Board C

* Pololu Boost Supply
* Wow headers
* I2C EEProm header
* Electrolytic capacitors
* Center Sonar
* Bottom Sonars
* 8 Edge detector sonars
* RTC Battery
* RasPi Display Power Header

### Board D

* Pololu Boost Supply
* Wow headers
* I2C EEProm header
* Electrolytic capacitors
* Center Sonar
* Bottom Sonars
* 8 Edge detector sonars
  * Solder suck 2 holes
* RTC Battery
* RasPi Display Power Header

### Board E

By accident, nucleo is soldered to female connector.
No further work on E until (if) this if resolved.