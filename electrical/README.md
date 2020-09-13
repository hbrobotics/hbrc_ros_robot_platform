<!--
MIT License

Copyright 2020 Home Brew Robotics Club

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
-->
<!-- <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 100 characters >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> -->

# HR2 Electrical Directory

The overall architecture of the `electrical/` directory is:

* `README.md`:
  A this document written in markdown format.
* `docs/`:
  A sub-directory containing documentation.
* `orders/`:
  A sub-directory contain per order information.
  This includes schematic symbols, PCB footprints, and parts ordering information.
* `master_board/`:
  The design files for the main HR2 master PCB.
  The master board also has the `STM32CubeIDE` project file for pin assignment
  and associated firmware generation to configure the microcontroller.
* `st_adapter/`:
  The design files for the ST-Link adapter PCB.
* `encoder/`:
  The design files for shaft encoder PCB.
Over time, additional PCB's a will be added.

Each PCB directory is broken into revision sub-directories:

  * `rev_a/`:
    This is the first PCB revision.
  * `rev_b/`
    This is the second PCB revision
    (only started after the first version has been built.)
  * etc.

The `orders` sub-directory is broken into a sequence of sub-directories labeled
`order1/`, `order2/`, `order3/`, ...
The `order`*N* sub-directory attempts to capture all of the schematics symbols,
PCB footprints, and electrical/mechanical parts that are all ordered together
(e.g. PCB Gerbers go to PCB fabrication and parts lists go to parts vendors.)
The requirement is that all PCB's share a consistent set of PCB footprints,
schematic symbols, and electrical parts.

After the orders go out.  All of the existing PCB's and contents of the
associated files are locked down into a `git` repository.  This is so that
subsequent revisions do not the existing designs.  For the next revisions,
a new `order`*N+1* sub-directory is created.  Many of the symbols from the
previous `order`*N* sub-directory into the `order`*N+1* sub-directory,
where they can be freely modified to without breaking any of the older revisions.

The contents of an `order`*N*`/` directory are:

* `Makefile`:
  This is used by the `make` program to build everything.
* `pretty/` sub-directory:
  This is a sub-directory where all of the shared KiCad PCB footprints live.
* `kiparts/` sub_directory:
  This is a sub-directory of `.csv` files,
  that specify the various rectangular schematic symbols definitions are stored.
  There is a program called [`kipart`](https://kipart.readthedocs.io/en/latest/)
  that takes a `.csv` file and generates a bot the `.lib` and `.dcm` file.
  Since many schematics are full of square modules, program generates the square
  schematic symbols with the right pin names, numbers, and signal types,
  it is generally easier to use `kipart` to manage KiCad schematic symbols than
  to use the KiCad schematic symbol library.
* `order`*N*`.lib`:
  This is the library of generated schematic symbols.
* `order`*N*`.dcm`:
  This is the schematics symbol documentation file.
  
The design rules for schematic capture are:

* Almost all text is horizontal.
  The only  exceptions only allowed for "stock" schematic symbola) that come with KiCad.
* Most rectangular symbols have wires coming out vertically connections are horizontal.
  There can be an occasional exception.
* All net names are upper case letters and digits.  An lower case `n` can be used
  as a prefix indicate an active low logic line.
* All wires are on a rectangular grid.
* When possible try to organize the schematic such that inputs are on the left
  and outputs are on the right.
* Hierarchical design is used to organize the sheets.
* Try to keep "air wires" to a minimum.
  Instead use "blue" buses to help reduce the wire mazes.
* Make all power and ground connections explicit.
* Footprint names are metric (e.g. 1608 instead of 0603.)
* Footprint names are of the form *part_name*`;`*footprint_name*.
* Resistors use the older American "3 hump" style of resistor.
* Resistance values use the ISO unit symbol of Ω.
* Capacitance values use the units of µF, nF, and pF.

<!--

[Raspian Shutdown Button](https://www.quartoknows.com/page/raspberry-pi-shutdown-button#:~:text=Press%20the%20shutdown%20button.,stopped%2C%20aborting%20the%20shutdown%20procedure.)

USB2 Micro connector Pinouts:

USB_Micro-B_Amphenol_10103594-0001LF_Horizontal

  USB_MICRO_B;S+T:FCI_10118194_0001LF;CONN USB MICRO B RECPT SMT R/A 1:[bus_loki: N23]
    Newark element14:67T2260 [Amphenol: 10118194-0001LF] 1/$0.403 10/$0.388 

1: VBUS +
2: D-
3: D+
4: ID
5: GND

H-Bridges

MTS62C19A: Pretty complicated, current sense resistor.  $1.09
TC78H651xxx: 7V max $1.25
LV8549MC: 1 Amp total output. $1.29

TI: DRV8870DDAR dumb dual H-bridge, NFET's; no thermal; $1.92

Grove:

All:
  pin3=VCC
  pin4=GND
Digital:
  pin1=Dn (bi_directional)
  pin2=Dn+2 (bi_directional)
Analog:
  pin1=An (Analog)
  pin2=Dn+2 (Analog)
UART:
  pin1=RX
  pin2=TX
I2C:
  pin1=SCL
  pin2=SDA

Part Number	Pin 1	Pin 2	Size	Part		Title
101020000	RX	TX	N/S			Serial	Grove - Serial Camera Kit  Serial
101020001	A0	NC	1x1	WSP2110		HCHO Sensor
101020002	A0	NC	1x2			GROVE GAS SENSOR(O2)
101020003	D0	NC	1x1			GROVE BUTTON (pull down resistor)
101020004	D0	NC	1x1			GROVE SWITCH
101020005	D0	NC	1x1	MVS0608.02	Collision
101020006	A0	NC	1x2	MQ-3		Gas Sensor (alchool)
101020008	A0	NC	1x3~			Moisture Sensor
101020009						Line Finder (obsolete)
101020010	A0	NC	1~x2~			Ultrasonic Ranger
101020011	A0	NC	1x2	DHT11		Temp/Humidity Sensor
101020012	A0	NC	--			Dust Sensor
101020013	SCL	SDA	--	DS1307		Real Time Clock
101020014				GL5528, LM358	Grove Light Sensor (Digi-Key discontinued)
101020015	A0	NC	1x1	LM358, NCP18WF104	Temperature Sensor
101020016	NC	SIG	1x1	TSOP382		Infrared Receiver
101020017	A0	NC	1x1			Rotory Angle Sensor (potentiometer)
101020018	A0	NC	1x1			Water Sensor
101020019	A0	NC	1x2	AM2302		Humidity Sensor
101020020	D0	NC	1x2			PIR Motion Sensor
101020021						Air Quality Sensor  (obsolete)
101020022						Light Sensor  (obsolete)
101020023	A0	NC	1x1	LM386		Sound Sensor
-
101020025	D0	NC	1x1	LM386		Tilt Switch




101020017	A0	NC	1x1			Rotary Angle Sensor (10K Pot)
101020025	Dn	NC	1x1			Tilt Switch (digital)
101020028	A0	A1	1x2			Thumb Joystick (2 20K Pot's)



There are plenty of timers left.

Lidar Notes:

Lidars are kind of a mess.  All of them have different interfaces.  The right strategy
is a daughter board strategy.  The maximum number of pins seems to be 7:
  * VCC (5V)
  * GND (0V)
  * TX (3.3V)
  * RX (3.3V)
  * MOTOR_PWM (3.3v)
  * MOTOR_EN (3.3v)
  * DEV_EN (3.3V)
This board can be quite small.

* YDLIDAR X2:
  * Cost: $69US
  * Range: 8m
  * Voltage 4.8V - 5V - 5.2
  * Start current: 300mA - 400mA - 500 mA
  * Working current: 200mA - 350mA - 380 mA
  * RPM: 5Hz - 8Hz
  * Pins:
    * M_CTR (PWM or volatage) 0V - 1.8V 3.3V. 0% duty cylce => full speed .
    * GND
    * Tx  (115200 8N1) 1.8V -3.3V - 3.5V
    * VCC

* YDLIDAR X4:
  * Cost: $99US
  * Range 10m
  * Scan Rate: 6-12Hz
  * Volatage: 4.8V - 5V - 5.2V
  * Start Current: 400mA - 450mA - 480mA
  * Working Current: 330mA - 350mA - 380mA
  * Pins:  PH1.25-8P
    * VCC 5V
    * Tx 3.3V 128000 8N1
    * Rx 3.3V
    * GND
    * M_EN (Motor Enable) 3.3V
    * DEV_EN (Device Enable) 3.3V
    * M_SCTP (Motor Speed Control) Voltage or PWM 0-3.3
    * NC
  
* RPLidar A1M8:
  * Cost: $99US
  * 115200 8N1
  * Voltage: 5V(typical) - 9V
  * MOTOCTL: 0V - VMOTO
  * VCC_5: 5V
  * TX: 0-5V
  * RX 0-5V
  * Start Current 500mA - 600mA
  * Working current: 300mA - 350mA
  * Pins
    * Connector 1:  PH1.25-4P
      * TX 115200 8N1 3.3V
      * RX
      * VCC
      * GND
    * Connector2:  PH1.25-3P
      * VMOTOR 5V (9V max)
      * MOTOCTL
      * GND

* YDLIDAR G2:
  * Cost: $159US
  * Range: 12m
  * Scan Rate: 5-12Hz
  * Voltage 4.8-5.0-5.2V
  * Start Current: 550mA-600ma-650mA
  * Sleep: <50mA
  * Working Current: 250mA-300mA-350mA
  * Pins: PH1.125-5P
    * NC
    * GND
    * Rx (230400 8N1) 3.3V
    * Tx
    * VCC

* YDLIDAR G4:
  * Cost $304.20US
  * Range: 16m
  * Scan Rate: 5-7-10Hz
  * Voltage: 4.8-5-5.2V
  * Start Current: 700-800-850mA
  * Standby Current: < 50mA
  * Working Current: 350-400-450mA
  * Pins same as G2

SPI Devices:

The STM32F676 has 6 SPI interfaces and support NSS and TI mode.
* SPI1, SPI4, SPI5, and SPI6 can operate at 54MBits/sec.
* SPI2 and SPI3 can operate at 25Mbits/sec.
The STM32F676 has 3 S2C interfaces:
* SPI1, SPI2, SPI3.


-->
