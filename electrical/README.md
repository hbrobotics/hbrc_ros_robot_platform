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

# HBRC ROS Robot Electical Issues

{Wayne: This text is a bit out of date...}

The organization of this directory is organized around PCB's (Printed Circuit Boards) and
orders (PCB/Part orders.)  The KiCad printed circuit board software is exclusively used.
Each PCB is given PCB specific sub-directory (e.g. `master_board` or `encoder`).  Furthermore,
each PCB has a separate revision directory of the form `rev_` followed by a lower case letter
(e.g. `rev_a`, `rev_b`, etc.)  All of the KiCad files and sub-directories are installed
under these revision sub-directories.  In each, revision sub-directory, there are three
special sub-directories called `pretty`, `sweet` and `kiparts`:

* `pretty`: A sub-directory containing KiCad component footprints.

* `sweet`: A sub-directory contains KiCad schematic symbols.

* `kiparts`: A sub-directory of `.csv` (Comma Separated Values) that are used
  to construct some of the parts in the `sweet` directory.

The `orders` directory is organized PCB and part orders.  Each order has a
separate sub-directory called `orderN`, where N is an integer starting from 1
(e.g. `order1`, `order2`, etc.)  Each `orderN` directory contains a `pretty`, `sweet`,
and `kiparts` sub-directory that contains, KiCad footprints, KiCad Schematic symbols,
and `kipart` `.csv` files respectively.

Here is the punch line.  Each PCB board revision `pretty`, `sweet` and `kiparts`
sub-directory is actually a symbolic link to an order directory.  What this ensures
is that ***ALL*** KiCad footprints/schematic and kiparts are the same and shared
across a single order.  Once an order is made, all of the KiCAD files and order
files are locked down ***HARD*** and never edited again.  This way we can safely
come back and look at older revisions.  This also means that any footprints and
schematic symbols from the KiCad libraries are copied over into the order directories.

In general, you should just use what you want when designing a board.  The order
person will get everything locked down before shipping the board orders and PCB
orders out.

Here is a crufty ASCII art directory tree:

     electrical/
     +- master_board/
     |  +- rev_a/
     |  |  +- pretty => ../../orders/order1/pretty
     |  |  +- sweet -> ../../orders/order1/sweet
     |  |  +- kiparts -> ../../orders/order1/kiparts
     |  +- rev_b/
     |     +- pretty => ../../orders/order2/pretty
     |     +- sweet -> ../../orders/order2/sweet
     |     +- kiparts -> ../../orders/order2/kiparts
     +- encoder/
     |  +- rev_a/
     |     +- pretty => ../../orders/order2/pretty
     |     +- sweet -> ../../orders/order2/sweet
     |     +- kiparts -> ../../orders/order2/kiparts	
     +- orders/
           +- order1/
           |  +- pretty/
           |  +- sweet/
           |  +- kiparts/
           +- order2/
              +- pretty/
              +- sweet/
              +- kiparts/

This is a program called [`kipart`](https://kipart.readthedocs.io/en/latest/)
that takes a `.csv` file and generates a KiCad schematic part file.  Since
many schematics are full of square modules, program generates the square
schematic symbols with the right pin names, numbers, and signal types.  It
is generally easier to use `kipart` to manage KiCad schematic symbols than
to use the KiCad schematic symbol library.  There is a `makefile` that runs
`kipart` over all of the `.csv` files in the `kiparts` sub-directory.

Finally, there is the microcontroller.  The microcontroller is from the
STM Electronics family of microcontrollers.  There is a tool called `STM32CubeMX`
that is used to configure the pin bindings of the microcontroller.  The main
content of the file has a suffix of `.ioc`.  The `STM32CubeMX` program reads
and manipulates the `.ioc` file.  In addition, you can generate a `.csv` file
that has a summary of the pin bindings from the `.ioc` file.  The `kicube32`
program reads this `.csv` file and generates another `.csv` file that contains
the schematic symbols for the processor.

Thus the flow is:

     .ioc => STMCubeMX => .csv => kicbube32 -> .csv -> kipart -> .lib -> KiCad

Whew.  This is all automated using the `make` program and appropriate `Makefile`'s.

The design rules for schematic capture are:

* All text is horizontal (no exceptions) and the same size.
* All net names are upper case letters and digits.  A lower case `n` can be used
  as a prefix indicate an active low logic line.
* No rectangular symbols have wires coming out vertically connections are horizontal.
* All wire are on a rectangular grid.
* When possible try to organize the schematic such that inputs are on the left
  and outputs are on the right.
* Hierarchical design is used to organize the sheets.
* Try to keep "air wires" to a minimum.
* Make all power and ground connections explicit.
* Resistors use the older "3 hump" style.

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

Timer Notes:

There are at total of 14 timer modules in STM32F767:
* 2 Advanced 16-bit timers (TIM1/TIM8): 4-inputs and 6-outputs. (Has PWM shoot through protection!)
* 4 Medium 16/32-bit timers (TIM2/TIM3/TIM4/TIM5): 4-inputs and 4-outputs:
  * TIM3/TIM4: 16-bit timers
  * TIM2/TIM8: 32-bit timers
* 2 Basic 16-bit timers: (TIM6/TIM7): 0-inputs and 0-outputs.  Internal timers only.
* 6 General Purpose 16-bit Counters (TIM9/TIM10/TIM11/TIM12/TIM13/TIM14):
  * TIM9/TIM12: 2-inputs and 2-outputs
  * TIM10/TIM11/TIM13/TIM14: 1-input and 1-output
Thus, the total number of counters is 14.  Note that typically, the input and the output
pin are one and the same, so it can only be used as either an input or an output but not both.
Also, only the first 6 counters listed above support encoder mode.
By the way, this is a lot of counters to pay around with!

The devices that need timing support are:
* 1 Lidar:
  Some of the less expensive Lidars out there need a PWM signal for the motor control.
* 16 LED's:
  There is GPIO pin dedictated to each LED.
  It would be nice to be able to PWM the LED's.
  After some thought, the conclusion is to put all 16 LED's on one 16-bit port
  and use DMA triggered off of a timer to transfer a 16-bit wide LED "waveform" to the the port.
  The LED "waveform" send bit0 to LED1, bit1 to LED2, ..., bit15 to LED16.
  This is very similar to the writing a wave form out to a DAC to play a
  (sound)[https://vivonomicon.com/2019/07/05/bare-metal-stm32-programming-part-9-dma-megamix/]
  The DMA is put into circular mode and the timer can be adjusted to tweak the waveform "frequency".
  If this does not work, PWM is probably out.
* 7 Sonar's:
  There is one trigger and one echo line per sonar.
  Again the concept is to detect the echos using the External Interrupt functionality.
  It is a little strange because, there are 16 pin change interrupts and they can be
  mappped to pretty my any GPIO pin.  It is only possible to select one pin N form PA, ..., PJ
  for external interrupt.  Thus, PA0, PB1, ..., PJ15, would work or PA0, PA1, ..., PA15,
  of some mixture of PA0, PB1, PA2, PC3, ...., PB15.  The SYSCFG registers are used to set
  the pin selections up in addition to the Extended Interrupts (EXTI).  There needs to be one
  a free running timer to time length of the echo pulses.
* 4 Servos:
  There a 3 servos for the arm and 1 extra server.
  Accurate pulse widths between 1ms and 2s are very desirble to prevent servo chatter.
  Servos only need to be updated approximately every 20ms with a pulse that is between 1ms and 2ms.
  Making the pulse width very accurate is a requirement,
  but the inter pulse time is not that critical.
* 2 Encoders:
  There are 2 encoders and each encoder requres two inputs signals.
  The encoder mode for the STM requires 2 timer inputs per encoder.
* 2 Motors:
  There are two drive motors.
  Each motor driver has two inputs where one input is active PWM and the other side
  is either high or low.
  There is no need to PWM both inputs at the same time.
  Another way to to think of it is that one side will fractional PWM
  and the other side will be either 100% or 0% PWM.
  This will chew up 4 timer outputs for both motors.

The summary is:
* LED's:    0 inputs, 0 outputs, 1 timer needed trigger DMA to write to LED's.
* Sonars:   0 inputs, 0 outputs, 1 EXTI interrupt + 1 free running timer.
* Servos:   0 inputs, 4 outputs, 0 interrupts (PWM) + 1 32-bit timer module with 4 PWM's enabled.
* Encoders: 4 inputs, 0 outputs, 2 timer modules (1 per encoder) + 1 timer (sysclock?) for PID loop.
* Motors:   0 inputs, 4 outputs, 0 interrupts (PWM) + 1 timer module with 4 PWM's enabled.

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

-->