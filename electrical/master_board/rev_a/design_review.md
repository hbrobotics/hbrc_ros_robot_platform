# HR2 Design Review

This document goes over the various sections of the HR2 (Rev. A) schematic
to get some feedback before shipping off to be manufactured.

The HR2 schematic is broken into the following pages:

* Overview (Block Diagram)
* Power Supply
* Single Board Computer
* WOWBus (and EStop)
* Motors and Encoders
* ST-Link
* LED's
* Servos and Sonars
* Connectors

The sections in this document roughly follow the sequence above.
There is a Conclusions section at the end.

The sheet order in the `master.pdf` file (i.e. the HR2 schematic) may exactly not match
the order above.

## Overview

The HR2 uses KiCAD hiearchical design.

The center of the overview is the Nucleo144 daughter board.
Pretty much all of the signals flow into to the Nucleo144.

The power supply bus snakes around the entire overview show which sub-sheet uses which power rails.

The signals from each sub-sheet use KiCad bus notation (blue wires) to show groupings of signals.

There are some SPI shift registers shared between the bottom right two sub-sheets.
The SPI signals and a serial shift line show up between the two.

Lastly, the various mounting holes are explicitly listed in the schematic at the bottom.

## Power Supply

The plan for powering the HR2 is to use an USB Power Bank module.
These modules have all of the charge and discharge circuitry for
the lithium batteries built in.
The intention is that the HR2 owner buys a USB Power Bank that meets their needs.

There are multiple versions of power banks out there.
The HR2 uses the older USB-B micro connector based power banks.
The USB-C standard support greater power transmission, but is more complicated to use.
The USB-B connector is a simple electrical connection and was selected for that reason.

The USB Micro Connector used for Power input is the GTC USB3140-30-0230-1-C
vertical insertion USB micro-B  connector (Digi-Key: 2073-USB3140-30-0230-1-CCT-ND).
Vertical insertion was selected to allow the cable to be plugged in from the top.
Side insertion was rejected because the cables would stick out the side.

The selected connector specification sheet has the following electrical specifications:

* Voltage Rating: 30 VAC
* Current Rating: 1.8A
* Contact Rating: 30mΩ

There are 2 USB-B micro connectors on the HR2.
At 1.8A max current and 5V input that yields 2 * 1.8A * 5V = 18W maximum power.
At first 18W sounds like a lot of power, but there are some real power hungry items on the HR2.

In addition, to the 2 USB-B micro connectors,
there is a jumper that can be wired to a bench-top power supply for initial power up.
This allows setting current limits for shorts, etc.

There are 4 power rails for the HR2:

* 5V:
  The input power rail from the USB connectors.
  This rail powers the following:

  * SBC: Single Board Computer (i.e. Raspberry Pi)
  * Nucleo-144: This board further regulates the 5V down to 3.3V.
  * WOW-Bus: The 5V is needed for the "CANBus" transceivers.
  * Sonars: The sonars and associated 3.3V to 5V level shifters need 5V.
  * Connectors: Eventually (not HR2 Rev. A) some of the expansion connectors will need 5V.

* P5V:
  A "dirty" (pulsed) 5V power rail.
  This "dirty" rail is connected to a 1000µF 10V electrolytic capacitor through an ideal diode.

  The selected ideal diode is the LM66100 and it has the following specifications:
  * On-Resistance: 79mΩ @ 5V
  * Maximum continuous current: 1.5A

  The dirty rail is used to power the following:
  * Rim LEDS: The outer rim LEDs around the are pulse width modulated.
  * Servos: The servos have motors that can have current surges.

  The electrolytic capacitor is in a 10mm diameter x 13mm high radial can.
  In order to avoid touching any of the components on the underside of the Nucleo-144,
  the capacitor height is limited to 13mm.

* 9V:
  A 9V power rail is generated using a 5V to 9V boost converter.
  The boost converter is a Pololu U3V70F9 (#2894) which has the following specifications:
  * Max Input Current: 10A
  * Efficiency: ~90% @ 5V

  At 90% efficiency and 5V input the maximum output wattage is .9 * 10A * 5V = 45A
  which far exceeds the available input power.

  The 9V power rail is used as follows:
  * WOW-bus: The 9V rail is daisy chained down the WOW-bus.
  * Motors: The motors are driven from 9V.

  The 9V power rail is expected to be somewhat dirty due to the drive motors.
  It has 2 470µF 25V 10mm diameter 13mm high electrolytic capacitors
  to provide 940µF of capacitance to support drive motor surges.

* 3.3V:
  A 3.3V power rail is used to power the 3.3V is used to provide power to various IC's.
  The 3.3V regulator is in a SOT-323 regulator with a heat sink tab.

* Ground:
  Technically, ground is not a power rail, but is deserves some discussion:
  * Both PCB layers are ground filled.
  * Care is taken in the schematic to ensure that each ground pin is explicitly connected to ground.
  * The Ground planes are "stitched" to ensure that copper fills all of the nooks and crannies and
    that there is excellent via connectivity between the layers.
  * There is a nasty ground path problem due to component placement geometry issues
    and the thermal cutouts for the Raspberry Pi 4 heat sinks.
    This problem is solved by soldering a thick beefy ground wire to bridge the ground gap.
    This is in the northeast quadrant of the PCB, near the Raspberry Pi camera slot.

Other comments:

* All power traces are .5mm wide.

* Each power rail has both a 0Ω resistor and a shunt jumper.
  For HR2 bring up, the 0Ω resistor is not installed and the shunt jumper is shorted instead.
  The shunt jumper can have a short loop of wire
  so that a Hall Effect current probe can be used to measure current on an oscilloscope.

* The 4 rails hug the outer rim of the PCB and are tapped into as needed.

  The rails are arranged as follows:
  * Top Outer: P5V
  * Top Inner: 3.3V
  * Bottom Outer: 9V
  * Bottom Inner: 5V

* There is a bank of 4 LED's that show that there is power on the 4 power rails.
  There is a jumper to enable/disable these LED's

* Near each power LED, there is a power test point.

* The soft latching power switch circuit comes from
  [World's Simplest Soft Latching Power Switch](https://www.youtube.com/watch?v=Foc9R0dC2iI)
  from David Jones #262 Electrical Engineering Vidoe Blog.

* There are a number of places in the circuit where a PFET is used to switch power:
  * Power Supply: Soft Latch circuit
  * Single Board Computer: Raspberry Pi Display
  * WOWBus: Bus Daisy Chain Power
  
* The nominal PFET used for switching power is the Diodes Incorporated DMP1045U.
  The DMP1045U has the following specifications:
  * FET Type: P-Channel
  * Technology: MOSFET (Metal Oxide)
  * Current - Continuous Drain (Id) @ 25°C: 4A (Ta)
  * Drive Voltage (Max Rds On, Min Rds On): 1.8V, 4.5V
  * Rds On (Max) @ Id, Vgs: 31mΩ @ 4A, 4.5V
  * Vgs(th) (Max) @ Id: 1V @ 250µA
  * Vgs (Max): ±8V
  * Power Dissipation (Max): 800mW (Ta)
  * Operating Temperature: -55°C ~ 150°C (TJ)
  * Supplier Device Package: SOT-23
  * Package / Case: TO-236-3, SC-59, SOT-23-3
  * Drain to Source Voltage (Vdss): 12V
  * Gate Charge (Qg) (Max) @ Vgs: 15.8nC @ 4.5V
  * Input Capacitance (Ciss) (Max) @ Vds: 1357pF @ 10V
  Alternative selections for the P-Channel PFET are welcome.

## Single Board Computer

The nominal Single Board Computer is a Raspberry Pi 4.
Any board that conforms to the 40 pin header and mounting holes can be substituted in.

Comments:

* Power:
  The Raspberry Pi Computer has a current range of 575mA - 885mA.
  [Power Measurements](https://raspi.tv/2019/how-much-power-does-the-pi4b-use-power-measurements)
  This is a lot of power.

* Real Time Clock (RTC):
  * The RTC is an MCP7940.
  * The RTC is needed to run ROS on the SBC.
  * The SBC can access the RTC but the Nucleo can not.
  * Access is performed using the SBC I2C bus.
  * The battery holder a TE Connectivity Amp 1376164-1 (Digi-Key A108891CT-ND)
    for a 6.8mm diameter coin cell.
  * The crystal is 32.768kHz crystal.

* EEPROM:
  * There is a 32K I2C EEPROM used to be compatible with the Raspberry Pi Hat protocol.
  * There is a jumper that can be used to program the EEPROM from the SBC.

* Signals:
  The signals are:
  * `SBC_TX`/`SBC_RX`: Used to talk to the Nucleo via a serial protocol.
    The nominal baud-rate is 115200 bits per second in an 8N1 framing protocol.
  * `SBC_SDA`/`SBC_SCL`:
    An I2C bus used to talk to the RTC.
  * `RPI_DSP_EN`:
    A signal to enable/disable power to the Raspberry Pi display.
  * `SBC_STOP`:
    A button that the user can push to request that the SBC shut itself down.
  * `SBC_ALIVE`:
    A signal from the SBC to the Nucleo to indicate that the operating system is up.
    There is an LED that when this signal is active.

* Raspberry Pi Display:
  The Raspberry Pi Display has the following features:
  * Jumper:
    There is a jumper that specify if the display is to be enabled on power on.
  * `RPI_DSP_EN`:
    If this signal is asserted by the SBC, the jumper is ignored.
    This means that the display could be configured to power up on boot and
    then get powered off later on.
    Alternatively, the display can stay off during boot up and be explicitly
    powered on after the software is ready to use it.
  * The Raspberry Pi Display consumes 660mA.
  * There is a possibility of providing a third micro USB-B connector specifically
    power the Raspberry Pi display.
  * There is a regular FET that level shifts from 3.3V to enable the PFET.
    Please make sure that this circuit looks correct.

## WOWBus and EStop

The WOWbus is an expansion bus an EStop is for emergency stop.
Both of these features are merged onto one schematic page.

### Emergency Stop

A small robot like the HR2 does not technically need an Emergency Stop.
since it is extremely unlikely to hurt anybody, their pets or their property.
Given the pedagogical nature of the HR2,
exposing robot enthusiasts to think about Emergency Stop is a good idea
because encourages people to think about how to safely stop and restart robot.
This is huge deal out in industrial robotics, so it worth exposing people early.

The usual goal for an Emergency Stop system is to stop all robot motion.
For the robot base, this can be done by disabling power to the drive motors.
For an arm, the usual goal is to stop its movement.
Disabling power to the arm could cause it to damage things by releasing its grip,
moving due to gravity, etc.

Emergency Stop can be triggered from the following sources:

* `NRST`:
  The reset signal is asserted at power on.
  Thus, the robot starts off in an ESTOP condition that needs to be cleared first.
* `MOTOR_FAULT`:
  If the motor driver chip is unhappy it asserts the motor fault signal.
* On Board Push Button:
  There is a button on the HR2 that can be pushed to trigger an ESTOP.
* Off board Push Button:
  There is a jumper that can be wired to an external ESTOP button.
* WOWBus:
  The WOWBus has a dedicated ESTOP channel.

There are two additional signals wondering around:

* `ESTOP`:
  When assert the system is in emergency stop.
* `ESTOP_CLR`:
  When assert it clears the emergency stop condition.

Ideally, there would be a way to identify what triggered the 
but there is currently inadequate HR2 PCB space to run the ESTOP trigger signals to input pins.

The Emergency Stop is implemented using a '74 D-type flip-flop.
The Emergency Stop is either triggered from the 3-input AND gate (button, fault, or reset) or
from a falling edge on the Emergency Stop signal on the WOWBus.
(Oops, I just noticed that I need to invert this signal,
since it the WOWBus idles high and only goes low when the a WOWBus device triggers ESTOP.)

The `ESTOP` signal is forwarded to the Nucleo so it can immediately shut things down.
It can forward the `ESTOP` condition to the SBC.
When the SBC is happy, it can send a message to the Nucleo to clear the Emergency Stop condition
via the `ESTOP_CLR` wire.

### WOWBus

The WOWBus is basically Wayne's expansion bus.
Currently, there are no boards that use it, so it is kind of unnecessary.
However, Wayne wants to experiment with this bus so that is why it is there.
The ultimate goal is to push support Arudino shields, MikroBus modules, and Grove Modules
to the WOWBus.

The following was copied from the schematic:

1. WOWBus is an an expansion bus.
2. WOWBus stands for Wayne's Omnipotent Wonderful Bus (heavy sarcasm.)
3. WOWBus uses 2x6 ribbon daisy chain cables for interconnect.
4. WOWBus has a nominal 9V rail, but this can droop due to resistive losses down the daisy chain.
5. The WOWBus connector is designed to mirror image power, ground, and signaling.
6. If a WOWBus connector is accidently reversed on a daisy cable, nothing shorts out.
7. However, an accidentally reversed cable, will basically make the bus non-functional
   until corrected.
8. The WOWBus has two "CAN bus" (ISO-11898-2) physical differential pairs for signaling.
9. The first differential pair is used as a multi-drop bidirectional data transfer.
10. The second differential pair is used to signal Emergency Stop.
11. When the WOWBus is enabled, it powers up both the transceivers and the differential
    pair termination resistors.
12. The expansions boards are responsible for providing the bus terminator resistors for
    for the other end of the differential pairs.
13. The Bus powers up in the EStop conndition and must be explicitly cleared out.
14. When ESTOP occurs, it latches up and must be explicitly cleared.

## Motors and Encoders

The motors and encoders are on one schematic page since they are kind or related to one another.

### Motor/Encoder PCB's

There is a Motor/Encoder PCB that is soldered to each drive motor.
This Encoder PCB  is connected to HR2 via a 6-pin connector.
The connector is split into two 1x3 pin connectors, but that is not shown in the schematic.
It is best to think of it is one 1x6 pin connector.
The pins of this connector are:

1. `MOTORIN+`:
   The plus side of the motor.
2. `MOTORIN-`:
   The minus side of the motor.
3. `VCC`:
   3.3 Volt to power the Hall effect quadrature encode sensors.
4. `GND`:
   The ground return from the Hall effect quadrature encoder sensors.
5. `QUADA`:
   The 3.3V logic level of the A channel of the quadrature encoder.
6. `QUADB`:
   The 3.3V logic level of the B channel of the quadrature encoder.

The `MOTOR+`/`MOTOR-` wires are .5mm wide and are run to the motor driver chip.
One of the motor power pairs is relatively short and the other is quite a bit longer.

The quadrature encoders are feed into input pins on the Nucleo that are setup to handle
quadrature encoder signals.

### Motors and Motor Driver

The motor has the following specifications:

* Full Name: 120:1 Mini Plastic Gearmotor HP, Offset 3mm D-Shaft Output, Extended Motor Shaft
* Vendor Number: Pololu #1520
* Size 36.5 x 20 x 27.4 mm
* Weight: 19 g
* Shaft diamter: 3 mm (D Shaft)
* Typical operating voltage: 4.5V
* No-load speed @ 4.5V = 150 rpm
* No-Load current @ 4.5V = 130 mA
* Stall current @ 4.5V = 1250 mA = 1.25 A
* Stall Torque @ 4.5V = 25 ozf-in = 25 / 141.612 N-m = 17.64 N-m

The stall current is quite high,
Luckily, the motor should not be in a stall condition for very long.
The 940µF of capacitance should help out with the stall condition.

The motor driver chip is the Texas Instruments DRV8833PWPR.
It can drive two motors using two H-bridges.

There 6 digital I/O pins:

* `LMOTOR_CTL1`/`LMOTOR_CTL2` (input):
  These two pins control the direction and speed of the left motor.
* `RMOTOR_CTL1`/`RMOTOR_CTL2` (input):
  These two pins control the direction and speed of the right motor.
* `MOTOR_SLEEP` (input):
  This pin puts the motor driver to sleep.
* `MOTOR_FAULT` (output):
  This pin is asserted when the motor driver is unhappy (triggers Emergency Stop too!)

The DRV8833 has a current sense resistor for each motor that is configured to enter chop mode
when the current exceeds 1.25A.

The 4 control lines are run to 32-bit counter/timer chips in the Nucleo.

## ST-Link

The Nucleo-144 comes with a detachable ST-Link PCB board.
The ST-Link is a USB module provides 3 basic functions:

* Firmware Download:
  The firmware can be download over the USB cable into the Nucleo.
* Serial Port:
  The USART3 on the Nucleo is wire to the ST-Link and it can be used for `printf` style code
  debugging over the USB cable.
* Debugger Support:
  The ST-Link also supports debugging using a couple of debugging signals.

The original goal was to leave the ST-Link module attached to the Nucleo board.
Unfortunately there is a mechanical interference with the robot wheels and it needs to be detached.
Once the ST-Link it is detached, there needs to be a way to reconnect it to the Nucleo.
This is done with a ST-Link adapter board that plugs ST-Link plugs into and
then the adapter board is plugged the HR2 PCB.

The adapter board has the following signals via a 2x4 connector.

* Power:
  `3V3` (pin 1) and `GND` (pin 3).
* Debug/Download:
  `SWCLK`(pin 2), `SWDIO` (pin 4), and `SWO` (pin 6).
  `SWO` is currently unused.
* Communications:
  `RX` (pin 7) and `TX` (pin 8).
  These 2 pins are connected to USART3 on the Nucleo.
* Reset:
  `NSRT` (pin 5).

## LED's

There are 16 LED's around the rim of the robot.
The LED's are organized into 4 quadrants:

* Left-Front: 5 LED's
* Left-Rear: 3 LED's
* Right-Rear: 3 LED's
* Right-Front: 5 LED's

This results in a total of 16 LED's.

Due to a lack of direct I/O pins on the Nucleo,
the LED's are controlled via a SPI port.
There are 2 8-bit shift registers that are organized to provide a total of 16-bits LED control.

The selected LED is a Vishay TLPG5600.
This is a green LED with through hole mounting leads.
The LED illumination beam is 90 degrees from the mounting leads and point outwards.

The part specifications are:

* Manufacturer: Vishay Semiconductor Opto Division
* Manufacturer Part Number: TLPG5600
* Digi-Key part #: TLPG5600-ND
* Forward Voltage Drop: 2.4V
* Absolute max. current: 200mA

The target maximum current is 20mA.
Using R = V/I, the computed resistance is (5V - 2.4V) / .020A = 130Ω.
Using power P = I * I * R = .020A * 0.020A * (5V - 2.4V) = 0.052W.
Thus, 120Ω rated at 0.1W should be adequate.

(The schematic currently specifies 470Ω rather than 120Ω.)

If all 16 LED's are on a full current, the total current through the LED's is approximately,
16 * 0.020A = 0.32A = 320mA.

## Servos and Sonars

The servos and sonars are on the same schematic sheet since the share the same voltage
level shift IC's.

### Servos.

There are 4 servo connectors:

* Servos 1-3:
  The first 3 servos are for standard hobby servos.
  Two of the standard servos are meant to be used with the Pololu Romi Arm.
  The remaining hobby servo is not allocated, but could be used to tilt or pan a camera.
* Servo 4:
  The fourth connector is meant for a an arm gripper servo for the Pololu Arm.
  This is probably a FEETECH FS90-FB with a Position Feedback (Pololu #3436).

According to the Pololu documentation:

* Two standard-size servos sit at the base of the arm and control the height and tilt angle
  of the micro gripper while a micro servo is housed inside the gripper to actuate the
  paddles themselves.
  The larger lift and tilt servos can briefly draw up to around 1.8 A each when commanded
  to move abruptly, but the typical current draw should be under an amp each.
* The stall current of the micro gripper servo is approximately 0.8 A at 6 V.
  All three servos are intended to be powered from 4.8 V to 6 V (nominal) battery packs or
  a suitable power supply in that general range.
  For each servo, there is an approximately 1:1 correspondence between the servo position
  in milliseconds and the feedback voltage in millivolts, so for example,
  the feedback voltage will be around 1.5 V at the position corresponding to 1.5 ms servo pulses.

Notice that maximum current for the 3 servos is 1.8A + 1.8A + .8A = 4.4A which exceeds
the total current available.
The hope is that in practice the servos will draw less than that.
Actual current measurements are needed to verify this hypothesis.

Due to the risk of overloading the power source,
there is a current sensor to measure just the current through the servos.
The current sensor selected is the ACS711ELCTR-12AB-T (Digi-Key 620-1370-1-ND .)
When run a 3.3V, the sensor will output 3.3V/2=1.65V when there is no current.
For each Ampere of current the voltage increases by .110V.
For maximum current of 4.4A, this results in a voltage of 3.3V/2 + 0.110V * 4.4 = 2.134V.
This should provide a coarse idea of how much current is flowing through the servos.

The signals associated with servos are:

* `SERVO1/2/3/4`: These are 3.3V signals that are level shifted up to 5V.
* `SERVO_CUR`: The total servo current measured.
* `SERVO_SEN`: The servo gripper position.

The `SERVO1/2/3/4` signals are connected to 32-bit resolution counter timers on the Nucleo.
This should result in extremely high resolution servo pulses.

### Sonars

There are 7 sonars on the HR2 and they are organized as follows:

* 1 Rear Left
* 2 Front Left (1 on top and 1 on bottom)
* 1 Front Center
* 2 Rear Right (1 on top and 1 on bottom)
* 1 Rear Right

The sonars are "standard" the HCSR04's that run at 5 volts.
The 7 echo pulses are level shifted by resistor voltage dividers back down to 3.3V
and feed directly into the Nucleo.
The Nucleo configures the pins as interrupt on change pins so it can record the start
and end time of each edge transition.
The time difference corresponds to the distance.

The signals to and from the sonars are:

* `TRIG1`/`2`/`3`/`4`/`5`/`6`/`7`:
  The 7 sonar trigger pulses that are level shifted up to 5V.
* `ECHO1`/`2`/`3`/`4`/`5`/`6`/`7`:
  The 7 sonar echo pulses that are level shifted from 5V back down to 3.3V.

Due to a lack of I/O pins on the Nucleo,
there a insufficient Nucleo pins available to directly trigger the sonars.
Instead a latching shift register is used to provide generate the trigger pulses.
Writing the code for this should be a total pain in the rear.

## Connectors

The connectors are a bunch of various connectors that are put in one place
because they share some logic.

The following connectors are in various stages of support:

* MikroBus Connectors:
  Currently there are 2 MikroBus connectors are present on the PCB.
  One MikroBus connector is a "short" and the other is a "medium".
  Neither MikroBus connector is wired up for Revision A.a
  If revision B is a 4-lay board, it should be possible to support these a bit better.
* Grove Connectors:
  Currently there no Grove connectors present on the PCB.
  Maybe in Revision B, some can be added.
  There are plenty of places to mount Grove boards though.
* Input Connectors:
  There are 8 input connectors for edge detectors.
* Lidar Connector:
  There multiple different LIDAR's available and each one is slightly different.
  The solution is to have a small adapter board for each different Lidar.
  This connector is present.
* ESTOP Inputs:
  It would be nice to be able to figure what cause an E-stop so can be reliably reported
  for subsequent clearing.
* Button Inputs:
  It would be nice to have a couple of user input buttons.

### MikroBus

There are hundreds of Mikrobus modules out there.
It would be nice to interface to them.

### Grove

The are hundreds of Seeedstudio Grove connectors.
It would be nice to interface to them.

### Input Connectors

The are lots of sensors that take power and ground and provide one bit of digital input.
For example, the
[IR Sensor](https://www.amazon.com/HiLetgo-Infrared-Avoidance-Reflective-Photoelectric/dp/B07W97H2WS)
This connectors use these input pin-outs to provide up 8-bits of edge and/or line sensing.

### Lidar Connector

The Lidar connector has the following pins:

* `LDR_TX`/`LDR_RX`: Serial data connected to a Nucleo USART.
* `LDR_PWM`: A PWM signal for controlling the lidar speed.
* `LDR_SPIN`/`LDR_EN`: Enable pins for some of the lidars.
* `5V`/`3.3V`/`GND`: Power for various Lidars.

Each different lidar needs a different adapter board.

### Emergency Stop Inputs:

The signals are on the board, but there is insufficient PCB space to route the signals.

### Button Inputs:

There might be a little board space left over to squeeze in a a button or two.

### Shift Register

There is a digital I/O shift register shared with the servos.
There are two output for the Lidar enables.
There 16 inputs for reading digital data.
8 of the digital inputs are tied to input connectors.
The remaining 8 inputs are currently unused.

## Conclusions

The Power summary is not good:

* SBC: .9A 
* Raspberry Pi Display: .66A
* WOWBus: 0A (for now, could increase if WOWBus boards ever happen.)
* Motors: 2 * 1.25 (for stalled motors)
* LED's: .32A
* Servos: 4.4A (absolute maximum and quite unlikely)


The absolute worst case is .9A + .66A + .25A + .32A + 4.4A = 6.53A.
The current maximum current allowed is 3.6A, so there is problem here.

The reality is the drive motors are unlikely to be stalled for very long and
the surge capacitors should help out a lot.

Likewise, the servos are unlikely to be at maximum current all at the same time.

Possible solutions are:

* Keep adding vertical micro USB-B headers until there is enough power coming in.

* Go with the current system and measure the worst actual cases.

Suggestions ***EXTREMELY*** are welcome on this issue.

