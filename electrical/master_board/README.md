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

# HR2 Master PCB Board

This is the common design information that is pretty applicable across all revisions
of the HR2 Master PCB Board.
The intention is to always use a higher end STM Nucleo-144 development board
as the embedded microcontroller.
This also allows processor upgrades to occur when newer Nucleo-144 boards come out
by simply upgrading to a newer Nucleo-144 board.
The higher end chips in the STM32Fxxx series of processors try to maintain peripheral
pin mappings across all versions of the 144-LQFP (144 pin Low profile Quad Flat Package.)
Time will tell how good ST Microelectronics is at adhering to this goal.
The nominal Nucleo-144 selected for revision A of the Master board is the Nucleo144-767ZI.
This may change over time with subsequent PCB board revisions.

It is a strong goal of the HR2 support the development of development boards that
plug onto the Nucleo-144.
The Nucleo-144 has two groups of connectors:

* Morpho Connectors:
  The Nucleo-144 has two Morpho connectors are called CN11 and CN12,
  each of which is a 2x35 pin connector.
  In general, the Morpho connectors connect to almost of the pins
  associated with STM32F microcontroller soldered to the Nucleo-144 board.
  The Nucleo-144 board ships without having either of the CN11 and CN12 connectors populated.
  It is necessary to solder connectors for CN11 and CN12.
  The HR2 master PCB sits under the Nucleo-144 and uses two female 2x35 headers to connect to the
  Nucleo-144 Morpho connectors.
  For the HR2, the Nucleo-144 is sufficiently big that it needs to positioned
  above the HR2 drive motors.
  For this reason, the Nucleo-144 needs long pins to bridge the distance
  between the CN11/CN12 connectors and to the master PCB.
  This is accomplished with two 2x35 male "wire-wrap" pin headers that are soldered to the
  bottom of the Nucleo-144 in CN11 and CN12.
  (Wire-wrap was a popular technology for prototyping electronics back in the 70's and 80's
  and it is still used occasionally to this day.)
  The Morpho connectors are exclusively dedicated to be used to connect the Nucleo-144
  to the HR2 master PCB.

* Zio Connectors:
  The Nucleo-144 has four Zio female pin connectors are called CN7 through CN10.
  These four connectors are sized and spaced to be compatible with Arduino shields.
  The difference is that while Arduino shields only use pin headers that are 1 wide,
  the Zio connectors are 2 pins wide.
  The extra row on each connector provides many more connectivity options
  than just simple Arduino pin connectors that are just 1 pin wide do.
  Despite these extra pins,
  the CN11/CN12 Morpho connectors still have pins than the CN7-CN10 Zio pins.

The ultimate goal of the HR2 is to support daughter boards that plug onto the Zio connectors.

The are 4 classes of daughter boards supported by the HR2.
Two of these classes use the Zio connectors and two do not.

* Arduino Shields (uses Zio connectors):
  An Arduino shield has 3 categories of pins:

  * Power/Miscellaneous Pins:
    The power and miscellaneous pins are RESET, 3.3V, 5V, GND, and VIN.
    These pins are easy to deal with and are not really mentioned any further in this document.

  * Digital Pins:
    There are 18 digital I/O pins named D0-D15.

  * Analog Pins:
    There 6 "analog" pins named A0-A5 that can used as analog-to-digital inputs
    or as digital I/O pins.
    Two of the "analog" pins (D4 and D5) are connected to an I2C peripheral.

  There are literally hundreds of Arduino shields that have been developed over the years.
  While  many of these Arduino shields are pretty mediocre, some of them are quite excellent.

* FPGA Daughter Board (uses Zio connectors):
  An FPGA  (Field Programmable Gate Array) daughter board is a PCB with an FPGA chip on it.
  The goal is to either allow the FPGA to totally control the robot or to partially control
  the robot in conjunction with the Nucleo-144 microcontroller.
  In addition, there is no particular reason why the FPGA should not be able to additionally host
  an Arduino shield on top if it so desires.

* Mikrobus Click Boards (does not use Zio connectors):

  The MikroElectronica company has created 20-pin standard peripheral boards
  that is called the MikroBus Click boards.
  These boards power, I2C, SPI, UART, and other pins.
  The come in three sizes -- small, medium and large.
  These 25.4mm wide (i.e. 1 inch) and 28.6mm (small), 42.9mm (medium), and 57.15mm (large).
  The HR2 has 2 Mikrobus connectors -- one small and one medium.
  A large can be plugged into the medium MikroBus connector on the HR2,
  but it will stick out of the robot.
  There are literally hundreds of MikroBus boards.

* Seeed Studio Grove Boards (does not use Zio connectors):
  Seeed Studio has come up with a huge library of peripheral boards that use a 4-pin
  proprietary connector.
  Two of the pins are power and ground and the other two vary from board to board.
  They fall into three broad categories:

  * Digital/Analog I/O: 2 pins of either digital and/or analog input or output.
    
  * I2C bus: 2 pins of I2C bus.

  * Serial: 2 pins of serial I/O (i.e. RX/TX.)

  The are 7 locations to mount Grove boards on the HR2.

The bottom line is that there many expansion opportunities for the HR2.

The following pin terminology is introduced.

* Arduino Pins:
  The Arduino signal pins are the pins that are reserved to connect to an Arduino shield.
  As mentioned above,
  the pins are further sub-divided into "analog" pins (A0-A5), and "digital" pins (D0-D13).
  Note that the "analog" pins can be connected to an A/D converter,
  but they can also be used as digital I/O pins.
  Two of the "analog" pins are bound to an I2C peripheral.
  There are 6 "analog" pins and 14 "digital" pins, for a total of 20 Arduino pins.

* Daughter Pins:
  The daughter pins are the remaining signal pins on the Zio connectors that are not
  are not Arduino Pins.
  It turns out that there are 45 dauhgter pins.

* Morpho Pins:
  The Morpho pins are the signal pins that are routed to the Morpho connectors
  but not also routed to the Zio connectors.
  It turns out that there are 17 Morpho pins.

* Nucleo Pins:
  The Nucleo-144 development board connects also connects some of its signal wires
  to the Morpho/Zio connectors.
  This is to be expected.
  Most of these pins are for the Ethernet connector, USB connector, and the ST-link signals.
  With three exceptions, Nucleo pins only connect to the Morph pins mentioned immeidately above.
  The three exceptions are:

  * PA7:
    PA7 is connected to the Ethernet interface chip.
    If you want to use the Ethernet interface,
    you must not attempt to plug in an Arduino shield that also needs to use D11.

  * PB0:
    PB0 is connected to LED called LD1.
    It is pretty harmless if the LED flickers if it is being used for some other purpose.

  * PB3:
    PB3 is connected to the SWO signal that technically is used by the ST-Link.
    However, the SWO signal is currently not used by the ST-Link, so it is actually available.

By the way, the pin name conventions are described it greater detail below.

To summarize, the Ardunio, Daughter, and Morpho pin sets are mutually exclusive with no overlap.
The Nucleo pins are mostly in the Morpho pin set with just the three exceptions,
two of which are harmless and one is annoying (e.g PA7 == D11.)

In general, what is desired is to route as many of the robot control pins to Daughter pins.
What all of this means is that Daughter pins are a central design constraint for the HR2.

## Pin Binding

The task of mapping internal chip signals => phyisical chip package pins => Morpho/Zio connectors
is called "pin binding" and is a very, very complex process with many constraints and trade-offs.

It useful to starting with a consistent definition of some common terms:

* Package:
  Microcontrollers come in a variety of packages different packages.
  For this, project the 144-pin LQFP package is used.
  LQFP the common abbreviation for Low profile Quad Flat Pack.
  It is a square package with 36 "gull wing" pins on each side with a distance of .5mm between pins.

* Pin Number:
  Each pin on the package is given a number.
  For the 144-LQFP, the names are simply numbers from 1 to 144.
  (Other packages, like ball grid packages that are arranged in a grid,
  use letter and an number.)

* Pin Name:
  Each pin on the package is also given a name (e.g. "GND", "VSS", "RESET", "PC0", etc.)
  Some pin names, are assigned to multiple package pins.
  This is particularly true of power and ground pins which are electrially connected to each other
  inside of the chip.
  Again, power and ground pins are not interesting to pin binding are are not dicussed any further.
  However, each signal pin is given a unique name.
  This is 1-to-1 binding between the pin number and the pin name.

* Peripheral:
  A peripheral is a chunk circuitry inside the microcontroller that implements some functionality
  (e.g. a timer, a UART, and I2C bus manager, etc.)
  There can be multiple similar peripherals (e.g. UART1, UART2, etc.)
  Each different microcontroller has a different set of peripherals.

* Internal Signal:
  An internal signal is usually (but not always) associated with a peripheral
  (e.g. UART1_TX, TIM1_CH1, etc.)


The vast majority microcontroller are pin limited in the sense that they have more
internal signals than available pins to access these signals.
The task of pin binding is to get the signals that needed for a project
out of the microcontroller and onto the PCB.

Most of the pins on a microcontroller are I/O (Input/Output) pins.
For an STM32Fxxx microcontroller, the I/O pins are organized into 16-bit I/O ports.
The ports are named "PA", "PB", "PC", etc.
The 'P' stands for "port" and the following letter stands identifies which 16-bit I/O port it is.
There are generally 16 pins associated with each port,
although some ports do not manage to get all of the bits assigned to actual package pin numbers.
Each I/O port pin is also number from 0 to 15.
A port pin name is just the port name followed by the pin number (e.g. "PA0", "PC14", etc.)
These port names are assigned 1-to-1 with package pin numbers.
Thus, PA0 can be assigned to package pin 123, and PA1 to package pin 12, etc.
For the rest of this document, the term "port pin name" is shorted to just "pin name".
Thus, PA0 and PG15 are pin names.

For the STM32F767ZI, their are 8 I/O ports labeled PA through PH.
The PH pins are generally not accessible, since the are connected to oscillator crystals.
Thus, only PA through PG ports are generally used.

For the SM32Fxxx series, routing internal signals to I/O pins is done using "alternate functions".
Each I/O pin can have up to 15 alternate function signal mappings called AF0 through AF14.
Internally, for each port pin, there are 4 bits that can specify the alternative function.
The values 0b0000 through 0b1110 are for alternative functions
and 0b1111 specifies the that pin is just a plain digital I/O pin.
Only one of the 15 alternative functions can be applied to a particular pin at a given time.
While it is technically feasible to dynamically switch between alternative functions
during microcontroller execution, but in practice this is rarely done.
Instead, the alternative function registers are set up once during the boot up process
and left alone thereafter.

There are many internal microcontroller peripherals.
The peripheral names tend to be of the form "KIND###", where "###" is a number.
For example, "TIM1", "TIM2", "USART1", "USART2", "I2C1", "I2C2", etc.
In addition, these peripherals also have one or more signal names associated with them.
These signal names are appended to the peripheral name separated by and underscore '_'.
For example, "TIM1_CH1", "USART2_TX", "USART6_RX", "I2C1_SCK", "I2C1_SDA", etc.
For most (but not all) peripherals,
a given signal can be attached to more than one I/O pin using an alternate function.

Finally, one last issue is that on the schematic,
the internal signal names are usually given a logical name that us used on the schemetic,
rather the chip internal name.

To summarize, pin binding involves selecting the desired peripherals and digital I/O pins,
selecting the needed internal signals and assigning the pin names with an associated
alternate function.

1. Peripheral Name:
   TIM1, I2C2, USART3, etc.

2. Signal Name:
   TIM1_CH1, I1C2_SDA, USART3_TX, etc.

3. Pin Name:
   PA1, PB6, PF15, etc.

4. Alternate Function:
   AF0, AF3, AF14, digital/IO, etc.
   
5. Package Pin number (not terribly important):
   12, 42, 123, etc.

6. Morpho Connector Name and Pin:
   CN10-13 or CN11-70.

7. Zio Connector Name and Pin (if available):
   CN7-1, CN8-7, CN9-18, or CN10-23, etc.

8. Schematic Logical Name:
   LIDAR_PWM, DEBUG_TX, LEDS_NSS, SONAR3_ECHO, etc.

Figuring all of this information out is pretty involved,
but the board will not correctly unless these bindings are correct.

## Peripherals

The peripherals of interest to the HR2 Master board are:

* Timers:
  A timer can be used to either generate or measure pulses.
  There is a special case of the timer which it use it to read in a quadrature encoder signal.

* UART's:
  A UART (Universal Asynchronous Receiver Transmitter) is used to send and receive
  data over a serial line.

* I2C's:
  An I2C peripheral is used to talk to chips attached via the I2C bus protocol.

* SPI's:
  An SPI peripheral is used to talk to a chips that use the SPI bus protocol.a

* A/D pins:
  Some of the pins are attached to the Analog to Digital converter.

* Digital I/O pins:
  Some pins are dedicated to simple digital I/O.

* External Interrupt pins:
  Most pins can also be used to generate external interrupt signals.

A brief descriptions of the various peripherals follows below:

### Timers

There are at total of 15 timer modules in STM32F767.
These are summarized in the table below:

     Kind       Count  Names             Size     Encoder  Inputs + Outputs
     ============================================================================
     Advanced   2      TIM1/8            16-bits  Yes      4 + 4 + 2 inverted
     Medium32   2      TIM2/5            32-bits  Yes      4 + 4
     Medium16   2      TIM3/4            16-bits  Yes      4 + 4
     Basic      2      TIM6/7            16-bits  No       0 + 0 (internal only)
     Simple2    2      TIM9/12           16-bits  No       2 + 2      
     Simple1    4      TIM10/11/13/14    16-bits  No       1 + 1
     Low Power  1      LPTIM             16-bits  Yes      2 + 1

When the time encoder function is used, they have the following extra constraints:

* For the STM32F6xxx series, only CH1 and CH2 can be used for the encoder inputs.

* In encoder mode, none of the other inputs or outputs can be used in any useful way.
  The other timer internal signals become effectively unusable for encoder mode.

The HR2 has the following timer needs:

* 2 Encoders:
  There are 2 encoders and each encoder requires two inputs signals.
  The encoder mode for the STM requires 2 timer inputs per encoder.

* HAL (Hardware Abstraction Layer.):
  The Hardware Abstraction layer uses TIM6 which is an internal only timer.

* 16 LED's:
  There are 16 LED's.
  The original fantasy was to put each LED on a dedicated Timer so that it could be
  pulse width modulated.
  This chewed up way too many timers and there was no way to make all of the pin
  bindings work.
  So an alternative strategy was selected.
  The alternative strategy is to use a 16-bit shift register with a latch function to
  control the 16 LED's.
  The 16-bit shift register is filled with using a single SPI device.
  This reduces the pin requirements from 16 to 3 (SCK, SDA, and NSS.)
  The LED wave forms are copied from memory to the SPI device using the DMA device.
  The update rate is controlled by the a single internally generated timer.
  Thus, the LED's need one internal timer.

* 1 Lidar:
  Some of the less expensive Lidars out there need a PWM signal for the motor control.

* 2 Motors:
  There are two drive motors.
  Each motor driver has two inputs where one input is active PWM and the other side
  is either high or low.
  There is no need to PWM both inputs at the same time.
  Another way to think of it is that one side will fractional PWM
  and the other side will be either 100% or 0% PWM.
  The PWM pins for each motor need to be synchronized,
  but each motor can be on a different peripheral.
  This will chew up a total of 2 timer outputs for both motors for a total of 4 timer outputs.

* 4 Servos:
  There a 3 servos for the arm and 1 extra servo.
  Accurate pulse widths between 1ms and 2s are very desirable to prevent servo chatter.
  Servos only need to be updated approximately every 20ms with a 1ms to 2ms wide pulse.
  Making the pulse width very accurate is extremely desirable,
  but the inter pulse delay time is not that critical.
  Very accurate pulse widths become easier if the timer is 32-bits.
  There are not very many 32-bit timers available.
  Most of the timers are 16-bit timers.
  The 4 servers need 4 timer outputs, but they do not all have to be on the same timer.

* 6 Arduino PWM's:
  The Arduino has 6 pins that are specified to support PWM outputs.
  This pins are D3, D5, D6, D9, D10, and D11.
  The 6 PWM outputs consume 6 timer outputs, but the do not have to be on the same timer.

* 7 Sonar's:
  There is one trigger and one echo line per sonar.
  The plan is to detect the echos using the External Interrupt functionality.
  When the echo pin rises, a free running counter is read.
  When the echo pin falls, the same free running counter is read.
  The difference between the two counter values (dealing with counter wrapping)
  is the pulse width.
  There are some interesting constraints concerning external interrupts that are
  discussed in further below in the sonars section.
  The timer requirements for the sonar is 1 free running timer.

<!--
	  The w
  It is a little strange because,
  there are 16 pin change interrupts and they can be mappped to pretty my any GPIO pin.
  It is only possible to select one pin N form PA, ..., PJ for external interrupt.
  Thus, PA0, PB1, ..., PJ15 would work, PA0, PA1, ..., PA15,
  or some mixture mixture of PA0, PB1, PA2, PC3, ...., PB15.
  The SYSCFG registers are used to set the pin selections up
  in addition to the External Interrupts (EXTI).
  There needs to be one a free running timer to time length of the echo pulses.
  This uses up 14 GPIO pins and maybe one internal timer.
-->

The Timer requirements summary is:

     Function  Inputs  Outputs  Timers  Notes
     ===========================================================================================
     Encoders  4       0        2       1 timer per encoder (CH1 and CH2 only)
     HAL       0       1 (int)  1       TIM6 dedicated to HAL
     LED's:    0       1 (int)  1       Triggers DMA to write to LED's
     Lidar:    0       1        1       1 PWM
     Motors:   0       4        1 or 2  1 timer w/4 channels or 2 timers w/2 channels
     Servos:   0       4        1 or 2  32-bit timers strongly preferred.
     Arduino:  0       6        ?       6 PWM outputs are required
     Sonars:   0       0        1       1 free running needed for interrupt driven echo signals

It turns out that timer pin binding for the HR2 is extremely difficult due to the
a variety of reasons.

### SPI (Serial Peripheral Interface)

An SPI peripheral is capable of interacting with SPI enabled peripherals.
There are 6 SPI peripherals available for the STM32F767.
They are named SPI1 through SPI6.
SPI1 through SPI3 support the I2S serial sound mode and SPI4 through SPI6 do not.

The HR2 SPI requirements are currently that one SPI is needed to drive the LED shift register.

### USART's

USART stands for Universal Synchronous Asynchronous Receiver Transmitter.
There are 8 USART's:

* USART1/USART2/USART3/USART6:
  These support all USART functionality.

* UART4/UART5/UART7/UART7:
  These do everything except synchronous and smartcard mode.

The HR2 requirements for USART's are:

The USART requirements are:

* Arduino Shield USART:
  Many Arduino shields need a dedicated serial connection.
  For the Nucleo, UART6 is connected to these pins.

* ST-Link debug USART:
  The ST-Link supports a serial port over USB.
  This is traditionally USART3.

* SBC (Single Board Computer):
  The single board computer (e.g. Raspberry Pi) needs to be able to talk to the microcontroller.

* Lidar:
  Many LIDAR's generate a pretty high stream of data over a serial link for their point cloud.

* WOWBus (Wayne's Omnipotent Wonderful Bus):
  Despite the silly name,
  this a multi-drop serial bus that uses CAN bus transceivers to send serial data between modules.
  In addition, there is an emergency stop bus for signaling an emergency stop condition.

* FTDI Debug USART (Morpho Only):
  This yet another USART that can be used for debugging.

* FPGA USART (Zio Only):
  This is a dedicated USART for talking to the FPGA.

## ZIO Connectors Mapping Table Grove boards

In the
[STM32 Nucleo-144 Board User Manual](../docs/stm32_nucleo_144_manual.pdf),
there is a table that corresponds to the Morpho connectors.
On pages 38 through 41 are Morpho pin-outs for the Nucleo144-STM32F767F.
These pages were extracted into small and easily searchable
[STM32 Nucleo-144 Board User Manual Pages 38-41](../docs/stm32_nucleo_144_manual_pages38-41.pdf)
file by reading the file into `libreoffice` and writing out just pages 38 through 41.
Using this file it is possible to easily find connector/pin name/Arduino bindings.
The dense table is shown below where the Arduino signals are prefixed with an `@`:

        PA:                      PB:                      PC:                 PD:
        PA0:  CN10-29:           PB0:  CN10-31:           PC0:  CN9-3:   @A1  PD0:  CN9-25:
        PA1:  -------            PB1:  CN10-7:            PC1:  -------:      PD1:  CN9-27:
        PA2:  -------            PB2:  CN10-15:           PC2:  CN10-9:       PD2:  CN8-12:
        PA3:  CN9-1: @A0         PB3:  CN7-15:            PC3:  CN9-5:   @A2  PD3:  CN9-10:
        PA4:  CN7-17:            PB4:  CN7-19:            PC4:  -------:      PD4:  CN9-8:
        PA5:  CN7-10:@D13        PB5:  CN7-13:            PC5:  -------:      PD5:  CN9-6:
        PA6:  CN7-12:@D12        PB6:  CN10-13:           PC6:  CN7-1:        PD5:  CN9-4:
        PA7:  CN9-14:(1,2) @D11  PB7:  -------:           PC7:  CN7-11:       PD6:  CN9-2:
        PA8:  -------            PB8:  CN7-2:   (1) @D15  PC8:  CN8-2:        PD8:  -------:
        PA9:  -------            PB9:  CN7-4:   (1) @D14  PC9:  CN8-4:        PD9:  -------:
        PA10: -------            PB10: CN10-32            PC10: CN8-6:        PD10: -------:
        PA11: -------            PB11: CN7-34:            PC11: CN8-8:        PD11: CN10-23:
        PA12: -------            PB12: CN7-7:             PC12: CN8-10:       PD12: CN10-21:
        PA13: -------            PB13: CN7-5:             PC13: -------:      PD13: CN10-19:
        PA14: -------            PB14: -------:           PC14: -------:      PD14: CN7-16:  @D10
        PA15: CN7-9:             PB15: CN7-3:             PC15: -------:      PD15: CN7-18:  @D9

        PE:                      PF:                      PG:                 Misc:
        PE0:  CN10-34:           PF0:  CN9-21:            PG0:  CN9-29:       NRST: CN8-5    @NRST
        PE1:  -------:           PF1:  CN9-19:            PG1:  CN9-30:
        PE2:  CN10-25: (3)       PF2:  CN9-17:            PG2:  CN8-14:
        PE3:  CN9-22:            PF3:  CN9-7:   @A3       PG3:  CN8-16:
        PE4:  CN9-16:            PF4:  CN7-11:            PG4:  -------:
        PE5:  CN9-18:            PF5:  CN9-9:   (1) @A4   PG5:  -------:
        PE6:  CN9-20:            PF6:  -------:           PG6:  -------:
        PE7:  CN10-20:           PF7:  CN9-26:            PG7:  -------:
        PE8:  CN10-18:           PF8:  CN9-24:            PG8:  -------:
        PE9:  CN10-4:  @D6       PF9:  CN9-28:            PG9:  CN10-16: @D0
        PE10: CN10-24:           PF10: CN9-11:  (1) @A5   PG10: -------:
        PE11: CN10-6:  @D5       PF11: -------:           PG11: -------:
        PE12: CN10-26:           PF12: CN7-20:  @D8       PG12: -------:
        PE13: CN10-10: @D3       PF13: CN10-2   @D7       PG13: -------:
        PE14: CN10-28:           PF14: CN10-8:  @D4       PG14: CN10-14: @D1
        PE15: CN10-30:           PF15: CN10-12: @D2       PG15: -------:

The ZIO Mapping Table Footnotes are:

1. For more details refer to Table 12: Solder bridges.

2. PA7 is used as D11 and connected to CN7 pin 14 by default,
   if JP6 is ON, it is also connected to both Ethernet PHY as RMII_DV and CN9 pin 15.
   In this case only one function of the Ethernet or D11 must be used.
   By the way, this is really quite annoying.

3. PE2 is connected to both CN9 pin 14 (SAI_A_MCLK) and CN10 pin 25 (QSPI_BK1_IO2).  Only one
   function must be used at one time.

4. PB13 is used as I2S_A_CK and connected to CN7 pin 5 by default, if JP7 is ON, it is also
   connected to Ethernet PHY as RMII_TXD1.
   In this case, only one function of the Ethernet or I2S_A must be used.


## Alternative Functions Table

The alternative functions table is used as part of the pin binding process.

The Alternative Functions table generated as follows:

1. Start up `stm32cubeide` program that resides in the HR2 `.../bin/` directory.

2. Start a dummy project that uses the STM32F767 processor with an 144-LPQF package. 

3  Once you have a graphical representation of the chip shows,
   click on the [Pinout] menu and select `Export pinout with Alt Functions`.
   This file was saved to the file `(stm32f767_af.csv)[stm32f767_af.csv]`.

The `stm32f676_af.csv` file is read by the `pins_bind.py` program discuesed immediately below.

## `pins_bind.py`

To program to manage the HR2 pin binding process is a program called (`pinsbind.py`)[pinsbind.py].
This file can be executed by the (`Makefile`)[Makefile] in the same directory.
This program reads the previously generated `stm32f767_af.csv` file figures everything out.

The pins are bound in an order that attempts to leave as many options open as possible.
The following order is used:

* Arduino pins:
  The Arduino pins are totally fixed, so they get bound first.

* Nucleo pins:
  The Nucleo pins are totally fixed, so they go in next.
  The PA7 pin for RMII_CRS_DV, is left as an Arduino pin.
  Over time, some of the Morpho pins are likely to get used for other purposes.
  It is likely the TMS/TCK will go first, followed by the USB signals.
  After that is will really start to hurt.

* STLINK:
  It is really important that the ST-Link work.
  This just involves 3 software download and debug pins called SWCLK (PA14), SWDIO (PA13), and NRST.
  The SWCLK and SWDIO pins must be bound early.
  In addition there are two pins connected to a UART on the STLink board.
  Traditionally, these are connected to UART3 on the Nucleo chip,
  but this is not a hard requirement.
  Finally, there is a pin called SWO that is reserved by currently unused by the STLink.
  So, the SWO pin is actually free to use and does not need to be bound.

* Ethernet:
  Keeping the Nucleo Ethernet connector alive is more important than
  keeping the USB connector alive.
  With the exception of the Arduino D11 pin (i.e PA7),
  there should be no problems avoiding the Ethernet pins.
  
* USB:
  The Nucleo USB connector can be bound next.
  However, it option if some Morpho pins are needed for something else.
  The USB is less interesting than the Ethernet,
  so the USB pins are going to be raided before the Ethernet pins.

* Arduino:
  The Arduino pins are fixed and can not be moved.
  However the some of the pins want to be routed to timers, A/D pins, SPI peripheral
  I2C peripheral, etc.
  The requirements are:

  * A2D inputs (A0-A5):
    (A0=ADC123IN(PA3), A1=ADC123_IN1(PC0), A2=ADC123_IN113(PC3),
    A3=ADC_IN9(PF3), A4=ADC_IN15(PF5), A5=ADC_IN8(PF10).

  * PWM output pins:
    (D3=TIM1_CH3, D5=TIM1_CH2, D6: TIM_CH1, D9=TIM4_CH4, D10=TIM4_CH3, D11=TIM3_CH1)

  * SPI I/O pins:
    (D10=SPI[1|6]_NSS, DD11=SPI[1|6]_MOSI, D12=SPI[1|6]_MISO, D13=SPI[1|6]_SCK)

  * I2C I/O pins:
    (A4:I2C?/SDA A5:I2C?/SCL)

  The Nucleo does not currently does route any I2C peripheral to A4/A5;
  this is a problem that needs to be solved wiring a couple of I2C signal pins
  to a couple of spare daughter pins.

* Timers:
  Timers are a nightmare that are discussed below.
  Frankly, everything was eventually figured out.

* I2C, SPI, UARTS, SONARS.


The trick to using the program is to specify the order in which to bind the pins.

The tricky thing to figure out was the timer bindings.
It was not possible to accomplish the task without cross connecting one Morpho pin
to a Daughter pin.

## Timer Peripheral Bindings

The connectivity of the Timers to the Zio connectors is listed alphabetical order below.
In the assignments below, square brackets ([...]) indicate a Morpho pin,
curly braces ({...}) mean an Ardunio pin, and angle brackets (<...>) mean a Daughter pin.
The schematic symbol name is in parentheses (...).
The assignments are summarized below with further discussion immediately following:

* LPTIM: LENCODER
  * LPTIM_IN1: <PD12:AF3> (LENCODER_A) ; Or [PG12:AF3]
  * LPTIM_IN2: [PE1:AF3] (LENCODER_B) [Morpho pin needs to be shorted to Daughter pin ??]

* TIM1: Ardunio PWM
  * TIM1_CH1: {PE9:AF3} (D6_PWM) ; [or PA8:AF1]
  * TIM1_CH2: {PE11:AF3} (D2_PWM) ; [or PA9:AF1]
  * TIM1_CH3: {PE13:AF3} (D3_PWM) ; [or PA10:AF1]
  * TIM1_CH4: (unused) ; {or PE14:AF1} [or PA11:AF1]

* TIM2: SERVOS (32-bits)
  * TIM2_CH1: (unused) <or Daughter PA0:AF1, PA15:AF1> {or PA0:AF1}
  * TIM2_CH2: <PB3:AF1> (SERVO2) [or PA1:AF1]
  * TIM2_CH3: <PB10:AF1> (SERVO3) [or PA2:AF1]
  * TIM2_CH4: <PB11:AF1> (SERVO4) {or PA3:AF1}

* TIM3: LMOTOR + RMOTOR
  * TIM3_CH1: <PB4:AF2> (LMOTOR+) <or Daughter PC6:AF2> {or PA6:AF2}
  * TIM3_CH2: <PB5:AF2> (LMOTOR-) <or Daughter PC7:AF2> {or PA7:AF2}
  * TIM3_CH3: <PC8:AF2> (RMOTOR+) <or Daughter PB0:AF2>
  * TIME_CH4: <PB1:AF2> (RMOTOR-) <or Daughter PB1:AF2>

* TIM4: Arduino PWM
  * TIM4_CH1: (unused) <or Daughter PD12:AF2 or PB6:AF2>
  * TIM4_CH2: (unused) <or Daughter PD13:AF2> [or PB7:AF2]
  * TIM4_CH3: <PD14:AF2> (D10_PWM) {or PB8:AF2}
  * TIM4_CH4: <PD15:AF2> (D9_PWM) {or PB9:AF2}

* TIM5: SERVOS (32-bits)
  * TIM5_CH1: <PA0:AF2> (SERVO1)
  * TIM5_CH2: (unused) [or PA1:AF2]
  * TIM5_CH3: (unused) [or PA2:AF2]
  * TIM5_CH4: (unused) {or PA3:AF2}

* TIM6: HAL (internal only)

* TIM7: LEDS (internal only)

* TIM8: RENCODER
  * TIM8_CH1: <PC6:AF3> (RENCODER_A)
  * TIM8_CH2: <PC7:AF3> (RENCODER_B)
  * TIM8_CH3: (unused due to encoder) <or PC8:AF3>
  * TIM8_CH4: (unused due to encoder) <or PC9:AF3>

* TIM9: Unused
   * TIM9_CH1: (unused) <or Daughter PE5:AF3> [or PA2:AF3]
   * TIM9_CH2: (unused) <or Daughter PE6:AF3> [or PA3:AF3]

* TIM10: (Unused)
   * TIM10_CH1: (unused) [or Morpho PF6:AF3] {or PB8:AF3}

* TIM11: (Unused)
   * TIM11_CH1: (unused) <or Daughter PF7:AF3> {or PB9:AF3}

* TIM12: (Unused)
   * TIM12_CH1: (unused) [or Morpho PB14:AF9]
   * TIM12_CH2: (unused) <or Daughter PB15:AF9>

* TIM13: LIDAR
   * TIM13_CH1: <PF8:AF9> (LIDAR_PWM) {or PA7:AF9}

* TIM14: Arduino PWM
  *  TIM14_CH1: <PF9:AF9> (D11_PWM) <or PA7:AF9>

The summary of how the timer binding went is summarized below:

* Servos:
  The servos work best with 32-bit counters (i.e. TIM2/5).  It turns out that
  there are exactly 4 Zio available pins to TIM2/5, so that are used for the servos.

* Encoders:
  The encoders are a real challenge.
  TIM8 has CH1/2 available on the Zio, so it got assigned to the RENCODER.
  The none of the other encoder enabled timers (TIM1/2/3/4/5/6/8) have both CH1/2 available.
  This is fixed by shorting a Morpho pin to LPTIM1_IN2.
  This allows LPTIM1 to be used for LENCODER.

* Motors:
  Motors turned out to be easy.  They were all assigned to TIM_3_CH1/2/3/4.

* LED's:
  The LED's just need an internal timer to trigger the DMA peripheral.
  TIM6 is used by HAL, so TIM7 will have to do.
  No actual pin needs to bound since the signal is strictly internal.

* Lidar:
  The Lidar was stuffed onto TIM14_CH1, but there are many other options.

There are some timers left over, but not many -- TIM9/10/11/12/13/14.
That is it.
In general, Timer pin binding was every bit of a nightmare as expected.

## SPI Peripherals



<!--

UART's:

UART4

USART1:
    USART1_RX: ('+PB15:AF4', '-PA10:AF7', '-PB7:AF7')                PB15
    USART1_TX: ('-PB14:AF4', '-PA9:AF7', '+PB6:AF7')                 PB6
USART2:
    USART2_RX: ('+PD6:AF7',)                                         PD6
    USART2_TX: ('-PA2:AF7', '+PD5:AF7')                              PD5
USART3:
    USART3_RX: ('+PB11:AF7', '-PD9:AF7', '+PC11:AF7')                PB11 or PC11
    USART3_TX: ('+PB10:AF7', '-PD8:AF7', '+PC10:AF7')                PB12 or PC10
UART4:
    UART4_RX: ('-PA1:AF8', '-PA11:AF6', '+PC11:AF8', '+PD0:AF8')     PC11 or PD0
    UART4_TX: ('+PA0:AF8', '-PA12:AF6', '+PC10:AF8', '+PD1:AF8')     PC10 or PD1
UART5:
    UART5_RX: ('+PB12:AF8', '+PD2:AF8')                              PB12 or PD2
    UART5_TX: ('+PC12:AF8', '+PB6:AF1')                              PC12 or PB6
UART6
    USART6_RX: ('+PC7:AF8',)                                         PC6 or PF8
    USART6_TX: ('+PC6:AF8',)
UART7:
    UART7_RX: ('-PF6:AF8', '+PE7:AF8', '-PA8:AF12', '+PB3:AF12')     PE7 or PB3
    UART7_TX: ('+PF7:AF8', '+PE8:AF8', '+PA15:AF12', '+PB4:AF12')    PF7 or PE8 or PA15 or PB12
UART8:
    UART8_RX: ('+PE0:AF8',)
    UART8_TX: ('-PE1:AF8',)

-->

