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
  In general, the Morpho connectors connect to almost (but not quite all) of the associated
  STM32F microcontroller mounted on the Nucleo-144 board.
  The Nucleo-144 board ships without having either of the CN11 and CN12 connectors populated.
  It is necessary to solder connectors for CN11 and CN12.
  The master PCB sits under the Nucleo-144 and uses two female 2x35 headers to connect to the
  Nucleo-144 Morpho connectors.
  For the HR2, the Nucleo-144 is sufficiently big that it needs to positioned
  above the HR2 drive motors.
  For this reason, the Nucleo-144 needs long pins to bridge the distance
  between the CN11/CN12 connectors and to the master PCB.
  This is accomplished with two 2x35 male "wire-wrap" pin headers that are soldered to the
  bottom of the Nucleo-144 in CN11 and CN12.
  (Wire-wrap was a popular technology for prototyping electronics back in the 70's and 80's and
  it is still used occasionally to this day.)
  The Morpho connectors are dedicated to be used to connect the Nucleo-144 to the master PCB.

* Zio Connectors:
  The Nucleo-144 has four Zio female pin connectors are called CN7 through CN10.
  These four connectors are sized and spaced to be compatible with Arduino shields.
  The difference is that while Arduino shields only use pin headers that are 1 wide,
  the Zio connectors are 2 pins wide.
  The extra row provides more connectivity options than just simple Arduino 1 wide
  pin connectors do.
  Despite these extra pins, the CN11/CN12 Morpho connectors still have pins than the
  CN7-CN10 Zio pins.

The ultimate goal of the HR2 is to support daughter boards that plug onto the Zio connectors.

The are two broad classes of daughter boards:

* Arduino Shields:
  An Arduino shield has 3 categories of pins:

  * Power/Miscellaneous Pins: RESET, 3.3V, 5V, GND, and VIN.

  * Digital Pins: There are 18 digital I/O pins named D0-D15 and D18-D19.

  * Analog Pins: There 6 "analog" pins named A0-A5 that can used as analog-to-digital inputs or
    as digital I/O pins.

  There are literally hundreds of Arduino shields that have been developed over the years.
  (Frankly, many of them are not very good.)

* FPGA Daughter Board:
  An FPGA  (Field Programmable Gate Array) daughter board is a PCB with an FPGA chip on it.
  The goal is to either allow the FPGA to totally control the robot or to partially control
  the robot in conjunction with Nucleo-144 microcontroller.
  In addition, there is no particular reason why the FPGA should not be able to
  host an Arduino shield on top if it so desires.

What all of this means is that ZIO connectors are a central design constraint for the HR2.
Since there are fewer ZIO pins, not all of the signals on the Morpho pins can be mapped
to the ZIO pins.

A further constraint is that ideally, all of the pins needed to control the robot
should be routed to ZIO pins that do not intersect the with the Arduino Shield pins.
It may not be possible to meet this last constraint, but it is worth a try.

## Pin Binding

The task of mapping signals inside the chip to pins outside the on the package is
called "pin binding" and is a very, very complex process with many constraints and trade-offs.

Start with a definition of some common terms:

* Package:
  Microcontrollers come in a variety of packages different packages.
  For this, project the 144-pin LQFP package is used.

* Pin Number:
  Each pin on the package is given a name.
  For the 144-LQFP, the names are simply numbers from 1 to 144.

* Pin Name:
  Each pin on the package is also given a name (e.g. "GND", "VSS", "RESET", "PC0", etc.)
  Some pin names, are assigned to multiple package pins.  This is particularly true
  of ground and power pins.
  All pins with the same name are electrically connected to one another inside the chip.

* Peripheral:
  A peripheral is a chunk circuitry inside the microcontroller that implements some functionality
  (e.g. a timer, a UART, and I2C bus manager, etc.)
  There can be multiple simple peripherals (e.g. UART1, UART2, etc.)
  Each different microcontroller has a peripheral set.

* Internal Signal:
  An internal signal is usually (but not always) associated with a peripheral
  (e.g. UART1_TX, TIM1_CH1, etc.)

The vast majority microcontrollers are pin limited in the sense that they have more
internal signals than available pins to access these signals.
The task of pin binding is to get the signals that needed for a project
out of the microcontroller and onto the PCB.

Most of the pins on these microcontrollers are I/O (Input/Output) pins.
For the STM32Fxxx microcontrollers, the I/O pins are organized into 16-bit I/O ports.
The ports are named "PA", "PB", "PC", etc.
The 'P' stands for "port" and the following letter stands identifies which 16-bit I/O port it is.
There are generally 16 pins associated with each port,
although some ports do not manage to get all of the bits assigned to actual pack pin numbers.
Each I/O port pin is also number from 0 to 15.
A port pin name is just the port name followed by the pin number (e.g. "PA0", "PC14", etc.)
These port names are assigned 1-to-1 with package pin numbers.
Thus, PA0 can be assigned to package pin 123, and PA1 to package pin 12.

For the STM32F767ZI, their are 8 I/O ports labeled PA through PH.
The PH pins are generally not accessible, only PA through PG are generally used.

For the SM32Fxxx series, routing internal signals to I/O pins is done using "alternate functions".
Each I/O pin can have up to 15 alternate function signal mappings called AF0 through AF14.
(Internally, for each port pin, there are 4 bits that can specify the alternative function.
The values 0b0000 through 0b1110 are for alternative functions
and 0b1111 specifies the that pin is just a plain digital I/O pin.)
Only one of the 15 alternative functions can be applied to a particular pin at a given time.
It is technically feasible to dynamically switch between alternative functions
during microcontroller execution, but in practice this is never done.
Instead, the alternative function registers are set up once during the boot up process
and left alone thereafter.

There are many internal microcontroller peripherals.
The peripheral names tend to be of the form "KIND###", where "###" is a number.
For example, "TIM1", "TIM2", "USART1", "USART2", "I2C1", "I2C2", etc.
In addition, these peripherals also have one or more signal names associated with them.
These signal names are appended to the peripheral name separated by and underscore '_'.
For example, "TIM1_CH1", "USART_TX", "USART_RX", "I2C1_SCK", "I2C1_SDA", etc.
For most (but not all) peripherals,
a given signal can be attached to more than one I/O pin using an alternate function.
One last issue is that on the schematic,
the internal signal names are usually given a logical name for what the signal is used for.

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
but the board will not work to specification unless these bindings are correct.

## Peripherals

The peripherals of interest to the HR2 Master board are:

* Timers:
  A timer can be used to either generate or measure pulses.
  There is a special case of the timer which it use it to read in a quadrature encoder signal.

* UART's:
  A UART (Universal Asynchronous Receiver Transmitter) is used to send and receive
  data over a serial line.

* I2C's:
  An I2C peripheral is used to talk to chips attached to an I2C bus.

* SPI's:
  An SPI peripheral is used to talk to a chips that use the SPI bus protocol.

* A/D pins:
  Some of the pins are attached to the Analog to Digital converter.

* Digital I/O pins:
  Some pins are dedicated to simple digital I/O.

### Timers

There are at total of 15 timer modules in STM32F767.
These are summarized in the table below:

     Kind       Count  Names               Size     Encoder  Outputs
     =====================================================================
     Low Power  1      LPRTIM              16-bits  Yes      ?
     Advanced   2      TIM1/8              16-bits  Yes      4
     Medium32   2      TIM2/3/4/5          32-bits  Yes      4
     Medium16   2      TIM2/3/4/5          16-bits  Yes      4
     Basic      2      TIM6/7              16-bits  No       0 (internal only)
     Simple2    2      TIM9/12             16-bits  No       2       
     Simple1    4      TIM10/11/13/14      16-bits  No       1

When the encoder function is used they have the following extra constraints:

* Only CH1 and CH2 can be used for the encoder inputs.

* In encoder mode, none of the other inputs or outputs can be used.

The HR2 has the following timer needs:

* 2 Encoders:
  There are 2 encoders and each encoder requires two inputs signals.
  The encoder mode for the STM requires 2 timer inputs per encoder.
* HAL (Hardware Abstraction Layer.):
  The Hardware Abstraction layer uses TIM6.
* 16 LED's:
  There are 16 LED's.
  The original fantasy was to put each LED on a dedicated Timer so that it could be
  pulse width modulated.
  This chewed up way too many timers and there was no way to make all of the pin
  bindings work.
  Ultimately, the adopted strategy is to put the LED's onto latch-able 16-bit shift register
  that can is controlled by a filled using an SPI device.
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
  Another way to to think of it is that one side will fractional PWM
  and the other side will be either 100% or 0% PWM.
  The PWM pins for each motor need to be synchronized,
  but each motor can be on a different peripheral.
  This will chew up a total of 4 timer outputs for both motors.
* 4 Servos:
  There a 3 servos for the arm and 1 extra servo.
  Accurate pulse widths between 1ms and 2s are very desirable to prevent servo chatter.
  Servos only need to be updated approximately every 20ms with a pulse that is between 1ms and 2ms.
  Making the pulse width very accurate is a requirement,
  but the inter pulse time is not that critical.
  This becomes easier if the timer is 32-bits.
* 7 Sonar's:
  There is one trigger and one echo line per sonar.
  Again the concept is to detect the echos using the External Interrupt functionality.
  It is a little strange because,
  there are 16 pin change interrupts and they can be mappped to pretty my any GPIO pin.
  It is only possible to select one pin N form PA, ..., PJ for external interrupt.
  Thus, PA0, PB1, ..., PJ15, would work or PA0, PA1, ..., PA15,
  of some mixture of PA0, PB1, PA2, PC3, ...., PB15.
  The SYSCFG registers are used to set the pin selections up
  in addition to the Extended Interrupts (EXTI).
  There needs to be one a free running timer to time length of the echo pulses.
  This uses up 14 GPI pins and maybe one internal timer.

The summary is:

     Function  Inputs  Outputs  Timers  Notes
     ===========================================================================================
     Encoders  4       0        2       1 timer per encoder (CH1 and CH2 only)
     HAL       0       1 (int)  1       TIM6 dedicated to HAL
     LED's:    0       1 (int)  1       Triggers DMA to write to LED's
     Lidar:    0       1        1       1 PWM
     Motors:   0       4        1 or 2  1 timer w/4 channels or 2 timers w/2 channels
     Sonars:   0       0        1       1 free running needed for interrupt driven echo signals
     Servos:   0       4        1 or 2  32-bit timers strongly preferred.

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
There are eight USART's:

* USART1/USART2/USART3/USART6:
  These support all USART functionality.

* UART4/UART5/UART7/UART7:
  These do everything except synchronous and smartcard mode.

The HR2 requirements for USART's are:

The USART requirements are:

* SBC (Single Board Computer):
  The single board computer (e.g. Raspberry Pi) needs to be able to talk to the microcontroller.

* Arduino Shield USART:
  Many Arduino shields need a dedicated serial connection.

* ST-Link debug USART:
  The ST-Link supports a serial port over USB.
  This is traditionally USART3.

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

## ZIO Connectors Mapping Table

In the
[STM32 Nucleo-144 Board User Manual](../../docs/stm32_nucleo_144_manual.pdf),
there is a table that corresponds to the Morpho connectors.
On pages 38 through 41 are Morpho pin-outs for the Nucleo144-STM32F767F.
These pages were extracted into small and easily searchable
[STM32 Nucleo-144 Board User Manual Pages 38-41](../../docs/stm32_nucleo_144_manual_pages38-41.pdf)
file by reading the file into `libreoffice` and writing out just pages 38 through 41.
Using this file it is possible to easily find connector/port/Arduino bindings.
A dense table is shown below where the Arduino signals are prefixed with an `@`:

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
        PA12: -------            PB12: CN7-7:   @D19      PC12: CN8-10:       PD12: CN10-21:
        PA13: -------            PB13: CN7-5:   @D18      PC13: -------:      PD13: CN10-19:
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

3. PE2 is connected to both CN9 pin 14 (SAI_A_MCLK) and CN10 pin 25 (QSPI_BK1_IO2).  Only one
   function must be used at one time.

4. PB13 is used as I2S_A_CK and connected to CN7 pin 5 by default, if JP7 is ON, it is also
   connected to Ethernet PHY as RMII_TXD1.
   In this case, only one function of the Ethernet or I2S_A must be used.


## Alternative Functions Table

The alternative functions table is used as part of the pin binding process.


The process of figuring this all out takes multiple steps.

1. Generate an `stm32f767_af.csv` file, using the `stm32cubeide` program that
   resides in the HR2 `bin/` directory.
   This program is executed and configured to use an STM32F767 processor.
   Next, once you have a graphical representation of the chip show,
   you click on the [Pinout] menu and select Export pinout with Alt Functions.
   This is saved to the file `stm32f767_af.csv`.

2. Next, the STM32F676 reference manual is read to figure out what timers are available
   are available and what are their various constraints.
   There are a lot of different.
   


Thus, the total number of counters is 14.  Note that typically, the input and the output
pin are one and the same, so it can only be used as either an input or an output but not both.
Also, only the first 6 counters listed above support encoder mode.
By the way, this is a lot of counters to pay around with!

Tentative Timer bindings are:
* LED's: TIM7: We just need an internal clock, that can trigger a 16-bit DMA transfer to a SPI.
* Sonars: TIM14 We just just need a course trigger pulse, that can be done using systick in RTOS.
  After that, there needs to be a free running timer that we can read on an interrupt for
  each edge transition of the echo return.
* Servos: TIM2: We need 32-bits.  There are 4 servos, so all 4 channels are used.
* Encoders: TIM3,TIM4,TIM8: Only of two of the 4 possible pins are consumed for each.
* Motors: 
* Lidar:
* HAL: TIM6

Servos (Need all 4 channels)

In timer order:
LPTIM1:
    LPTIM1_IN1: ('+PD12:AF3', '-PG12:AF3')
    LPTIM1_IN2: ('-PE1:AF3',)
TIM1:
    TIM1_CH1: ('-PA8:AF1',)
    TIM1_CH2: ('-PA9:AF1',)
    TIM1_CH3: ('-PA10:AF1',)
    TIM1_CH4: ('+PE14:AF1', '-PA11:AF1')
    TIM1_CH4: ('+PE14:AF1',)
TIM2: LMOTOR
    TIM2_CH2: ('-PA1:AF1', '+PB3:AF1')
    TIM2_CH3: ('-PA2:AF1', '+PB10:AF1')
    TIM2_CH4: ('+PB11:AF1',)
TIM3: LENCODER
    TIM3_CH1: ('+PC6:AF2', '+PB4:AF2')
    TIM3_CH2: ('+PC7:AF2', '+PB5:AF2')
    TIM3_CH3: ('+PB0:AF2', '+PC8:AF2')
    TIM3_CH4: ('+PB1:AF2', '+PC9:AF2')
TIM4: RENCODER
    TIM4_CH1: ('+PD12:AF2', '+PB6:AF2')
    TIM4_CH2: ('+PD13:AF2', '-PB7:AF2')
    TIM4_CH3: ()
    TIM4_CH4: ()
TIM5:
    TIM5_CH1: ('+PA0:AF2',)
    TIM5_CH2: ('-PA1:AF2',)
    TIM5_CH3: ('-PA2:AF2',)
    TIM5_CH4: ()
TIM6: HAL
TIM7: LEDS
TIM8: SERVOS
    TIM8_CH1: ('+PC6:AF3',)
    TIM8_CH2: ('+PC7:AF3',)
    TIM8_CH3: ('+PC8:AF3',)
    TIM8_CH4: ('+PC9:AF3',)
TIM9: RMOTOR
    TIM9_CH1: ('+PE5:AF3', '-PA2:AF3')
    TIM9_CH2: ('+PE6:AF3',)
TIM10:
    TIM10_CH1: ('-PF6:AF3',)
TIM11:
    TIM11_CH1: ('+PF7:AF3',)
TIM12:
    TIM12_CH1: ('-PB14:AF9',)
    TIM12_CH2: ('+PB15:AF9',)
TIM13:
    TIM13_CH1: ('+PF8:AF9',)
TIM14:
    TIM14_CH1: ('+PF9:AF9',)

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





## Rev. A

The revision A board actually contains six PCB's that are inter-related:

* *master*:
  The master board is goal board for the whole HR2 system.
  Unfortunately, it is too large to prototype using the Bantam Labs PCB milling machine.
  Thus, this particular board is not intended to be actually manufactured.
  Instead it is broken into 5 smaller boards that the Bantam Labs machine can prototype.
  When meshed together these 5 boards exactly match the outline of the master board.
  These 5 boards consist of a center board that is surrounded by 4 smaller boards
  located at the 4 corners.
  The board is oriented to match Romi `.dxf` file which has the front pointing downwards.

* *center*:
  This is the center portion of the *master* board.
  It contains the connections to the Raspberry Pi class single board computers,
  the Nucleo-144, the encoders, and the ST-link adapter.
  In addition, it contains the H-bridge and power supply electronics.
  Both the encoder board and ST-link adapter boards reside in a different directories.
  The remaining corner boards mostly contain sonars, LED's, and miscellaneous connectors.

* *ne*:
  This North East corner board and contains sonars and LED's.
  All component references are in the range 60-69 (e.g. R60, CN61, etc.)

* *nw*:
  This North West corner board and contains sonars and LED's.
  All component references are in the range 70-79 (e.g. R70, CN71, etc.)

* *se*:
  This South East corner board and contains sonars and LED's.
  All component references are in the range 80-89 (e.g. R80, CN81, etc.)

* *sw*:
  This South East corner board and contains sonars and LED's.
  All component references are in the range 90-99 (e.g. R90, CN91, etc.)

There are 6 KiCad `.pro` project files and associated 6 `.kicad_pcb` files that are one-to-one
with the board list immediately above.
The schematics are much more complicated.
KiCad supports hiearchical schematics, which are broken into multiple pages,
where each page in a separate file with a `.sch` suffix.
Through the magic of symbolic links the *master* board project shares the
schematic files of the other 5 boards.
It is an open question whether the KiCad design rule checker will work properly across
all boards.

The code in the HR2 `mechanical` directory is responsible for generating KiCad footprints
(e.g. `.kicad_mod` files) and positioning the various footprints, installing the board
outlines, mounting holes, and any associated cut-outs into the associated `.kicad_pcb` files.
This code ensures that everything is placed in the same locations on the master board and
the other 5 boards.


