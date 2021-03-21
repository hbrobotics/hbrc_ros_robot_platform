# ST-Link Adapter PCB

The ST-Link adapter PCB allows the ST-Link board to be detached from the Nucleo-144 board
and then reattached using this PCB.

## Introduction

The ST-Link Adapter PCB was primarily designed to "solve" an aesthetic problem.
The Nucleo-144 boards consist of a main board with the and additional smaller ST-Link board
that can be cut off.
The aesthetic problem is that when the Nucleo board is mounted on the robot
with the ST-Link board still attached,
the ST-Link board sticks out of the circular profile of the robot and looks both weird and lopsided.
By cutting the board off, in theory it can be still be used while hidden inside of robot.

That being said, there have been 3 revisions of the board to fix various issues.

* Revision A:
  This version had one of the connectors placed such that the connector "stuck out"
  and it could not be mounted flat.
* Revision B:
  This revision fixed the connector position,
  but it still did not work because it turns out the pins got swapped.
* Revision C:
  This revision fixes the pin swap issue, but has not been manufactured
  because the ST-Link adapter board interferes with the Raspberry Pi 4 micro-SD connector.
  This was discovered by literally destroying one Raspberry Pi 4
  by breaking both the micro-SD card *AND* the micro-SD card connector.
  Revision C still needs to modified to solve this problem.

Interest in the ST-Link adapter board is low right now,
since the true solution is to move the mating connector on the HR2 master board
to deal with the Raspberry Pi 4 micro-SD card interference problem.
For now, the ST-Link look unaesthetic and sticks out the side of the robot.

## Detaching Options

There was no way to make an intact Nucleo-144 fit on the HR2
without having corners stick over the edge of the Pololu Romi base.
The solution is to cut the ST-Link daughter board off of the Nucleo-144.
The ST-Link is still useful but now it needs to be connected to the whole system.

There are three options for using the detached ST-Link:

1. Patch Cables:
   This would work, but is prone to errors and disconnections due to vibration.
2. Direct Master Board Connections:
   This was considered, but there is insufficient room for the ST-Link and
   other devices such as sonars.
3. ST-Link Adapter PCB:
   This is a board that plugs into all of the semi randomly placed ST-Link connectors
   and rationalizes them to a single accessible connector

Option 3 is the only one that would make everything fit.
It is still a *very* tight fit.
(Too tight it turns out!)

There are all sorts of wires that flow back and forth between the Nucelo-144 and the ST-Link.
These wires are cut when the ST-Link is detached from the Nucleo-144.
These connections need to be reestablished using the adapter board.

Most of the signals are on the CN6 connector from Table 5 of the Nucelo-144 documentation:

1. VDD_TARGET: VDD from application (3.3V for Nucleo-144)
2. SWCLK: SWD clock.
3. GND: Ground.
4. SWDIO: SWD data input/output.
5. NRST: Reset.
6. SWO: Reserved.

What is missing from this is power.
In detached mode, the power will come from the USB cable.
Power is available from JP1 "left pin", but it is not needed by the HR2.

In addition, there is CN4 which is a 1x4 jumper header that is used to select between when
the Nucleo and ST-Link mode.
The adapter board is meant for Nucleo mode, so it just shorts the jumpers.

In addition, there is CN5 which has a TX/RX pair that is routed to UART3 on the Nucleo-144:

1. RX: Received serial data.
2. TX: Transmitted serial data.

A further complication is that there are no mounting holes for spacers on the ST-Link board.
This means that additional connectors are needed to provide mechanical stability when the
adapter board is connected.

CN4, CN5, and CN6 are adequate, but JP1 and JP2 are also used as overkill.

A 2x4 pin connector is defined for the adapter that is (Revision A/B):

1. 3.3V: 3.3V supplied from Nucleo-144.
2. SWCLK: SWD clock.
3. GND: Ground.
4. SWDIO: SWD data input/output.
5. NRST: Reset.
6. SWO: Reserved. (Presumably unused.)
7. RX: Received serial data.
8. TX: Transmitted serial data.

A 2x4 pin connector is defined for the adapter that is (Revision C):

1. SWCLK: SWD clock.
2. 3.3V: 3.3V supplied from Nucleo-144.
3. SWDIO: SWD data input/output.
4. GND: Ground.
5. SWO: Reserved. (Presumably unused.)
6. NRST: Reset.
7. TX: Transmitted serial data.
8. RX: Received serial data.

On the Nucleo-144 board, these pins are routed as follows:

1. 3.3V: 3.3V (+3V3) CN11 pin 16
2. SWCLK: TCK (PA14) CN11 pin 15
3. GND: Ground.
4. SWDIO: TMS (PA13) CN11 pin 13
5. NRST: Reset (NRST) CN11 pin14
6. SWO: Reserved (PB3) CN12 pin 31
7. RX: STLK_RX (PD8) CN12 pin 10
8. TX: STLK_TX (PD9) CN11:  pin 69

One last detail that is very important is that typically out of the box,
the ST-Link generates an 8MHz clock signal (called MCO) that is fed into the Nucleo-144.
In detached mode, this signal is not available on an ST-Link connector pin.
This means the Nucleo-144 must be reconfigured to use its on-board crystal instead of MCO.
This involves bridging SB8 and SB9 on the Nucleo-144 board.
This is obviously annoying, since these are solder bridges rather than jumpers.

Lastly, there is
(open source ST-Link software)[https://github.com/stlink-org/stlink/] available for
using the ST-link.
This code probably needs to be recompiled to run on the Arm instruction set for the Raspberry Pi
or other non-x86 processors.

<--
USB_IP Article:
    https://www.linux-magazine.com/Issues/2018/208/Tutorial-USB-IP
Systemd USBIP:
    https://unix.stackexchange.com/questions/528769/usbip-startup-with-systemd


Random notes:

Name      STLNK      MPU Port Name   Morpho      ZIO
GND       CN6:1
VCC       JP1
TCK                  PA14            CN11:15     -
TMS                  PA13            CN11:13     -
NRST                 NRST            CN11:14     CN8:5
SWO                  PB3             CN12:31     CN7:15 (D23)
STLK_RX              PD8 USART3_TX   CN12:10
STLK_TX              PD9 USART3_RX   CN11:69


Example of hooking up ST-Link to NUCLEO:
     https://os.mbed.com/questions/7974/F401RE-Cut-off-ST-LINK/

https://github.com/stlink-org/stlink/

				F410RE
St-link part(left Nucleo)	rest of Nucleo (right Nucleo)	signal
CN4 pin 2			CN7 pin 15			SWCLK
CN4 pin 3			CN7 pin 19			GND
CN4 pin 4			CN7 pin 13			SWDIO
CN4 pin 5			CN7 pin 14			NRST
JP1 left pin			CN7 pin 12			+3V_ST_LINK

Name      STLNK      MPU Port Name   Morpho      ZIO
GND       CN6:1
VCC       JP1
TCK                  PA14            CN11:15     -
TMS                  PA13            CN11:13     -
NRST                 NRST            CN11:14     CN8:5
SWO                  PB3             CN12:31     CN7:15 (D23)
STLK_RX              PD8 USART3_TX   CN12:10
STLK_TX              PD9 USART3_RX   CN11:69

# Tricky stuff
MCO                  PH0/PF0 OSC_IN  CN11:29                       Crystal: Open SB149 (trace cut)
                                                                   Close SB8/SB9 to use Nucleo xtl
PWR_ENn              USB 5V PWR      not used                      Use E5V.
E5V                                  CN11:6                        Feed 5V in here. or VIN?
VIN                  Voltage > 5     CN11:24                       Probably better to use E5V

-->

