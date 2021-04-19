# HR2 Encoder

The HR2 Encoder deals with the geometry constraints of a the Robot Computer
(i.e. Raspberry Pi 4) mounted on the Pololu Romi Base.
There is only one way to mount the Raspberry Pi processor on the Romi.
This means that the standard Pololu Romi encoders will not work.
Thus new encoders are needed.
These encoder boards are soldered to the motor and have 2 male 1x3 right angle connectors
that plug into 2 associated female 1x3 receptacles on the master board.
The connectors are to the side of each motor.

The Pololu encoder schematic uses the Diodes Incorporated AH1751 in the SOT23 package.
The AH1751 has A Bhys (Magnetic field hysteresis) 75 Gauss = 7.5mT
The CGS units for magnetic field strengths are Gauss and the MKS units are Tesla.
To convert from Gauss to Tesla, divide by 10000 (note: 10^4, not 10^3!)

The recommended replacement is the DI AH1751 is the DI AH3774.
Instead, for Rev. C and Rev D. the TI DRV5013 is used.
The AH1751 has A Bhy (Magnetic field hysteresis) 75 Gauss = 7.5mT
The TI DRV5013 has 4 versions of the chip that have different Bhys:
* FA: 2.6mT
* AD: 5.4mT
* AG: 12mT
* BC: 24mT
The closest match to the DI AH1751 is the TI DRV5013AD.

There are the following versions:

1. [Rev A.](#rev_a):
   The Rev A. boards had the 2 F1x3 offset by .1inch and had to refabricated.

2. [Rev B.](#rev_b):
   The Rev B. boards were designed using an incorrect hall effect sensor.
   A unipolar rather than a bipolar sensor was selected.
   In addition there was insufficient clearance for the motor hub to fit through the encoder PCB.

3. [Rev C.](#rev_c):
   The Rev C. boards are correct, but the right encoder mating connectors on the Rev. A
   master board are wired up backwards.

4. [Rev D.](#rev_d):
   The Rev D. board is an encoder board that works around the inverted wiring
   on the Rev. A master board, by inverting the wiring on the encoder.
   Thus, this board is only meant to be used as the Right encoder for the Rev. A  master board.

5. [Rev E.](#rev_e):
   A board spin on Rev.C that swaps a ground pin with quadrature signal to match a corresponding
   error on Master Board Rev. A.

6. [Rev F.](#rev_f):
   A board spin on Rev.D that swaps a ground pin with quadrature signal to match a corresponding
   error on Master Board Rev. A.


<!--
Pin locations are from left to right facing the top side are:
Left Side:
* 1: GND
* 2: QUADA
* 3: QUADB

Right Side:
* 4: VCC (3.3V-5V)
* 5: MOTOR-
* 6: MOTOR+

Motor tabs:
* 7: MOTOR- (Left tab)
* 8: MOTOR+ (Right tab)

LQUAD_A:
* PD12: CN10-21  LPTIM1_IN1 (CN12-43)

LQUAD_B: (Two pins shorted together)
* PA4: CN7-17  (CN11-32)
* PE1: CN10-24 LPTIM1_IN2 (CN11=61)

RQUAD_A:
* PC6: CN7-1 TIM8_CH1 (CN12-04)

RQUAD_B: 
* PC7: CN7-11 TIM8_CH2 (CN12-19)

GND:
* CN8-13
* CN9-23
* CN9-12
* CN7-8
* CN10-5
* CN10-17
* CN10-27
* CN10-22

+3.3V:
* CN8-7

+5V:
* CN8-9

Rev E: Left to right from front side:  Works for left motor.
1: QUADB
2: QUADA
3: GND
4: VCC
5: MOTOR+
6: MOTOR-

Rev F: Left to right from front side:   Works for right motor.
6: MOTOR-
5: MOTOR+
4: VCC
3: GND
2: QUADA
1: QUADB


-->
