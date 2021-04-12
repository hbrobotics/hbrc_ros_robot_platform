# HR2 Encoder

The HR2 Encoder deals with the geometry constraints of a the Robot Computer
(i.e. Raspberry Pi 4) mounted on the Pololu Romi Base.
There is only one way to mount the Raspberry Pi processor on the Romi.
This means that the standard Pololu Romi encoders will not work.
Thus new encoders are needed.
These encoder boards are soldered to the motor and have 2 male 1x3 right angle connectors
that plug into 2 associated female 1x3 receptacles on the master board.
The connectors are to the side of each motor.

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

