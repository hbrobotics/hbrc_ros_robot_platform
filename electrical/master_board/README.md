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


