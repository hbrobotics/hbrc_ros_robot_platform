# HBRC ROS Robot Platform

The HBRC ROS Robot platform (ie. HR<Sup>2</Sup> or just plain HR2) is a
pedagogical robotic platform developed for teaching various robotics skills.

There is a Google Groups mailing list to discuss this project at
[HbrcRosRobotPlatform@GoogleGroups.Com](mailto:HbrcRosRobotPlatform@GoogleGroups.Com).
To join the group, visit the
[HBRC ROS Robot Platform web page](https://groups.google.com/d/forum/hbrcrosrobotplatform)
(i.e. `https://groups.google.com/d/forum/hbrcrosrobotplatform`) and request to join.
If that does not work, squirt a quick message to the group manager
(`Wayne` ATSIGN `Gramlich` PERIOD `Net`) requesting an invitation to join
the group.

## Possible Classes

The list of possible classes for HR2 platform are:

* Low level robot peripheral drivers.
  * C/C++
  * Micropython
* ROS driver development
  * C/C++
  * Python
* More generic ROS Programming
* Image processing with OpenCV
* AI Frameworks
* Arm manipulation
* PCB design
* MCAD design
  * OpenSCAD
  * Fusion 360
  * FreeCAD
* FPGA class
* Soldering Skills
  * Basic Through Hole
  * Surface Mount
* Rapid PCB prototyping w/Bantum Labs
* etc.

## Goals/Requirements

Initial Thoughts:

* Start with the Pololu Romi platform used by FPGA class.
* Use Patrick's PCB breakout board as a starting point.
* Consider ditching the Pololu motor board.
* Work hard to see if the Pololu arm can be bolted on.
* Try to use USB battery pack instead of AA batteries.
* Provide support RasPi 3B+ and RasP4:
  * 40-pin connector.
  * I2C EEPROM for "hat" identification.
  * Real Time Clock needed for ROS.
  * Various RasPi I2C, SPI, GPO, Serial lines made avaiable.
* Some sort of MPU on board:
  * Probably STM32 but could be ESP32
  * Roomy enough to run MicroPython
  * Must support C/C++ debugging.
  * Ethernet Phy to talk to boards other than RasPi
    (e.g. Xavier, other not RasPi 40-pin connector compatibles.)
* FPGA support:
  * Soldered in or daughter board?
* Arduino support:
  * Soldered in AVR? or daughter board?
* Expansion Capabilities:
  * Arduino Shields
  * MikroBus
  * Demand Peripherals Connector(s):
  * Wayne's Bus?
  * Grove connectors
* Sensors:
  * Motor encoders (comes with Pololu motors)
  * Various edge sensors for table top challenge, maze follower, etc.
  * Edge sensors (TOF, IR)
  * Sonars, the are cheap.
  * Servos (arm, camera tilt)
  * Inexpensive Lidar
    * YLidar X2 (~$70)
    * RPLdar A1M8 (~$100)
  * E-Stop
* Camera
  * RasPi Camera
  * Tilt platform to move between forward vs. upward direction.
    * https://www.servocity.com/ddt500
* Misc:
  * What issues are missing?

## Mechanical

The mechanical issues are worked out in greater detail in the
[mechancial directory](mechanical/README.md).

The mechanical tasks are:

* Model the Romi platform in OpenSCAD to ensure that the PCB outline
  and mounting holes are correct.
* Model the Romi arm in OpenSCAD to make sure that the Arm does not
  interfere with anything.
* Model both a RasPi 3B+ and RasPi4 for attaching to PCB.

## Electrical

The electrical issues are worked out in greater detail in the
[electrical directory](electrical/README.md).

Some comments on the electrical:

* The design is to be done using KiCad, since KiCad is 100% free.
* It would be nice to be a two layer PCB.
* It would be nice to be able to take power from a USB Power pack
  or the on board AA batteries.

## Software

The software issues are worked out in greater detail in the
[software directory](software/README.md).

Terminology:

* Single Board Computer (SBC):
  The nominal SBC for the platform is either the RasPi 3B+
  or the RasPi 4.  Other boards that are compatible with the RasPi
  pin-out *can* be supported, but somebody will have to step up to
  the task of doing the actual support.  The SBC resides on the robot.
* Micro-Processor Unit (MPU):
  This is the micro-controller that is on the main board.
  The nominal processor is ST32Fxxx, where xxx is to be decided.
  The MPU resides on the robot.
* Robot Processors: The robot processors are the SBC and the MPU.
* Development Processor: The development processor is not physically
  attached to the robot.  Instead, it communicates with the robot SBC
  via WiFi.

The software goals are:

* The platform runs ROS on the SBC.
  * Both ROS 1 and ROS 2 are goals with ROS 1 eventually being
    deprecated.
* We need to be able download firmware into the MPU from the SBC.
  This needs to be hands off.
  * Custom C/C++ drivers.
  * MicroPython.
  * Micro ROS.
* The development processor needs to be able to debug code
  running on the robot:
  * Debugging ROS nodes running on the SBC should be relatively easy.
  * Debugging C/C++ code running on the MPU is going to need JTAG
    support.  This can be done with a JTAG chip like the FTDI FT2232.
    This *may* be an "add-on".  OpenOCD can talk to this JTAG chip
    and `gdb` can talk to openocd.
  * Debugging MicroPython is tough.  It currently supports breakpoints
    but does not support data/stack inspection (yet!)
  * As a total stretch, it would be nice to support ARM ETM CoreSight.
    This requires a seriously expensive brick that weighs a ton.
    It is probably a fantasy.
* We need to support FPGA development:
  * We need to be able to download the FPGA chip from the SBC.
  * The FPGA compliation stack needs to run on the SBC and the development
    processor.

