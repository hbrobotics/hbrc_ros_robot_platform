# HBRC ROS Robot

The HBRC ROS () Robot platform (ie. HR<sup>2</Sub> or just plain HR2)
is a pedagogical robotic platform developed for teach various
robotics skills.  For example:

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
  * openscad
  * fusion360
  * freecad
* FPGA class
* Soldering Skills
  * Basic Through Hole
  * Surface Mount
* PCB prototyping w/Bantum Labs

## Goals/Requirements

Random Thoughts:

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
