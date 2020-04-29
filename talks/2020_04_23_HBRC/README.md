# Introduction to H2 Project

The HBRC ROS Robot (HR2) Project is an outgrowth of the relatively successful
FPGA class taught during the summer of 2019.  The ultimate goal is to have a
robot platform based on a commercially available hardware platform (the Pololu Romi)
and have the ability to do a wide range of basic robotic tasks (table top challenge)
through running a full ROS (Robot Operating System) stack.  This project is a work in
progress and is open to community involvement.

`https://github.com/hbrobotics/hbrc_ros_robot_platform/blob/master/talks/2020_04_23_HBRC/README.md`

## Information

* Speaker: Wayne Gramlich
* Date: April 29, 2020
* Time: 7:00PM PDT
* Location: Zoom Video Chat

## Zoom Issues

* Everybody mutes during the talks
* Please use chat for questions
* Zoom moderator will read questions after each slide
* Zoom moderator might activate microphone for follow on questions

## HBRC History

* HBRC Founded 1984 (Alza) Dick Prather@President
* Early 90's: HBRC moribund (no meeting place).
* Late 90's: HBRC revived (Cupertino Library) Chuck@President
* During the early 2000's, moved to Castro Middle School
* In 2002, HBRC started its Table Top Challenges  <========
* HBRC => CMU Wayne@President
* HBRC => Google Osman@President
* HBRC => Web (Coronavirus) Chris@President

## HBRC BOEBot Purchase

* Early 2000's:
* Club purchase 10+ Parallax BOE-Bots
* Basic Stamp!
* A bunch used in early Table Top Challenges
* ![BOE-Bot Image](boe-bot.png)

## Platforms Unchanged for 20 years

* 2 Motors + Castor + Encoders (maybe)
* Pathetic processor (Basic Stamp, Arduino, etc.)
* Many members wound up building their own.

## Problems with Custom Robot Platforms

* No commonality
* Most bases are pretty marginal
* Essentially no electronics/software sharing
* Reliability tends to really suffer

## 2019 FPGA Class

* Polulu Romi Robot (wheels/castor/motors/encoder/H-Bridge)
  * [Romi](https://www.youtube.com/watch?v=5Mrd0w9fmKo) Platform
  * URL: https://www.youtube.com/watch?v=5Mrd0w9fmKo
* Custom PCB for FPGA/Sonars/LED's/Edge sensors
* Group build!
* *EVERYTHING* ran on Raspberry Pi 3B+

## HR2: An Idea Is Born...

* [H]BRC [R]OS [R]obot => HRR => HR2
* Keep the Romi
* Support the Romi Arm
* Redo the custom PCB
  * Keep Raspberry Pi
  * Use a non-pathetic microcontroller
  * Continue to support FPGA's
* Add ROS1 & ROS2 support

## Some Decisions Get Made

* 100% Open Architecture
* MIT License
* Open Source Software development tools
* GitHub.Com (GitLab as backup)
* Use a daughter board architecture
  * Support more than Raspberry Pi SBC
  * Use [STM Nucleo-144 Dev. Boards](../../electrical/docs/stm32_nucleo_144_manual.pdf)
  * FPGA's plug on top of Nucleo
  * More expansion (Grove/MikroBus/etc.)

## Polo Romi

* Costs are Reasonable
  * 1 Platform + 2 Motors (~$30) + 1 Extra Castor ($3)
  * 2 Encoders ($12)
  * Half Expansion Plate (~5)
  * Arm ($80) (Expansion Pate, 3 Servos, Gripper, Arm)
* Small motors => payload constraints
* 6.5in Diameter => not a lot of room
* Lot's of mounting holes => good/bad

## Mechanical Design Process

* `scad.py` library for `.scad` generation (3.5K lines)
* QCad to Extract hole locations (qcad)
* `scad_models.py` generates `.scad` file (14.5K lines)
* OpenScad to view models, generate images/dxf files

## Mechanical Stackup:

* Qcad (show qcad)
* Base (show hr2_base)
* +Pi (show hr2_pi)
* +PCB (show hr2_master)
* +Motors/Encoders (hr2_wheel)
* +Nucleo (show hr2_nucleo)
* +Arm (show hr2_arm)

## Micro-Controller Selection

* Availability really matters!
  * ARM Cortex M is the obvious choice
* Vendor (Cypress/Infineon/Maxim/MicroChip/NXP/ST/TI/Toshiba?)
  * ST Microelectronics (huge selection)
* MPU placement On/Off main PCB?
  * Off-board: Use Dev. Board
* Dev-board Family: Nucleo-32/64/144? 144
  * Pro: Nucleo-144 has lots of pins
  * Pro: Arduino Shield capability
  * Con: Too big => Cut off ST-link and Remount

## PCB Development

* Schematic Capture:
  * STM32Cube (show cube)
  * Stm32cube => KiCube32 => KiPart => KiDocGen
  * [KiPart](https://kipart.readthedocs.io/en/latest/)
  * KiCAD (show kicad)
* PCB Fab:
  * Surface Mount
  * Rev. A: [Bantam Tools PCB Mill](https://www.bantamtools.com/)
    * Rectangular board 4in x 5in (~$1.25/board)
  * Rev. B: Shipped out to PCBA house
    * 6.5in mostly round board.

## Expansion Peripherals

* FPGA Boards (TBD)
* [Arduino Shields](http://shieldlist.org/)
* [Seeedstudio Grove](https://www.seeedstudio.com/grove.html)
* [mickroBUS](https://www.mikroe.com/mikrobus)
* Serial Bus (TBD)
* [Low Cost Lidar](https://www.robotshop.com/en/ydlidar-x2-360-laser-scanner.html)

## Software Issues:

* Ubuntu 18.04 for now (Virtual Box for other OS's)
  * Ubuntu 20.04 later
* Really just beginning:
  * Install script (`hr2_install`) largely done
  * Use `stm32cubeide` (Eclipse + STM32Cube stuff)
* ROS1/ROS2 not started
* No drivers written yet
* Simulators (Gazebo, Webots, ...)

## Possible Courses

* Really not started yet.
* Table Top Challenge
* FPGA Class (2nd generation)
* KiCAD
* ROS
* etc.

## Schedule

* Rev. A PCB(s) done by end of June
* Rev. B PCB done by end of August
* Working robot by Nov. HBRC Meeting (19Nov)
* Group buy Black Friday (27Nov) -- Cyber Monday (30Nov)

## Conclusions

* Definitely a Work in progress
* ~6 months to "completion"
* Open to amateur robotics community
* `https://github.com/hbrobotics/hbrc_ros_robot_platform/blob/master/talks/2020_04_23_HBRC/README.md`
* Questions?
