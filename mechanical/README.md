# HBRC ROS Robot Mechanical Issues

The "nominal" base for the HR<Sup>2</Sup> is the
[Pololu Romi Chasis](https://www.pololu.com/category/202/romi-chassis-and-accessories).
However, other bases can be considered.

## Mechanical Requirements:

The section below is a wish list of "requirements" for this HR2.
Not all of these requirements are expected to be met.

### Base Requirements

The base requirements are:

* Use a COTS (Commercial Off The Shelf) platform.
* Differential Drive with relatively large wheel diameter.
  * A maximum forward speed of 30cm/sec.  10 cm/sec. is acceptable.
* Over 1000 ticks per revolution odometry.
* Front and rear ball castors with low rolling friction.
  * One of the ball castors must be "floating" to deal with uneven floors.
* Runs off of rechargeable batteries.  A USB battery pack would be ideal.
  * The mass of the batteries must be along the robot center-line to
    prevent the robot from veering to one side.
  * The mass of the batteries must be towards the rear non-floating ball
    castor.
  * It would be nice if the battery pack could be easily swapped out.

### Sensor Requirements

The sensor requirements are:

* Bottom reflectance sensors for edge detection and/or line detection.
  Pololu QTR/QTRX/QTRLX sensors are acceptable.
* Time of flight edge sensors.  SparcFun SEN-12787 will work.
* Sonars: SR04's sensors from China are inexpensive (and not of the
  highest quality).  5 Sonars in front and back with 22.5 degree angular spacing.
  The sonars can be alternated to mounted on top and on bottom of the
  same PCB to save space.  There is an mechanical interference issue with
  the forward
* Bump Sensor: An inexpensive front and rear bump sensor is desired.
  It can be extremely simple.
* 3D Camera: The Microsoft Azure Kinect is the nominal one.  It needs
  to be replaceable as better cameras come along.  USB3 interface is
  acceptable.  This is heavy sensor, so managing center of mass is
  very important.
* E-Stop Button: It is nice to have an E-stop button that is readily
  available, when the robot code goes bad.  The E-stop would disable
  the motor drivers.
* Lidar: An optional inexpensive 360 degree Lidar is very desirable.
  ComputerShop sells a bunch -- YLIDAR X4 ($99), YLIDAR G2 ($159),
  YLIDAR X2 ($69), RPLidar A1Mi ($99).
* Cameras:
  * It would be nice to have an optional forward facing camera
    for object detection, etc.  The 3D camera might do this as well.
  * It would be nice to have an optional upward facing camera
    for fiducial navigation.
* Microphone: It would be nice to be able to talk to the robot.
  * A directional microphone would be cool so that the robot can
    turn towards a speaker.

### Actuator Requirements:
	
The actuator requirements are:

* Gripper Arm: An optional arm with gripper is desirable:
  * The Pololu Romi Arm is acceptable.  It is not required to mount it
    on the Pololu expansion plate; it can be mounted forward.  Managing
    center of mass is important.
  * Non-Gripper Arms: It would be fun if there is a "arm" on each side so
    that the robot can wave its "arms".  There are no grippers on these
    side arms.

### Indicator Requirements:

The indicator requirements are:

* LED's:
  * Front LEDS:
    * For debugging and fun there needs to be a line of LED's
      in the front.  Being able to do a Knight Rider LED sweep would be fun.
  * Rear LEDS:
    * It would be nice to have both Rear LED's that can glow read for
      braking and white for reverse (i.e. back up lights.)
* LCD: An optional LCD would be nice.
  * Ideally, this can be connected to the SBC as an output device.
  * If placed immediately below the 3D camera it can be programmed
    to be a mouth that can smile,  frown, smirk, etc.
  * If a standard size is selected, it can play video clips. (Woo Hoo!)
* Speaker: It would be nice if the robot could play sounds.  Stereo
    is not required.
    
### Miscellaneous Requirements:	

The miscellaneous requirements are:

* Miscellaneous:
  * Docking Station:
    * An optional docking station would be nice.  It can be *VERY* simple.
  * USB devices:
    * Keyboard/Mouse: For debugging, a keyboard and mouse is desirable.
      * Wireless keyboard and mouse is OK.
    * Game Controller:
      * Wireless remote controller is used to drive the robot around.
  * Carrying Handle: It would be very nice to be able to pick up the
    robot with a handle.
  * Carrying Case: Strictly optional, but being able to cart the robot
    around in some sort of clear case would be nice.
  * Mount points:
    * It would be nice to provide some mount points for other expansions
      modules like Seeedstudio Grove.  This is basically a 10mm x 10mm
      grid of holes.  These can be both internal and/or external.  There
      needs to be cable access holes for externally mounted modules.

### Robot Skin requirements:

The robot skin requirements are:

* A skin is optional, but very desirable.
* Humans like anthropomorphic robots because they are more "fun".
* Having a clear skin so people can see inside is very desirable.
  * Being able to print a paper inside to put inside the clear skin
    to "cloth" the robot would be cool.  This is strictly optional.

### Non-requirements:

The explicit non-requirements are:

* This platform is not expected to be operated over uneven floors.
  Going over door strips is *NOT* required.


## Installation and Use

The instructions below for Ubuntu based distributions.  For all other operating
systems and Linux distributions you are on your own.

* Download clone the repository:

            # cd .../somwhere  # Something like `~/Downloads` should work
            git clone https://github.com/hbrobotics/hbrc_ros_robot_platform.git

* Install Python Virtual Envoronments:

  * Read about
    [Python Virtual Envionrments](https://realpython.com/python-virtual-environments-a-primer/).
    The URL is `https://realpython.com/python-virtual-environments-a-primer/`.  The
    important stuff is at the end of tutoral document.  Make sure you edit your
    `~/.bashrc` file as directed in the tutoral.

  * Setup a Python Virtual Environment:

            mkvirtualenv -a .../somewhere -p python3.6 hr2
            workon hr2

* Install the required Python development tools:

            cd romi_model
            pip install -r develop.rec


* Install `openscad`, `convert` (i.e. ImageMagik), and `qcad`:

            sudo apt install openscad imagemagik qcad

* Build the `romi_base.scad` files:

            make
            scad_models

* Run `openscad` and run "[Design]=>[Preview]":

  There is one `scad_models.scad` file from which all of the variouls
  models are derived.  To view each model:

            openscad scad_models.scad -D 'name="NAME"'

  where NAME is one of:  <!-- NAME list starts here. -->

  * `battery_base_polygon`:
    Romi Battery Base Polygon

  * `hr2_robot`:
    Entire HR2 Robot Model

  * `other_pi_pcb`:
    Other Pi PCB

  * `otherpi_model`:
    Other Pi

  * `raspi3b_model`:
    Raspberry Pi 3B+

  * `raspi3b_pcb`:
    Raspberry Pi3B+ PCB Polygon

  * `romi_base`:
    Romi Base Polygon

  * `romi_base_polygon`:
    Romi Base Polygon

  * `romi_expansion`:
    Romi Expansion Chasis Polygon

  * `romi_motor`:
    Romi Motor

  After you run `openscad`, use your mouse to select `[Design]`=>`[Preveiw]` and `openscad`
  will show the resulting model.

## Documentation:

### Remote Documentation

  * [Pololu Romi Chasis User's Guide](https://www.pololu.com/docs/0J68/all):
    The Pololu Romi is mostly documented by the Pololu web site.  Alas, there
    is no `.pdf` version of the file, so if Pololu ever decides to take the
    User's Guide down, it will be *GONE*.

### PNG Files
  
  The `mechanical/png` directory is where various `.png` image files are
  stored.  Right now there are images for the `romi_base` and `expansion`
  models:

  * [mechanical/png/expansion.png](png/expansion.png):
    The OpenSCAD modle of the Romi Expansion Chasis:
    <Br>
    ![](png/thumb_expansion.png)

  * [mechanical/png/romi_base.png](png/romi_base.png):
    The OpenSCAD model of for the Romi Chassi Base:
    <Br>
    ![](png/thumb_romi_base.png)

### DXF Files

  There are two generated `.dxf` files available:

  * [mechanical/dxf/expansion.dxf](dxf/expansion.dxf):
    The generated `.dxf` file from the OpenSCAD model
    for the Romi Expansion Chassis.

  * [mechanical/dxf/romi_base.dxf](dxf/romi_base.dxf):
    The generated `.dxf` file from the OpenSCAD model
    for the Rom Chassis Base.

  In addition, there is are two `.dxf` files supplied by Pololu
  that are archived in the `mechanical/dxf` directory.  These
  are copyrighted by Pololu and if Pololu objects to these being
  cached in this repository, they will be removed immediately
  upon a request from Pololu:

  * [mechanical/dxf/romi_chassis.dxf](dxf/romi_chassis.dxf):
    The `.dxf` that can be used to extract the various hole,
    slot and rectangles on the Romi chassis.

  * [mechanical/dxf/romi-chassis-expansion-plate.dxf](dxf/romi-chassis-expansion-plate.dxf):
    The `.dxf` that can be used to extract the various hole,
    slot and rectangles on the Romi expansition plate.
    
### PDF Files

  There are two cached `.pdf` files from Pololu.  Again these
  are copyrighted by Pololua nd if Pololu objects to these being
  cached in this repository, they will be removed immediately
  upon a request from Pololu:

  * [mechanical/pdf/romi-chasis-expansion-plate-dimensions.pdf](pdf/romi-chasis-expansion-plate-dimensions.pdf):
    The `.pdf` that contains many (but by no means all) of then
    dimensions of the Romi Chasis Expansion Plate.

  * [mechanical/pdf/romi-chasis-expansion-plate-hole_alignment.pdf](pdf/romi-chasis-expansion-plate-hole_alignment.pdf):
    The `.pdf` that contains information about which holes line up
    between the expansion plate the the main chasis plate.

## Models:

The models are store in some Python files, which upon execution generate
the associated OpenSCAD files.  There are two directories:

### `scad_models`

The `scad_models` directory currently contains:

* `scad.py`: This is a library of Python code that is used to generate
  an OpenSCAD `.scad` file.

* `scad_models.py`: This is the Python code that builds all of the
  OpensCAD models using the `scad.py` liberary.

* `__init__.py`: This is the file that is required by Pythong to indicated
  that the `scad_models` directory contains a Python package.

The program `pydoc NAME.py` will print out the Python doc strings for some
documentation.

### `tests`

The `tests` directory contains the Python Unit tests:

* `test_scad.py`: The unit tests for the `scad.py` library.

* `test_scad_models.py`: A unit tests for the `scad_models.py` code.

## Miscellaneious:

There a a bunch of miscellaneous files in the `mechanical` directory:

* `README.md`: This documetation file.

* `setup.py`: This file is required by Python to construct the `scad_models`
  package.

* `develop.rec`: This is a list of Python packages that are used by this project.

* `Makefile`: There is a `Makefile` that is used to build everthing.
  It has the following three targets:

* `clean`: Removes all derived files to force a clean build.

* `all`: Build everything.  This involves:

  * Running the Python files through `mypy`, `flake8`, and `pydocstyle`.
  * Installing the `scad_modules` package in the Python virtual environment.
  * Runs the `scad_models` program from the Python virtual environment to
    generate `scad_models.scad`.
  * Generates various `.dxf` and `.png` files.

* `test`: Runs the Python unit test suites and outputs any code lines
   that is not covered by the test suites.

* [romi_base.csv](romi_base.csv): This is a `.csv` (Comma Separated Value) file that
  can be read in as a spreadsheet.  It identifies pretty much all of
  the holes, slot, and rectangles on the Romi Chasis with the origin
  set to the exact middle of the chassis.

* [romi_base.html](romi_base.html): This contains the same content as the `romi_base.csv`
  file, but in HTML table format.  It is easier to read in a web browser.

