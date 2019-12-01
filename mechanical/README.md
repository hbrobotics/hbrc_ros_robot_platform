# HBRC ROS Robot Mechanical Issues

## Installation and Use

The instructions below for Ubuntu based distributions.  For all other operating
systems and Linux distributions you are on your own.

* Install Python Vertual Envoronments:
  * See `https://realpython.com/python-virtual-environments-a-primer/`

* Down clone the repository:

       # cd .../somwhere
       git clone https://github.com/hbrobotics/hbrc_ros_robot_platform.git

* Install Python Virtual Envoronments:
  * See `https://realpython.com/python-virtual-environments-a-primer/`

* Setup a Python Virtual Environment:

        mkvirtualenv -a .../somewhere -p python3.6 hr2
	workon python hr2

* Install the required development tools:

        cd romi_model
        pip install -r develop.rec


* Build the `romi.scad` files:

        make
        romi_model

* Install `openscad`:

        sudo apt install openscad


* Run `openscad`:

        openscad romi.scad


* That is as far as we have gotten so far.

## Documentation:

* [Pololu Romi Chasis User's Guide](https://www.pololu.com/docs/0J68/all):

* [`romi-chassis.dxf`](romi-chassis.dxf):
  This is the `.dxf` file that can be used to pick out hole and slot locations.

Regards,

-Wayne





