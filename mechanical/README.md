# HBRC ROS Robot Mechanical Issues

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

  where NAME is one of:

  * `romi_base`: This is the basic polygon that represents the Romi
    Chasis.  It is 2D only.

  * `expansion`: This is the basic polygon that represent the Romi
    Expansion Plate to which the arm is attached.

  After you run `openscad` use your mouse to select `[Design]'=>'[Preveiw]',
  to show the resulting model.

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

