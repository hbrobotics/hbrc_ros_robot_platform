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

# HBRC ROS Robot Platform

The HBRC ROS Robot platform (ie. HR<Sup>2</Sup> or just plain HR2) is a
pedagogical robotic platform for teaching various robotics skills.

![HR2 Robot](mechanical/png/hr2_nucleo_assembly.png)

This top level document is broken into the following sections below:

* [Google Group](#google-group):
  The Google Group mailing list for this project.

* [Goals/Requirements](#goalsrequirements):
  The goals and requirements for the project.

* [Mechanical](#mechanical):
  The mechanical engineering aspects of the project.

* [Electrical](#electrical):
  The electrical engineering aspects of the project.

* [Software](#software):
  The software engineering aspects of the project.

* [Education](#education):
  The education components of the project.

* [Download, Installation and Workflow](#download-installation-and-workflow):
  The download, installation, and workflow portions of the project.

## Google Group

There is a Google Groups mailing list to discuss this project at
[HbrcRosRobotPlatform@GoogleGroups.Com](mailto:HbrcRosRobotPlatform@GoogleGroups.Com).
To join the group, visit the
[HBRC ROS Robot Platform web page](https://groups.google.com/d/forum/hbrcrosrobotplatform)
(i.e. `https://groups.google.com/d/forum/hbrcrosrobotplatform`) and request to join.
If that does not work, squirt a quick message to the group manager
(`Wayne` AT_SIGN `Gramlich` PERIOD `Net`) requesting an invitation to join the group.

Not all traffic on the group is going to be interesting to everybody else.
To help out with filtering please put one of the following prefixes at the
front of the each subject line:

* `ALL:` A message of interest to everybody.
* `INST:` A message concerning software installation.
* `ME:` A message to people interested in the HR2 Mechanical Engineering 
  (i.e. solid modeling.)
* `EE:` A message to people interested in the HR2 Electrical Engineering
  (i.e. PCB design, manufacture, bring-up, testing, etc.)
* `SE:` A message to people interested in the HR2 Software Engineering
  (i.e. programs, firmware, etc.)
* `ED:` A message to people interested in the HR2 course materials.
* `DL:` A message concerning downloading and installation.
* `MSC:` A miscellaneous message that does not really fit any of the above
  protocols.

That should be enough group mailing list structure for starters.

## Goals/Requirements

The list below summarizes the some of the design choices that have been
made so far.

* Start with the Pololu Romi platform used by FPGA class. [Decided]
* Use Patrick's PCB breakout board as a starting point. [Done]
* Consider ditching the Pololu motor board. [Done]
* Work hard to see if the Pololu arm can be bolted on. [Very Likely]
* Try to use USB battery pack instead of AA batteries. [USB Pack yes; AA's no]
* Provide support RasPi 3B+ and RasP4: [yes]
  * 40-pin connector.
  * I2C EEPROM for "hat" identification.
  * Real Time Clock needed for ROS.
  * Various RasPi I2C, SPI, GPO, Serial lines made available.
    *  [Serial: yes, I2C: yes, SPI: probably not, GPIO: OS shutdown
* Some sort of MPU on board:
  * Off the shelf microcontroller board [STM Nucleo-144 Dev. Board format selected]
  * Roomy enough to run MicroPython/MicroROS [STM NUCLEO-F767ZI nominal]
    * ~$25, 216MHz, 2MB Flash, 512K RAM, timers, UARTs, I2C, SPI, Ethernet, etc.
  * Must support C/C++ debugging [ST-Link].
  * Ethernet Phy to talk to boards other than RasPi [Yes]
* FPGA support [custom daughter board required.]
* Arduino support [AVR Dropped. AVR's are expensive/under powered]
* Expansion Capabilities:
  * Arduino Shields. [One can be plugged onto Nucleo.]
  * MikroBus [Iffy, space is tight]
  * Demand Peripherals Connector(s): [On FPGA daughter boards only]
  * Wayne's Bus? [Iffy, space is tight; ouch this hurts!]
  * Grove connectors [Very Likely]
  * Pololu power supply sockets [Iffy, space is tight.]
* Sensors:
  * Motor encoders. [Yes, 1440 step/rev.]
  * Various edge sensors for table top challenge, maze follower, etc.
  * Edge sensors (TOF, IR)
  * Sonars, the are cheap [5 in front, 4 in back.]
  * Servos (arm, camera tilt) [Minimum 4 required]
  * Inexpensive Lidar [Only if the platform is not overloaded]
    * YLidar X2 (~$70)
    * RPLdar A1M8 (~$100)
  * E-Stop [Probably not]
* Camera [RasPi Camera]
* Misc:
  * What issues are missing?

### Mechanical

The mechanical issues are worked out in greater detail in the
[mechancial directory](mechanical/README.md).

The mechanical tasks are:

* Model the Romi platform in OpenSCAD to ensure that the PCB outline
  and mounting holes are correct.
* Model the Romi arm in OpenSCAD to make sure that the Arm does not
  interfere with anything.
* Model both a RasPi 3B+ and RasPi4 for attaching to PCB.
* Model the basic PCB outlines, connector placements, etc.

A short summary of the mechanical design decisions to date are:

* Use the Pololu Romi Platform.
* Use an alternate layout of the motor encoder PCB to free up space to
  support some SBC (Single Board Computers) that are larger than the
  Raspberry Pi footprint.
* Design master board to use an STM Nucleo-144 development board for the
  processor.  All H-bridges, power management, to be put on one board.
* FPGA boards to be done with a a daughter board that plugs into Nucleo-144 board.
 
## Electrical

The electrical issues are worked out in greater detail in the
[electrical directory](electrical/README.md).

Some brief comments on the electrical aspects are:

* The design is to be done using KiCad, since KiCad is 100% free.
* It would be nice to be able to use a simple two layer PCB.
* It would be nice to be able to take power from a USB Power pack.
* It would be nice to support daughter boards for FPGA's.

The overall strategy is to prototype everything using the
[Bantum Labs PCB Milling Machine](https://www.bantamtools.com/)
over at [OLogic](http://www.ologicinc.com/).  This machine is
constrained to only prototyping boards that are 4 inches by 5 inches.
Once the electronics are debugged and working, the PCB's will be
sent out to an appropriate PCB Manufacturing house.

### Software

This section is pretty brief for the moment...

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

## Education

This section is more of a place holder for now...

A partial list of possible educational classes for HR2 platform are:

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

## Download, Installation, and Workflow

Software downloading and installation is done using the following basic steps.

1. [Optional Ubuntu Virtual Machine Installation](#optional-ubuntu-virtual-machine-installation]):
   For now, this project is developed exclusively using a Ubuntu 18.04LTS (Long
   Term Support) Linux distribution.  If you are not running Ubuntu 18.04LTS,
   you need to install it so that it runs using a virtual machine emulation software.
   Otherwise, this step can be skipped.

2. [Download and Install Software](#download-and-install-software):
   The HR2 project uses a diverse set of software.  This software is installed
   using some installation scripts followed by some tried and true technology
   called recursive Make.

3. [Workflow](#workflow):
   Modifications to the documention and design files is done using the
   [GitHub pull request workflow](https://guides.github.com/introduction/flow/).

### Optional Ubuntu Virtual Machine Installation

If you are running on either Windows or MacOS or a non-Ubuntu 18.04LTS Linux distribution,
you need to install Ubuntu 18.04LTS using virtual machine emulation software.  If you are
already running Ubuntu18.04LTS, you can skip this section right now.

There are many
(virtualization)[https://en.wikipedia.org/wiki/Comparison_of_platform_virtualization_software]
software products out there.  If you have a favorite, please feel free to use your favorite
virtualization product.  If you have no favorite, (VirtualBox)[https://www.virtualbox.org/]
is freely available and works.  The remaining instructions are in this section are for
VirtualBox.

Let's start off with some terminology:

* Host Machine:
  The host machine is the actual hardware and associated operating system that VirtualBox
  is running on.

* Guest Machine:
  The guest machine is the virtual hardware and associated operating system that is running
  under the VirtualBox software.  In this case, the guest machine is running Ubuntu18.04LTS.

The basic steps involved are:

1. Download VirtualBox:
   Visit the [VirtualBox Downloads] web page and follow the instructions appropriate
   for your host machine.

2. Download Ubuntu18.04LTS:
   [Download Ubuntu18.04 LTS](https://ubuntu.com/download/desktop) to your host machine.
   This shows up as a file with an `.iso` suffix.
   
3. Configure a VirtualBox Virtual Machine.
   Start VirtualBox software and a Graphical User Interface (GUI) should show up on your
   screen.

   Click on the [New...] button to bring up create pop-up window.

   A. Name and operating system:
      * Name: Pick a name, (suggestion: "Ubuntu18.04LTS").
      * Machine Folder: (Use the default.)
      * Type: Pick [Linux]
      * Version: Ubuntu (64-bit)
      * Click on [Next>].

   B. Memory size:
      * At least 4096 MB is recommended.  Do not use 1024 MB.
      * Click on [Next>]

   C. Hard disk:
      * Select "Create virtual hard disk now".
      * Click on [Create]

   D. Hard disk file type:
      * Select "VDI (VirtualBox Disk Image0"
      *Click on [Next>]

   E. Storage on physical hard disk:
      * Select "Dynamically allocated"
      *Click on [Next>]

   F. File Location:
      * Use the default file folder.
      * Increase the file size to at least 20 GB. 30 GB or 40 GB is preferred.
      * Click on [Create] to create the virtual machine and dismiss the pop-up window.

   G. Configure network:
      * Click on the [Settings...] Button to bring up [Settings] pop-up window:
      * Change "Attached to:" from "NAT" to "Bridged Adapter"
      * Click on [OK] to dismiss the pop-up window.

   The virtual machine is configured and ready to run.

4. Install Ubuntu18.04LTS.

   A. Start the virtual machine by either double clicking on the machine name or clicking
      on the [Start] button.  Please note that the guest machine and host machine can
      fight over who has the keyboard and mouse.  There are two control keys on your
      keyboard, one to the left of the space bar and the other to the right of the space bar.
      The right control key is used break up the fighting over mouse/keyboard input destination.
      If you are going "what happened to my mouse cursor are where are my keystrokes going"
      use the right control key to change things.

   B. After starting the virtual machine, VirtualBox will bring up a pop-up window
      asking you to select a virtual CD Rom image to to insert into the virtual optical
      disk drive.  Please use the file choose to select the Ubuntu 18.04LTS `.iso` file
      you previously downloaded.

   C. Click on [Install Ubuntu].

   D. Select your keyboard.

   E. Select: "Normal installation and download updates".

   F. Select: "Erase disk and install Ubuntu".
      * (Since this is virtual machine you can safely ignore the scary warning.)
      * Select Click on [Install Now]
      * Click on [Continue].

   G. Time Zone:
      *Select the correct time zone.

   H. Account setup:
      * Your name:
        Use your properly capitalize first and last name.
      * Your computer's name:
        Using a lower case name pick a name for the computer.
        For exam
      * Pick a user name:
        All lower case and one word.
      * Choose a password...:
        Type in your password twice.
      * Select "Require my password to log in".

   H. The installation takes some time.

   I. Click on [Restart Now]

   J. Ignore the "Please remove the installation medium" message, just press [Enter].

   K. Just type [Enter] for the Grub splash screen.

   L. Click on you user name and type in your password.

   M. Click through the startup screens.

   N. If the software updater window pops up click on [Install Now] and type in your password.
      If it pops up a restart machine window, just ignore it for now.

5. Install Guest Additions

   The guest extensions provide a more seamless integration between VirtualBox and
   the host machine.  For some reason, the documentation at for VirtualBox guest
   extension installation is actually pretty confusing.  Please try the instructions below:

   A. Bring up a terminal window by typing [Ctrl-Alt-T]  (use the left [Ctrl] key.)

   B. Type the following commands.  `sudo` will probably prompt you for your password.
   
      * `sudo apt update`
      * `sudo apt install build-essential dkms linux-headers-$(uname -r) --yes`

   C. Click on [Devices => Insert Guest Additions CD Image...]

   D. It should ask if it is OK to run some files.  Click [OK].

   E. Type in your password and click [Authenticate].
      This can take a while.

   F. Type [Enter] when [Press Return to close this window...] shows up.
      If your virtual machine reboots you can skip the next step.

   G. (Optional) If for some reason the virtual guest extensions do not properly
      install.  The following commands provide and alternative way of
      getting them installed.  Now run the following commands:

      * `sudo mkdir -p /tmp/cdrom`
      * `sudo mount /dev/cdrom /tmp/cdrom`
      * `sudo sh /tmp/cdrom/VBoxLinuxAdditions.run --nox11`
      * `sudo shutdown -r now`

      The machine should reboot.

   H. Select your user name and login with your password.

   I. Resize the virtual machine window to make sure the guest additions have taken hold.
      The background window should fill as it resizes.  If not, it will stay at a fixed
      size and something has gone wrong with your guest additions install.

   J. In the upper right corner there is a downward pointing triangle.
      Click on it and select [power off].
   
   Your virtual machine is installed and in a powered down state.

6. Make a clone.

   Installing all of the software is sufficiently painful that it is desirable
   to avoid having to do it again.  VirtualBox allows you to make a clone of
   an existing system so that the original system can be reused in the future.
   It is really easy and does not consume very much extra disk space,
   so making a clone is strongly recommended.

   A. Bring up the VirtualBox manager window again.

   B. Right mouse click on the name of your virtual machine:

      * Name: Give the clone a new name like "Ubuntu18.04LTS_HR2"
      * Path: Use the default
      * MAC Address Policy: Use the default
      * Leave the additional options unclicked.
      * Click [Next>] to go to the next pop-up window.

   C. Clone type

      * Select [Linked clone].  This saves lot's of disk space
      * Click on [Clone] to finish the clone.

   D. Now start the clone by by double clicking on it.

## Download and Install Software

The software is generally deployed in three broad catagories:

* Ubuntu packages: for basic tools and software (e.g. Kicad, OpenSCAD, etc.)
* Cloned Git Repositories: this project and some projects it depends upon used
  `git` repositories.
* Python `pip` packages: There is lots of Python 3 code sprinkled around this project
  and a bunch of it is packaged up as Python packages that can be installed with
  the Python `pip` program.  (By the way, `pip` stands for `Pip Installs Packages`.)
  There is heavy use of a technology called Python Virtual Environments.

There will be a number of cloned git repositories and they need to reside as
separate sub-directories under one parent repository directory.  (One parent
directory to rule them all!)  You can call this parent directory whatever you
want -- `repos`, `projects`, `funstuff`, `sauron`, etc.  For the directions
below we use `REPOS` for this parent directory name.  Please substitute the
directory name you picked for `REPOS` in the directions below:

### Clone 

There are 3 broad steps:

1. The first step is to clone the `hbrc_ros_robot_platform` repository.
   Let's get going:

        cd .../REPOS   # Change the current working directory to REPOS
        # We need to get `git` installed.
        # If `git` is already installed, skip the install below:
        sudo apt install git --yes # You may be asked for you user password:
        # Now "clone" the `hbrc_ros_robot_platform` repository using `git`:
	git clone -o upstream https://github.com/hbrobotics/hbrc_ros_robot_platform.git
        # Change the current working directory to the root of the cloned repository
        cd hbrc_ros_robot_platform
    
2. The second step is to run one or more installation scripts.  There are currently
   three scripts:

   * `mechancical/install_me.sh`:
     This script resides in the `mechanical` sub-directory and will install everything
     needed for the mechanical aspects of the project:

          ./mechanical/install_me.sh  # Install the mechanical portion of the project.

   * `electrical/install_ee.sh`:
     This script resides in the `electrical` sub-directory and will install everything
     needed for the mechanical aspects of the project.

          `./electrical/install_ee.sh  # Install the electrical portion of this project.`

   * `install_all.sh`:
     This script just installs everything, but takes the longest.

          `./install.sh  # Install everything.`

   Please run one of the three scripts mentioned above.  It will download a bunch of stuff.
     
3. This is important.  You need to activate your Python Virtual environtment.
   This is done as follows:

          `workon hr2   # Activate the hr2 Python virtual environment`

4. There are a bunch of `Makefile`'s sprinkled through out the various
   sub directories in the project.  The `make` program can recursively visit each
   of these `Makefile`'s and do any additonal needed steps.  There are two `make`
   targets in all `Makefile`'s:

   * `all`: This is the normal do day-to-day target that is used when you
     simply type `make`.
   * `everything`: This is meant to be used once since it does additional steps
     that are rarely needed -- create images, `dxf` files, etc.  This target
     is triggered by typing `make everything`.

5. There are two things left to do:

   A. Enable your python virtual environment:

           source ~/.bashrc   # Prepare for python virtual environments
           workon hr2         # Enable the hr2 virtual environment

   B. Use the `make` program to recursively build `everything` by typing:

           make everything

   For the first time through, we use the `everything` target:

        make `everything`  # Force everything to be recursively made.

Installing is a notoriously fragile process and it is quite possible that
something can go wrong.  If so, please send a message to the group list with
the `DL:` prefix on the subject line explaining the issue so it can be resolved.

### Workflow

Workflow is a generic term for how modifications are made to this project.

Examples of such modifications are.

* Modifying the mechanical models,
* Modifying electrical design and associated printed circuit board designs,
* Modifying the software,
* Fixing documentation errors,
* Developing educational course materials, Etc.

Some terminology here.  We are going to use the pronoun "you" to mean "you the
person doing project development" in the description below.

#### Workflow Introduction

There are two common workflows supported by GitHub.com:

* Shared Repository Workflow:
  This is the simplest model where there is one common shared parent repository stored
  at GitHub.com and you have local cloned copy of the shared parent repository stored
  locally on your development machine.  You develop and test your changes on your local
  machine.  When you decide your changes are "ready", you "push" the changes back up to
  the shared parent repository on GitHub.com.  Over time, you and other developers
  "pull" the accumulated changes stored in the parent repository down into your
  respective local repositories to stay up to date.  This workflow is extremely
  common and has been supported by many version control systems for literally decades.

* Fork and Pull Request Workflow:
  In this model, the common parent repository is the same as the shared repository
  workflow immediately above.  However, there now you need two repositories to do
  development.  The first repository is the same local repository on your local machine
  as before.  In addition to your local repository, you also have your own semi-private
  repository stored on GitHub.com that is used for staging purposes.  This staging repository
  is goes by a less descriptive name called a "fork repository".  Your fork repository
  is "connected" to the shared parent repository.  Since there are multiple developers,
  there multiple "forks" of the same shared parent directory on GitHub.com, where each
  is stored in a different user account to keep them separate from one another.

  The workflow is that you make modifications to your local repository and "push"
  them up to your staging "fork" repository.  Next, you generate something called
  a "pull request".  (Frankly, the term "pull request" is miss-named, it should be
  called a "merge request".  It is too late to change the terminology now, so we
  will stick with the less descriptive term of "pull request".)  The details of how
  to generate a "pull request" are discussed  little be further below.  The "pull request"
  generates an E-mail that is sent to one or more reviewers.   The reviewer looks at your
  changes using the GitHub.com GUI (Graphical User Interface.)  If the reviewer likes
  the changes, they are immediately accepted and merged from your fork repository to
  the shared master repository.  If any reviewer has some issues, an E-mail dialog ensure
  where the reviewer says things like "I really like this!", "This won't work because ...",
  "Fix this tiny issue", "What about doing it this way...", and an occasional
  "No way because..."  As a result of some of the review comments, you may need to make
  additional changes and push them up to the fork repository.  When the pull-request
  E-mail exchange dialog dies down and all of your changes have been pushed up to your
  fork repository, the reviewer accepts the changes and merges them into the
  shared parent repository.  The "Fork and Pull Request Workflow" is shown immediately
  below using some crude ASCII art:

                [Shared Parent]<---------------[Simi-Private Fork]   GitHub.Com
                       |                                ^
                ====== | ============================== | =====================
                       v                                |
                       +--------->[Private Local]------>+         Local Machine
                        
                         Fork and Pull Request Workflow
			   
The key advantage of the Fork and Pull Request work flow is that you do not need
permission from anybody to start making your changes.  You start your changes locally
and ask for permission only after you think they are ready.  When the reviewer
agrees, your changes are merged in.

It should come as no surprise to you that this project is using the fork and pull
request workflow.

#### Workflow Setup

If you have followed the download and install instructions above, you have a
local copy of the `hbrc_ros_robot_platform` repository as a sub directory under
your REPOS directory.  What you do not have is a staging fork repository *AND*
you have not configured `git` to do an asymmetric pull and push.  The instructions
below will remedy these issues.

Most of the tools you need to do the Fork and Pull Request workflow are already mostly
in place.  The primary tools are `git` and a program called `hub` which is a command
line interface to the GitHub.Com functionality.  We will do as much of the set up using
a command line tools as possible because the command line tools tend not to change as
fast as the web interface changes.  (If you are already comfortable the GitHub.Com web
interface, you can probably skip some of the steps below and use the web interface instead.)

There are a still a few manual steps to do:

1. The first step is to create your own personal account on GitHub.Com.  This is done
   visiting the main [GitHub.Com web page](https://github.com/) web page.  You need to
   specify a unique "Username" (all lower case with no spaces or punctuation is recommended),
   an "Email" address, and a "Password".  There will be a message sent your E-mail address
   that you must respond to with a response E-mail to finish the account creation.  The
   "Username" is your GitHub.Com account name and will be needed further below.  Do not
   log out of GitHub.Com yet.

2. In order to avoid having to type in a password all the time, we need to set up your
   computer to use an SSH key to talk to your GitHub.Com account.  The install scripts
   have ensured that an SSH key has been generated.  The public key is stored at
   `~/.ssh/id_rsa.pub`.  There is a
   [GitHub.Com SSH Key Installation](https://help.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account) web page that will guide you through the GitHub.Com
   web interface to install your public SSH key.  It is a little tedium for now, but will avoid
   much password prompting annoyance in the future.  If we are lucky, you should not need
   to deal with the GitHub.Com web interface any more.  Now you can disconnect from GitHub.Com.

3. Add GITHUB_ACCOUNT_NAME variable definition to your `~/.bashrc` file.  This is typically
   done a the end of the file by adding a line that looks like:

      GITHUB_ACCOUNT_NAME=whatever_you_typed_in_in_the_previous_step
      source ~/.bashrc  # Do no forget this step.

4. Specify your `user.name` and `user.email` values for `git`.  If you have previoulsy
   done this, there is no need to do it again.  The two commands are:

     git config --global user.name "Your Name"
     git config --global user.email "YourEmail@WhereEver"

5. Please rerun one of the installation scripts `install_all.sh`, `electrical/install_ee.sh`
   of `mechanica/install_me.sh`.  They will create your remote fork repository on GitHub.com
   and create git remote link named `staging` that points to it.  (Run `git remote -v` to see
   it.)

That pretty much wraps up the additional steps need to enable the "Fork and pull-request"
workflow.  Next, it is time to try it out.

#### Workflow Usage

The `git` program does a great deal of stuff and it is outside the scope of this
document to attempt to cover it all.  Instead, we will explain just enough to get started.

If you have followed the steps above, you have a repository named `hbrc_ros_robot_platform`
sitting on you local machine.

* An `upstream` remote descriptor that points to the shared project repository.
* A `staging` remote descriptor that points to the simi-private forked directory
  that is used for staging purposes.
* A `master` branch which has be set up so that it only accepts updates from the
  `upstream` remote.

The first thing to understand is that you should never to development on the `master`
branch.  We have tried to make it difficult for you to screw up `master`, but somebody
who really wants to can screw it up anyhow.

The second thing to understand is that all development takes place in one or more
side branches, called development branches.  The way to create a new branch is by

     git checkout -b NEW_BRANCH_NAME master

where you supply a new and hopefully descriptive name for the new development branch.
After this command, the new branch will be active.  Now you can modify things to your
hearts content.  You can check the files in as many times as you want.  You can perform
as many commits as you want on the branch.  Please put descriptive comments in the
commit logs.

When you think you changes are ready to be merged into the project you initialiate
a pull-request.  This is done by the command:

     pull_request

This command is in the repository `bin` directory.  This will push your commit up
to the staging repository and generate a pull request.  You will have add a comment
into the pull request via an editor that will pop up.

That is basically it.

#### Coding Style

The vast majority of the code is written in Python, C, and some Shell scripts.

* Python:
  Python 3 only.  All of the code is pushed through `mypy` (with type-hinting),
  `flake8` to catch coding issues, and `pydocstyle` to catch documentation issues.
  In general, the pretty standard coding style for Python is used.  One quirk is that
  Wayne Gramlich hates the `self` variable and usually assigns it to a more descriptive
  variable name.  Other than that, it is pretty standard.

* C:
  Pretty standard C coding style is used.  There are 4 spaces for each indentation level.
  The "then" clause of an if statement always has {...}, no exceptions.

* Shell:
  Shell scripts are tedious to write and debug and still tend to be fragile.
  Lot's of comments are used.

All documentation is written in [markdown](https://daringfireball.net/projects/markdown/).
Speaking of document, it is treated just like code in that it has bugs in it as well
(miss-spelled words, bad grammar, unclear desciptions, etc.)  The documentation is fixed
the same way that code is fixed, using the standard workflow.

Eventually we will use the Github issue tracker to track both code and documentation issues.
For now, the group mailing list will be used instead.

<!-- Random

        ;;; Make emacs notice when buffers change due to switching branches:
        (global-auto-revert-mode 1)




-->

