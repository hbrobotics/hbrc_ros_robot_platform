# HR2 Robot Software

<!-- ==================================== 100 Characters ======================================= -->

This document discusses how to download, install, and develop software for the HR2 platform.
This document is broken into the following sections:

* (Introduction)[#introduction]:
  The introduction to the whole software development process.

* (Software Download and Install)[#software-download-and-install].
  How to download and install the needed software.

* (Firmware Development)[#firmware-development].
  The workflow for developing microcontroller firmware.

## Introduction

There are three processors of interest:

* Development Computer:
  The development computer is used for all software development.
  Nominally it is a 64-bit x86 architecture processor that runs the Ubuntu 20.04 Linux distribution.
  The development computer runs both [ROS2](https://index.ros.org/doc/ros2/) and
  the microcontroller development software  (discussed shortly below.)
  The development computer needs to be able to access a local WiFi network
  in order to communicate with the HR2 robot computer.

* Robot Computer:
  The robot computer is either a 4GB or 8GB  Raspberry Pi 4
  and runs ROS2 under the Ubuntu 20.04  Linux distribution.
  (By the way, 4G is more than adequate.)
  The robot computer acts as the intermediary between the Development computer
  and the Robot Microcontroller (see immediately below.)
  The Robot Computer communicates with the development computer via the local Wifi network.

* Robot Microcontroller:
  The robot microcontroller is an STM (ST Microelectronics) 32-bit microcontroller
  that resides on the HR2 robot that handles lower level functions (motors, sensors, etc.)
  Currently, it is an STM32F767ZI, but that could change later on.
  All microcontroller firmware is developed on the development computer
  using a cross platform firmware development environment.
  The developed firmware is downloaded to the microcontroller
  using either a cable or over WiFi, whichever is most convenient.

The ultimate goal is to be able to develop all software on the development computer and
download/execute/debug the developed software to the appropriate processor.

## Software Download and Install

Each computer/microcontroller needs is its own software development software.
Please use hardwired network connections for all initial software installs.
Setting up Wifi properly is a discussed much further below.

This is broken into the following sections:

* [Development Computer Download and Install](#development-computer-download-and-install)

* [Robot Computer Download and Install](#robot_compute_download-and-install)

### Development Computer Download and Install:
  
There are broad steps for installing development computer software..

1. [Development Computer Ubuntu 20.04 Install](#development-computer-ubuntu-20.04-install):
   Get the Ubuntu 20.04 Linux distribution running on your development computer

2. [Development Computer ROS2 Install](#development-computer-ros-install):
   Install ROS2 (Foxy) on your development computer.

3. [Development Computer STM32CubeIDE Install](#development-computer-stm32cubeide-install):
   Install the STM32CubeIDE microcontroller software on development computer.

#### Development Computer Ubuntu 20.04 Install

The development computer is assumed to be running Ubuntu 20.04.
It is beyond the scope of this document explain how to download Ubuntu 20.04.

You may do one of the following:

* Purchase a laptop with Ubuntu preinstalled.

* Install Ubuntu 20.04 onto an 64-bit x86 processor.

* Install Ubuntu 20.04 to run under a virtualization environment
  like (VirtualBox)[https://www.virtualbox.org/].)

#### Development Computer ROS2 Install

Once you have Ubuntu 20.04 installed, please install the ROS2 version named
[Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/) which configured for Ubuntu 20.04.

#### Development Computer STM32CubeIDE Install

Downloading the STM32CubeIDE is pretty involved.
In the instructions below, the following notation in all capital letters shows up:

* HOME: This is the name of your home directory (e.g. `/home/alice`).
* PASSWORD: This is your login password.
* X: The major version of the `STM32CubeIDE`.
* Y: The major version of the `STM32CubeIDE`.
* D: Decimal digits that change from release to release.

Visit the [STMe2CubeIDE download page.](https://www.st.com/en/development-tools/stm32cubeide.html).

Basically, you must create an account with ST before you can get the software.
Just click on the [Get Software] button on the web mentioned immediately above to start the process.
Fill in the forms and wait for the account to happen.
(In the past, it took a while, but perhaps account setup is bit faster now.)
ST will offer the you the wonderful opportunity to let them send you lots of marketing information,
but they also give you the opportunity to decline this generous offer.

You need to be logged into you ST account before they will graciously allow you
to download the wonderful STM32CubeMX install and setup program.
Eventually, you will get your hands on a `.zip` file with a long ugly name
that looks vaguely like this:

     en.en-st-stm32cubeide_1-X-Y_DDDD_DDDDDDDD_DDDD_amd64_sh.zip

Obviously, the version numbers change over time, so an exact match with the
file name immediately above is extremely unlikely.
You need to remember the major and version numbers (e.g. X and Y)
for one step in the installation process below.
Please store the `.zip` file someplace, just in case you need to reinstall in the future.

Now, unzip the file:

     unzip -x en.en-st-stm32cubeide_1-X-Y_DDDD_DDDDDDDD_DDDD_amd64_sh.zip

This should result in a `.sh` (shell) file that looks kind of like:

     st-stm32cubeide_1.X.Y_DDDD_DDDDDDDD_DDDD_amd64.sh

(Again, the version information is always changing.)

This is the installer file.
It is going to make you agree to a whole bunch of license agreements, install stuff in places, etc.
There are also a few steps that need to be done in super user mode.

Run the `groups` command and verify that you are in the `sudo` group.
If not, search the web and find a web page that explains how to become a member of the `sudo` group.

Before running the installation shell script,
the installation directory needs some discussion.
The installer want to install the software in the `/home/HOME/st/stm32cubide_1.X.Y` directory.
For a relatively obscure reason having to do with generated `.launch` files
and subsequent interactions with the `git` version control system,
it is better if you install the software in a more global location.
The preferred global installation location is `/opt/st/stm32cubeide_1.X.Y`.

Now run the file:

     sh st-stm32cubeide_1.X.Y_DDDD_DDDDDDDD_DDDD_amd64.sh

When running the exact file above, the following prompts occurred:

* `Verifying archive integrity... All good.`
* `Uncompressing STM32CubeIDE 1.X.Y installer  100%`
* `STMicroelectronics Software License Agreement`
* *Big long license agreement*
* `I ACCEPT (y) / I DO NOT ACCEPT (N) [N/y] y`
* `License accepted.`
* `Do you want to install STLinkServer and Udev rules for STLink and Jlink?`
* `Without these packages, you will not be able to use debugging feature.`
* `Install them? (you need sudo permission to do that) [Y/n]Y`

The next prompt is the install directory mentioned above.
Please use the global installation location described above.
This the place where you type in type in more global installation location:

* `STM32CubeIDE install dir? [/home/HOME/st/stm32cubeide_1.X.Y/] /opt/st/stm32cubeide_1.X.Y`
* `Installing STM32CubeIDE into /opt/st/stm32cubeid_1.X.Y ...
* Java cacerts symlinked to /etc/ssl/certs/java/cacerts
* `[sudo] password for HOME: PASSWORD`
* `Creating directory root`
* `Verifying archive integrity... All good.`
* `Uncompressing STM udev rules installer  100%`
* `STMicroelectronics Software License Agreement`
* *Big long license agreement*
* `I ACCEPT (y) / I DO NOT ACCEPT (N) [N/y] y`
* `STMicroelectronics Software License Agreement`
* Another ginormous license agreement here.
* `I ACCEPT (y) / I DO NOT ACCEPT (N) [N/y] y`
* `License accepted.`
* `stlink-server v2.Z.Z (ZZZZ-ZZ-ZZ-ZZ:ZZ) installation started.
* More stuff.
* `Do you want to install Segger J-Link udev rules? [Y/n] n`
* `STM32CubeIDE installed successfully`

The final step of installing STM32CubeIDE is to put the executable in your path.
Using your favorite text editor, add the following to your `~/.bashrc` file:

     export PATH=$PATH:/opt/st/stm32cubeide_1.X.Y/stm32cubeide

If you do not have a favorite text editor for ubuntu, you can start with `nano`.

Now `source` the `~/.bashrc` file as follows:

     source ~/.bashrc

The command

     which stm32cubeide

should return `/opt/st/stm32cubeide_1.X.Y/stm32cubeide`.
If it does, you have successfully installed stm32cubeide.

That concludes software installation for the 

### Robot Computer Download and Install:

Installing software on the Robot Computer (i.e. Raspberry Pi 4) involves the following steps:

* [Robot Computer Ubuntu 20.04 Installation](#robot-computer-ubuntu-20.04-installation)

* [Robot Computer ROS2 Install](#robot-computer-ros2-install)

#### Robot Computer Ubuntu 20.04 Installation

Download and install
(Ubuntu 20.04)[https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4]
on the Raspberry Pi 4.
Alas, this page designed to be read in full screen mode.
(When will web page designers ever learn that requiring a full screen is a bad idea?)

#### Robot Computer ROS2 Install

There is a short [ROS2 Install Video](https://www.youtube.com/watch?v=AmuLiA840fA)
that walks you through the
[ROS2 Install Debians](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)
web page instructions.
Please follow these instructions.

### Software Download and Install Complete

The software download and install should now be complete

## Firmware Development

Developing firmware is done using `STM32CubeIDE`.
`STM` stands for `ST Microsystems` (where `ST` stands for `SGS-Thomson`).
It is unlear what `Cube` stands for, it was probably cooked up by the marketing department.
`IDE` stands for `Integrated Development Environment`.
The program is named `stm32cubeide` and should be in your execution path.

Before you run the `stm32cubeide` it is worth mentioning that this program
likes to fill your home directory with a bunch of sub-directories.
Currently, there are 4 "dot" directories:

* `.stm32cubeide`
* `.stm32cubemx`
* `.stmcube`
* `.stmcufinder`

In addition there are two non "dot" directories.

* `STM32Cube`:
   This is a huge directory contains my gigabytes of files.
* `STM32CubeIDE`:
  This is the default location of your workspace which is discussed further below.

If you ever need to clean `stm32cube` off your computer,
delete these 6 directories *AND* the `/opt/st` directory.

The next thing to understand is that this IDE (Integrated Development Environment)
uses the concepts of workspace and project:

* Workspace:
  A workspace is a directory where the IDE stores miscellaneous files needed by the IDE.
  In particular, this is the place where the IDE keeps track of your projects.
  Your workspace directory is generally not keep under revision control.
  The workspace directory defaults to `~/STME32CubeIDE/workspace/workspace_1.X.Y/`.
  It is recommended that you use this default workspace location.

* Project:
  A project is a directory that is self contained piece of software that can be downloaded
  to a microcontroller.
  The project directory *can* be stored as a sub-directory in the workspace.
  However, for this project that is not recommended.
  Instead, projects are store in a different location (the HR2 `git` Repository).
  None-the-less, the workspace can work with these projects in various different directories.
  Projects can easily be added and removed from a workspace.

The Project directory has some additional structure that is discussed further below.

Note that the user interface people are constantly changing things around,
so these descriptions will get out of date fast.

Now it is time run `stm32cubeide`:

1. The first window that pops up is the `Select a directory as workspace` window.
   The default directory is as described above.
   You might want to check off the box labeled
   `[x] Use this as the default and do not ask again`.
   Finally click on the `[Launch]` button.

2. Next comes the `Welcome...` screen.

3. Click on `[File]` => 'New STM32 Project'.

4. Next pops up the `Target Selection Window`.

5. There are 4 tabs -- `[MCU/MPU Selector]`, `[Board Selector]`, and two others.
   The first tab is used to select raw chips 

   Click on `[Board Selector]`.

6. In the `Commercial Part Number` entry field, type `767`.
   This will constrain the search to board to boards that contain `767`.

7. Click on `[NUCLEO-F767ZI]` followed by `[Next]`.

8. The `STM32 Project` window comes up.
   The options should default to `C`, `Executable`, and `STM32Cube`.

9. Type your project name `blinky` into the `Project Name:` field.

10. In a console window, type `mkdir /tmp/blinky`,
    to create a temporary directory to experiment with a blinking diode application.

11. Back in the `STM32 Project` window,
    unclick `[x] Use default location`, and
    type in `/tmp/blinky` into to the `Location:` entry field.

12. Now click on `[Finish]`.

13. A window pops up with `Initialize all peripherals with their default Mode ?`.
    Click on `[Yes]`.

14. Next a
    `Device Configuration Tool editor is associated with Device Configuration Toolperspective.`
    `Do you want to open this perspective now?` pop-up occurs.
    Click on `[Yes]`.

15. At this point a huge amount of additional files get downloaded and unzipped.
    Thankfully, these files rarely need to be downloaded again.

16. At this point in time, the `/tmp/blinky/` directory has been populated
    a bunch of files and sub-directories.

17. On the left is the `[Project Explorer]` tab which shows all of the files
    that have been populated into `/tmp/blinky/`.
    At the top level are the `Includes/`, `Core/`, and `Drivers/` directories.
    Generally speaking `Core/` sub-directory contains source code that can be edited by you,
    whereas the `Includes/` and `Drivers/` sub-directories contains the header files
    that should not be edited by you.
    In addition, the `blinky.ioc` file is generated,
    which specifies all of the pin bindings for the microcontroller.
    All `.ioc` files are managed by the `stm32cubeide`.
   
18. The `[blinky.ioc]` tab contains a tabbed panel with 4 sub-panels.

    These are:
    * `[Pinout & Configuration]`:
      This contains a picture of the `STM32F767ZI` microcontroller.
    * `[Clock Configuration]`:
      This contains an amazingly complex timing chart.
    * `[Project Manager]`:
      This is panel that is similar to `[Project Window]` used to originally create the project.
      It has some additional side tabs and features that can be ignored for now.
    * `[Tools]`:
      This is some advanced tools for power management.
    
19. In the `[Pinout & COnfiguration]' tab, the Ethernet peripheral needs to be disabled.
    This id done by clicking `[Connectivity]` => `[Eth]`.
    Next, in `ETH Mode and Configuration` there is a `Mode` scrollable drop down menu
    that shows `[RMII]`.
    Select the drop down menu and scroll upward to `[Disable]`.

20. In the top icon bar, click on the hammer to build the `[Debug]` configuration.

21. Now is a good time to plug the Nucleo-767ZI board into your development computer
    using a USB cable with male USB mini connector on it.
    There are two female USB connectors on the Nucleo-767ZI board.
    There is one next to the large RJ45 connector
    and there is one on the opposite end that is for the ST-Link module.
    Plug into the ST-Link module.
    A big red LED on ST-Link module should light up when you plug into it.
    In addition much smaller green power LED on the "main" board will light up.
    The big red LED will flash back and for between red and green when the 
    
22. When you run `lssub` in a console window, you should see a line that contains
    `STMicroelectronics ST-LINK/V2.1` or something quite similar to it.
    This means that your development computer sees the ST-Link board and
    it should be ready to use.
    
23. Now it is time to download and the null program into the microcontroller and "debug it".
    Up in the icon bar at the top, there is a bug with six legs.
    This is the debugger icon and to the right of this icon is a pull down menu.
    Click on the pull down menu and it will give some options.
    Click on the `Debug As` pull right menu and verify that it says
    `STM32/Cortex M C/C++ Application`.
    It may put up a `[Choose launch configuration to debug]` window.
    Please select on `[blinky Debug]` and click `[OK]`.
    If you look quickly you will see the ST-link LED flashing between Red and Green.
    
24. Frequently, a `[Confirm Perspective Switch]` pop-up will appear.
    Please click on `[Switch]`.
    This window comes up a lot, so you make decide to click on `[x] Remember my decision`.
    

25. Next to this icon is a drop down menu with downward pointing triangle.
    Click on this triangle to get the drop down menu.
    
26. The `[main.c]` table should be selected and a line labeld `HAL_Init();` should be highlighted.
    To the left of this should be a little blue arrow that indicates where
    the program is currently paused.

27. There are a number of debugger icons that showed up.
    There is an icon Red square with green triangle underneath.
    This the `[Terminate and Relaunch]` button.
    If you hover over this button, hover help shows up.
    Move to there right to hover help for each debugger icons.
    The three most important ones are the single stepping icons which are yellow arrows.
    (One my be grayed out.)

29. Click on the `[Terminate]` icon (big red square).
    This causes the ST-Link to disconnect and show the LED Red rather than flashing Red/Green.

30. Now this temporary project can be deleted by going to the `[Project Explorer]` tab.
    Right click on the `[blinky]` project, and select `[X Delete]`, and click on `[OK]`.
    Now the temporary project is gone from the workspace.

<!--

###################################################################################################

More Notes:

* The code seems to stash files all over the place:
  * `~/STM32Cube/`: All of the files land here.
  * `~/STM32CubeIDE/`:
  * `~/STM32CubeIDE/workspace/`:
  * `~/STM32CubeIDE/workspace/workspace_1.5.0/`:
  * `~/STM32CubeIDE/workspace/workspace_1.5.0/.metadata/`:  This 
  * `~/stm32cubeide/`:
  * `~/stm32cubemx/`:
  * `~/stm32cube/`:
  * `~/stm32cufinder/`:

Notes:

People started stuff should start with `stm32cubeide` from STM(STMicroelectronics.)

[STM32CubeIDE ST-LINK GDB server](https://www.st.com/resource/en/user_manual/dm00613038-stm32cubeide-st-link-gdb-server-stmicroelectronics.pdf)

[Another link to previous solution](https://stackoverflow.com/questions/57312510/debug-remotely-on-stm32cubeide-with-an-stm32-eval-board)


[Where to find stlink-gdb-server](https://community.st.com/s/question/0D50X0000BmnDcXSQU/question-on-stm32cubeide-and-remote-debugging)
They do not have a Linux version of this software.

[F5/F6 key Config](https://hbfsrobotics.com/blog/configuring-vs-code-arm-development-stm32cubemx)

## Roadmap

The steps required to do this are:

1. Blinky:
   Install enough code on the host computer to be able compile and download
   a blinkng LED program (i.e. blinky) onto an STM Nucleo board.
   Debugging is not supported yet.

2. Codium:
   Download `vscodium` (the community maintained version of `vscode`.)
   Configure it to support download *and* debug.

3. Host Computer ROS2 Install:
   Install ROS2 on a Ubuntu 20.04 host machine.

4. Robot Computer ROS2 Install:
   Bring up Ubuntu on the Robot computer, bring up WiFi connectivity, and download ROS2.

5. MicroROS:
   Bring up MicroROS on the Nucleo board and get it to talk to the Raspberry Pi 4.


## Current Status: Working through Embedded Geek Videos:

I'm currently about half way through the first video:
1. [Command line](https://www.youtube.com/watch?v=PxQw5_7yI8Q)

     openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"


2. [VSCode](https://www.youtube.com/watch?v=xaC5oWwzOt0)

## Blinky

The blinky task is to download a trivial blinking LED program into Robot microcontroller.
This ensures that the necessary tools (i.e. the tool chain) is present
and a reasonable workflow is present.

Two paths strategies were ultimately considered:

* `stm32cubeide`:
  `stm32cubeide` is a version of the Eclipse IDE (Integrated Development Environment)
  that has been configured for STM microcontroller software development.
  In particular, is integrated with a piece of code that STM calls `stm32cubemx`.
  `stm32cubemx` is program for configuring the various peripherals inside of an STM microcontroller.

* `vscode`:
  `vscode` is an IDE (Integrated Development Environment) that is developed and maintained
  primarily by Microsoft.
  It is primarily written in C# and is released under a the MIT license.
  Importantly, it seems to be possible to configure it to work well with `git`.

In the end, the `vscode` path won out over the `stm32cubeide`.
The reason was subtle and had to do issues of getting `stm32cubeide` to work well with `git`.
`stm32cubeide` likes to embed the full paths to files in its configuration files
where as `vscode` does not.
Various workarounds were found on the net, but they were all pretty ugly.
The `vscode` option is viable is because the critical piece of software is `stmcubemx` is
available as a standalone program.

The road map for blinky is:

1. Install Common Tools.
   The common tools need to be installed.

2. Install `STM32CubeMX`:
   The `stm32cubemx` program needs to be installed.

3. Install ARM Cross Compilers:
   ARM cross compiliers need to be installed.

4. Create an `STM32CubeMX` Project:

### Install Common Tools

The common tools are installed via `sudo apt install --yes` which downloads the
necessary packages from the Ubuntu main package repository:

     sudo apt install --yes build-essential git  # Needed for make, git, etc.
     sudo apt install --yes openocd  # Needed for download/debug
     sudo apt install --yes openjdk-8-jdk  # Needed for `stm32cubemx`
     sudo apt install --yes libncurses5 libncurses5:i386  # Needed for ARM `gdb` debugger

### Install STM32CubeMX

The STM32CubeMX is program developed by ST that is used to configure various ST microcontrollers.
ST has developed a HAL (Hardware Abstraction Library) that is shared across the entire product line.
The STM32CubeMX program generates code that makes calls to the HAL.
It very much worth the annoyance and tedium of downloading and installing this program.

The STM32CubeMX is not open source software.
STM32CubeMX appears to be written in Java and needs to run under the JRE (Java Runtime Environment).
In a step above, the JRE is installed.

In order to run this software the following steps need to occur:

1. Download.
   Download and install the STM32CubeMX.

3. Wrapper.
   Create a wrapper that calls STM3CubeMX.

The download and install process is pretty involved and is frankly pretty annoying.

#### Download the STM32CubeMX:

Downloading the STM32CubeMX is pretty involved.

The [STM32CubeMX Web Page](https://www.st.com/en/development-tools/stm32cubemx.html)
is a good place to start.
Note, whenever ST decides to reorganize their web site, the link above will probably fail.

Basically, you must create an account with ST before you can get the software.
Just click on the [Get Software] button on the web mentioned immediately above to start the process.
Fill in the forms and wait for the account to happen.
(In the past, it took a while, but perhaps account setup is bit faster now.)
ST will offer the you the wonderful opportunity to let them send you lots of marketing information,
but they also give you the opportunity to decline this generous offer.

You need to be logged into you ST account before they will graciously allow you
to download the wonderful STM32CubeMX install and setup program.
Eventually, you will get your hands of a file named something like:

     SetupSTM32CubeMX-6.1.1.linux

The version number will change over time.
This is an executable 64-bit x86-64 executable file.
When you execute this file (i.e. `./SetupSTM32CubeMX-6.1.1.linux`),
it will make you accept their license agreement again (you can never be too sure):

* [X] I accept the terms of this license agreement.
* [X] I have read and understood the ST Privacy and ST Terms of Use
* [ ] I consent that ST in N.V. ...   (no need to click this one!)
* Install directory:  Pick someplace like `.../stm32cubemx/stm32cubemx-6.1.1`.
  Make sure the directory does not previously exist.
* [OK]  The target directory will be created.
* [X] Auto installer (seems harmless)

Obviously, the sequence of questions will change over time at the discretion of ST.
The final executable is should show up as something like
`.../stm32cubemx/stm32cubemx-6.1.1/STM32CubeMX`.

#### `stm32cubemx` Wrapper

Due to the fact that the code is ultimately running the byte codes in the JRE,
it is best to provide a wrapper that executes the program.
The primary reason for wrapper is to make sure that JRE can find the byte codes.
The safest way to ensure that it always finds the byte codes is to always execute the program
with a full execution path specified (i.e. `/.../STM32CubeMX`.)

The final executable tends spews out a great deal of text to the console.
This first time you run the executable,
you can see what sort of exciting information is being spewed to the console.
After that you may choose run it with `> /dev/null` tacked onto the end
in order to send the intensely exciting console information to the bit bucket.

Add the following to your `.bashrc` file (or equivalent):

     export STM32CUBEMX_BIN_DIR="$HOME/download/stm32cubemx/stm32cubemx-6.1.1"
     export PATH=":$STM32CUBEMX_BIN_DIR:$PATH:"

Now in the same directory that contains `STM32CubeMX`,
create a file called `stm32cubemx` (lower case) with the following content:

     #!/usr/bin/env bash
     # STM32CUBEMX_BIN_DIR must be defined in `$HOME/.bashrc`:
     $STM32CUBEMX_BIN_DIR/STM32CubeMX "$@" > /dev/null

Remember the `> /dev/null` on the last line is optional,
but most people will prefer to leave it in there.

In order to test the `.bashrc` modifications, do the following:

     source ~/.basrhc
     stm32cubemx

The `stm32cubemx` should cause the initial window of the `STM32CubeMX` program to pop up.
The first time the program is started, it will ask additional questions.
It really wants to collect information about your usage, but you can say "No thanks".

Ultimately, the STM32CubeMX program is used to create and manage an `.ioc` file.
Presumably, `.ioc` stands for I/O Configuration, but it could *stand* for something else.
In addition to managing the `.ioc` file, it also generates configuration code for the HAL library.

### Install ARM Cross compilers

Every quarter,
Arm Holdings releases a bunch of Arm cross-compilers that run on the x86 architecture,
but generate code for the Arm architecture.
The (ARM cross compilers web page)[https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads],
provides links to the appropriate downloads.
The one of interest is `gcc-arm-none-eabi-...x86_64-linux.tar.bz2`,
where `...` specifies the `VV-YYYY-qQ` (VV=10+, YYYY=202x, Q=1-4).

Store this file into a directory like `.../gcc-arm-none-eabi`.
Unpack using the following commands:

     bunzip2 *.bz2
     tar xvf *.tar

This should result in a sub-directory that looks something like
`gcc-arm-none-eabi-10-2020-q4-major`.

Edit your `~.bashrc` (or equivalent) to do add the followings:
    
     # Fill in `...` as appropriate:
     export GCC_ARM_BIN=".../gcc-arm-none-eabi/gcc-arm-none-eabi-10-2020-q4-major/bin"

Be sure to make it match the actual version that was downloaded, since they are updated quarterly.

###  Create an `STM32CubeMX` Project:

If everything has gone right, the following programs should be in your path:

     gcc-arm-none-eabi-gdb --version
     openocd --version
     codium --version
     stm32cubemx  

## Construct an STM32CubeMX project.

The STM32CubeMX program organizes things into a WORKSPACE directory and
associated PROJECT directories.
For these examples, the WORKSPACE directory is `$HR2_DIRECTORY/software/non_ide/`.
The blinky PROJECT directory is `$HR2_DIRECTORY/software/non_ide/f767zi_blinky`.

The WORKSPACE directory is created manually using:

     mkdir -p $HR2_DIRECTORY/software/non_ide/f767z

Now run `stm33cubemx`:

    `stm32cubemx`

1. Under the `New Project` heading, find `Start My project from ST Board`.
   (It may necessary to expand `stm32cubmx` full screen in order to see this text in its entirety.)
   Click on the `[Access To Board Selector]` button.
   A window shows up with multiple tabs.
   The selected tab should be `[Board Selector]`.
  
2. (Optional) There is a text entry box called `Commercial Part Number`
   into which you may type a partial part number.
   For example, if you type `767` in this constrain all parts to contain `767`.

3. Select the `MCU/MPU Series`.
   For the `767` part, this is the `[x] STM32F7`.

4. There will be a `Boards List` that lists the boards that match.
   In this case, we want `Nucleo-F767ZI`, so please click on it.

5. Next, click on `[ ]--> Start Project`

6. A Popup that says `? Initialize all peripherals with their default Mode? [Yes] [No]`
   shows up.  Click on `[Yes]`.

7. A window with 4 tabs shows up.
   The 4 tabs are:

   * `[Pinout & Configuration]`:
      This one shows the outline of the integrated circuit with various pins highlighted.
      This is the initially selected tab.
      For now, we ignore this tab.

   * `[Clock Configuration]`:
      This one shows the clock configuration diagram, which can be quite complicated.
      For now, leave this one alone.

   * `[Project Manager]`:
      This one is used for setting up the project.
      There are numerous things on this tab.
      In the next step below, this tab is used.

   * `[Tools]`:
         This is some advanced tools that are ignored for now.

8. Select the `[Project Manager]` tab and fill in the following fields:

   * `Project Name`:
      Give the project a name.
      It is recommended that the name include at some fraction of the processor.
      For example, `blinky_f767zi`.

      * `Project Location`:
         This the WORKSPACE directory.
	 You must either browse or manually type in the full path to this directory.
        
      * `Toolchain / IDE`:
         This is a selection box that is normally set to `[EWARM]`.
         It is very important to change this to `[Makefile]`

 9. Select the `[Pinout & Configuration]` tab:

    * Select the `Connectivity >` drop down menu and select `ETH`.

    * A `Mode` side window shows up.
      It probably is showing a `Mode` of `RMII`.
      Scroll up to the first entry of the drop down menu and select `Disable`.
      This will disable the intialization of the Ethernet device.

10. In the upper right hand corner there is a button labeled `[Generate Code]`.
    Please click this button. 
    After a few moments a popup window that says
    `The Code is successfully generated under :`
    followed by the directory of the form `/...WORKSPACE/PROJECT`.
    Whole bunch of files have been created in this project directory.
    The most interesting ones are

    * `PROJECT.ioc`:
      This file is the I/O configuration file.
    * `Makefile`:
      This file is used to build the project using the `make` program.
    * `Core/Src/main.c`:
      This is the main C program that initializes the microcontroller.

    Both the `Makefile` and the `main.c` file need to be edited.

11. Edit the `Makefile` to have the following target immediately after the `clean:` target:
	
         flash: all
                 openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

    Please make sure you use a [Tab] character as the first character on the line
    containing `openocd`.
    Spaces will not work.

12. Bring up `Core/Src/main.c` file in your favorite text editor.
    Search for `Infinite loop`.
    Make sure the code looks as follows:

          /* Infinite loop */

          /* USER CODE BEGIN WHILE */
          while (1)
          {
            HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            HAL_Delay(250);
            /* USER CODE END WHILE */

            /* USER CODE BEGIN 3 */
          }
          /* USER CODE END 3 */

    This code toggles the 3 LED pins and then delay for 250 microseconds between toggles.

13. Plug Nucleo-F767ZI into your Host Computer.

14. Verify that Host computer found it with `lsusb`.
    It should show up as something like `STMicroelectronics ST-LINK/V2.1`.

15. Build and download the code using

         make flash

    This should download and install the blinky code.
    If it does not, please carefully check M

This completes all of the steps needed to run the Blinky program.

## VSCodium

It is possible to download `vscode` from Microsoft.
The `vscode` downloaded from Microsoft is configured with some additional trackers installed.
There is a version of `vscode` called `vscodium` which is the pure `vscode` code under the
MIT license, but without the trackers.
The `vscodium` software can be downloaded and installed by following the download/install
instructions at the [VSCodiaum web site](https://vscodium.com/).
By the way, the actual program that is installed is called `codium`.


C/C++ 1.1.3: C/C++ IntelliSense, debugging, and code browsing.
Cortex-Debug 0.3.7 Arm Cortex-M Debugger support for VSCode


https://code.visualstudio.com/docs/cpp/config-msvc#_cc-configurations
  [Cntrl-Shift-P] Select edit C/C++ configurations .json.

From `Makefile`:
     CPU = -mcpu=cortex-m7
     FPU = -mfpu=fpv5-d16
     FLOAT-ABI = -mfloat-abi=hard
     C_DEFS= \
     -DUSE_HAL_DRIVER \
     -DSTM32F767xx
     MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

Add to `c_cpp_properties.json`:
     "name": "STM32 F7",
     "compilerPath": "${env:GCC_ARM_DIR}/arm-none-eabi-gcc",
     "compilerArgs": [
         "-mcpu=cortex-m7",
         "-mthumb",
         "-mfpu=fpv5-d16",
         "-mfloat-abi=hard",
     ],
     "defines": [
         "_DEBUG",
         "UNICODE",
         "_UNICODE",
         "USE_HAL_DRIVER",
         "STM32F767xx"
     ],
     "cStandard": "c11",  # Default is "gnu17",
     "cppStandard": "c++17",  # Default is "gnu++14",
     "intelliSenseMode": "gcc-arm"


[Cntrl-Shift-P] Type "Debug:Open launch.json":
It prompts for a target:  Select "Cortex Debug":

     "excutable": "./build/f7676_blinky.elf",
     "type": "openocd",  # Not "jlink"
     "device": "STM32F767ZI",
     "configfiles": [
	"interface/stlink-v2-1.cfg",
	"target/stm32f7x.cfg",


	openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg \
	   -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

Search for "vscode openocd":

[PlatformIO?](https://gist.github.com/neta540/923b21932a6da69e2fdb63644b905844)

[no response](https://stackoverflow.com/questions/49342654/how-to-run-openocd-gdb-server-from-visual-studio-code)

[VSCode OpenOCD Setup](https://www.justinmklam.com/posts/2017/10/vscode-debugger-setup/)

[Shows how VSCode tasks work](https://gist.github.com/janjongboom/51f2edbee8c965741465fa5feefe4cf1)

-->
