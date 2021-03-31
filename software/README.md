<!--
MIT License

Copyright 2020-21 Home Brew Robotics Club

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

# HR2 Robot Software

<!-- ==================================== 100 Characters ======================================= -->

This document discusses how to download, install, and develop software for the HR2 platform.
This document is currently broken into the following sections:

* [Introduction](#introduction):
  The introduction to the whole software development process.

* [Software Download and Install](#software-download-and-install):
  How to download and install the needed software.

* [Firmware Development](#firmware-development):
  The workflow for developing microcontroller firmware.

## Introduction

For the HR2 there are three computers/microcontrollers of interest:

* Development Computer:
  The development computer is used for all software development.
  It is a 64-bit x86 architecture processor that runs the Ubuntu 20.04 Linux distribution
  (possibly in under an emulator.)
  The development computer runs both [ROS2](https://index.ros.org/doc/ros2/) and
  the microcontroller development software  (discussed shortly below.)
  The development computer needs to be able to access a local WiFi network
  in order to communicate with the HR2 robot computer.

* Robot Computer:
  The robot computer is either a 4GB or 8GB  Raspberry Pi 4
  and runs ROS2 under the Ubuntu 20.04  Linux distribution.
  (By the way, 4GB of memory is more than adequate for the Raspberry Pi 4.)
  The robot computer acts as the intermediary between the Development computer
  and the Robot Microcontroller (see immediately below.)
  The Robot Computer communicates with the development computer via the local Wifi network.

* Robot Microcontroller:
  The robot microcontroller is an STM (ST Microelectronics) 32-bit microcontroller
  that resides on the HR2 robot that handles lower level functions (motors, sensors, etc.)
  Currently, it is an STM32F767ZI, but that could change later on.
  All microcontroller firmware is written and debugged on the development computer
  using a cross platform firmware development environment.
  The developed firmware is downloaded to the microcontroller
  using either a physical cable or over WiFi, whichever is most convenient.

The ultimate goal is to be able to develop all software on the development computer and
download/execute/debug the developed software to the appropriate computer.

There is some [additional needed hardware](#additional-needed-hardware) to perform the software
and firmware download.

## Software Download and Install

Each computer/microcontroller needs is its own software development software.
Please use hardwired network connections for all initial software installs.
Setting up Wifi properly is a discussed much further below.

This is broken into the following sections:

1. [Additional Needed Hardware](#additional-needed-hardware):
   Some additional cables, micro-SD cards, etc. are needed
   during software download and installation.

2. [Development Computer Ubuntu 20.04 Install](#development-computer-ubuntu-2004-install):
   Get the Ubuntu 20.04 Linux distribution running on your development computer

3. [Development Computer ROS2 Install](#development-computer-ros2-install):
   Install ROS2 (Foxy) on your development computer.

4. [Development Computer STM32CubeIDE Install](#development-computer-stm32cubeide-install):
   Install the STM32CubeIDE microcontroller software on development computer.

5. [Development Computer Configuration](#development-computer-configuration)
   Some additional configuration for the development computer.

6. [Robot Computer Ubuntu Installation](#robot-computer-ubuntu-installation)

7. [Robot Computer ROS2 Installation](#robot-computer-ros2-installation)

### Additional Needed Hardware

In addition to the three computers above in the [introduction](#introduction),
you need some additional hardware.
All of the possible needed hardware is listed here,
but discussed in greater detail in various sections below.

In the United States For the more specialized electronics,
[Digi-Key](https://www.digikey.com/) is a common electronics distributor.
Many of the other items can be purchased from vendors such as Amazon, NewEgg, etc.
For Digi-Key, links into their website are provided,
so you do not have to figure out how to find things on their website.

The additional required hardware is needed:

* [4GB Raspberry Pi 4](https://www.digikey.com/en/products/detail/raspberry-pi/RASPBERRY-PI-4B-4GB/10258781):
  This is robot computer immediately above in the [introduction](#introducion).

* [Raspberry Pi 4 Heat Sinks](https://www.digikey.com/en/products/detail/seeed-technology-co-ltd/110991327/10451876)
  The Raspberry Pi 4 runs hot and needs heat sinks.
  The HR2 master printed circuit board (PCB)  has holes in the board to radiate heat from the heat sinks.
  They come in package of 4 -- 1 large for the processor, 1 medium for the memory, and
  2 smaller ones for the Ethernet controller and USB controller.
  *DO NOT* power up the Raspberry Pi 4 microcontroller without first attaching the heat sinks.

* [Nucelo-F767ZI](https://www.digikey.com/en/products/detail/stmicroelectronics/NUCLEO-F767ZI/6004740)
  This is the robot microcontroller the [introduction](#introduction).

* Micro-SD to USB Adapter:
  There are plenty of these available out there.
  Search for "USB A to micro SD adapter".

* Micro-SD card:
  32GB micro-SD cards are pretty affordable.
  Make sure it is `UHS-1`.
  SanDisk is a common vendor, but there are others.
  These cards are really small and easy to lose.
  So keep careful track of them.
  Also, it does not hurt to buy an extra just in case you lose one.

* [Battery Pack](https://www.amazon.com/gp/product/B08F7XM5GN/ref=ppx_yo_dt_b_asin_title_o07_s00):
  The Raspberry Pi4 is powered via a USB-C connector.
  Batteries packs come and go.
  The one that is currently recommend is the CONXWAN 26800mAh Power bank.
  What is nice about this one is that it has 3 3A output connectors (1 USB-C and 2 USB-A).
  In addition, it can be charged either via the USB-C or via a forth USB micro connector.
  While the exact number power capacity (i.e. 26800mAh) is not that important,
  it is nice to have at least 2 3A outputs.
  Since some these battery packs do not come with a complete set of charging cables,
  it may be necessary to purchase additional cables.
  In general, the HR2 does not need long cables.
  Also, when the battery first pack shows up,
  be sure to charge it up  since it takes a while to charge these battery packs.

* USB-A to USB-C Cable:
  The robot is powered via a USB-A to USB-C cable.
  Shorter cables are desirable (6in/15cm) to (12in/30cm).
  The theory was that all USB cables with the correct connectors on the end would "just work".
  In practice, the marketplace has learned to economize by creating specialized "power" cables
  which are slightly less expensive to manufacture.
  Thus, take any cable that is the correct length and do not worry about whether is says "power" or not.

For bringing up the Raspberry Pi 4, it is frequently useful to have some additional hardware.
The hardware is listed below is primarily used for bringing up WiFi on the Raspberry Pi 4.
For completeness, the hardware is listed here, but its usage is discussed more completely
in the [Robot Computer Ubuntu Installation](#robot-computer-ubuntu-installation) section
of this document.
Be sure to read about [keyboard/display vs. headless](#robot-computer-ubuntu-installation) configuration
before purchasing any of the hardware immediately below:

* HDMI Display (for keyboard/display based configuration):
  The resolution does not particularly matter.
  The only requirement is that it has a standard sized HDMI input jack.

* Micro-HDMI to regular HDMI adapter cable (for keyboard/display based configuration):
  The Raspberry Pi 4 has 2 micro HDMI connectors.
  This cable adapts between the Raspberry Pi 4 micro-HDMI connector
  and the regular display HDMI connector.

* USB-A Keyboard (for keyboard/display based configuration):
  This is needed to type commands into the Ubuntu console.

* RJ45 Ethernet Cable (for "headless" based configuration):
  It is counter-intuitive, but this is very useful for bringing up the Raspberry Pi 4 WiFi.

The list above should be a pretty complete list of the required additional hardware.

### Development Computer Ubuntu 20.04 Install

The development computer is assumed to be running Ubuntu 20.04.
It is beyond the scope of this document explain how to download Ubuntu 20.04.

You may do one of the following:

* Purchase a laptop with Ubuntu preinstalled.

* Install Ubuntu 20.04 onto an 64-bit x86 processor yourself.

* Install Ubuntu 20.04 to run under a virtualization environment
  like (VirtualBox)[https://www.virtualbox.org/].)
  If you use VirtualBox, be sure to configure the Ethernet adapter to be in "bridged mode".
  Other than that, just follow the lengthy installation instructions.

### Development Computer ROS2 Install

Once you have Ubuntu 20.04 installed, please install the ROS2 version named
[Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/) which configured for Ubuntu 20.04.
Do a full desktop install.

### Development Computer Configuration:

This is where various specialized configurations occur.
For now there is only one:

* [Install `usbip` Client Service](#install-usbip-client-service):
  The `usbip` protocol is used to allow the development computer to connect to the ST-Link
  on the robot micrcontroller.

#### Install `usbip` Client Service

The robot microcontroller has an ST-Link system on it that is used for firmware download and debug.
The robot microcontroller firmware is developed on the development computer.
The robot computer provides a bridge between the ST-Link on the robot microcontroller
and the development computer as shown immediately below:

                            IP                    USB
     Development Computer <====> Robot Computer <=====> Robot Microcontroller
        (usbip client)            (usbip host)

The protocol is called `usbip` for USB Internet Protocol.
This protocol enables the access to USB devices on other computers as if they are local.

<!-- Are the correct packages already installed? -->

Create a systemd service file:

     sudo -s   # switch over to super user
     cat <<EOF > /etc/systemd/system/usbip-client.service
     [Unit]
     Description=USB/IP Client Modules

     [Service]
     Type=oneshot
     RemainAfterExit=true
     ExecStart=/sbin/modprobe -qab vhci-hcd

     [Install]
     WantedBy=multi-user.target
     EOF

     exit # Exit super user mode

     systemctl enable usbip-client.service
     systemctl start usbip-client.service 
     systemctl status usbip-client.service  # Should be OK

     # Verify that it worked:
     lsmod | grep vhci
     # Should list two lines

### Development Computer STM32CubeIDE Install

There are multiple cross platform IDE's (Integrated Development Environment) out there.
The `STM32CubeIDE` IDE is selected because it is directly supported by microcontroller vendor
(i.e ST Microelectronics.)
There are others (e.g. `vscode`, `Keil`, etc.) that could work as well,
but they are not directly supported by STM.

Please note that these instructions are for STM32CubeIDE 1.5.x.
The 1.6.x version of STM32CubeIDE has been released and these install instructions
need to be updated.

Please do the following:

1. Uninstall Notes:

   Note that `STM32CubeIDE` is a big complicated program
   that feels obligated to create a bunch of sub-directories in your home directory.
   At this point in time, the following sub directories are created:
 
   * `.stm32cubeide/`:
     It seems to just contain `favorites.mcus.txt`.

   * `.stm32cubemx/`:
     Some `plugins/` and `thirdparty/` software.

   * `.stmcube/`:
     Empty!

   * `.stmcufinder/`:
     Some `plugins/`.

   * `st/`:
     This s a sub-directory that contains the actual unpacked IDE files
     from the download `.zip` file (see below.)

   * `STM32Cube/`:
     This is a huge sub-directory contains many gigabytes of files.
     It sometimes downloads new updates that essentially duplicated the contents
     of the `st/` directory.

   * `STM32CubeIDE`:
     This is the default sub-directory for your workspace (see further below.)

   If you ever need to clean `STM32CubeIDE` off your computer, delete all of these sub-directories.
   With the exception of the `st/` sub-directory,
   all of the other directories are created the first time you run `STM32CubeIDE`

2. Notation:

   Downloading the `STM32CubeIDE` is pretty involved.
   In the instructions below, the following notation in all capital letters shows up:

   * USER:
     This is your user name (e.g. `alice` for `/home/alice`).

   * PASSWORD:
     This is your login password.  It is needed for the `sudo` program.

   * X:
     The major version of the `STM32CubeIDE`.

   * Y:
     The minor version of the `STM32CubeIDE`.

   * D:
     Decimal digits that changes from release to release.

3. Enable `sudo` command:

   Run the `groups` command and verify that you are in the `sudo` group.
   If `sudo` is not present, you are not `sudo` enabled.
   Please visit
   [How to Add User to Sudoers](https://phoenixnap.com/kb/how-to-create-sudo-user-on-ubuntu).
   You can probably skip step 1, since you probably already have a user account.

4. Create STM Account:

   Visit the
   [STMe2CubeIDE download page.](https://www.st.com/en/development-tools/stm32cubeide.html).

   Basically, you ***MUST*** create an account with ST before you can get the software.
   Create your account now, if you do not already have one.
   Fill in the forms and wait for the account to happen.
   STM will offer the you the wonderful opportunity to let them send you lots of marketing,
   but they also give you the opportunity to decline this generous offer.

5. Download and Save `.zip` File:

   Now that you have the STM account, STM will let you download the software.
   Revisit,
   [STMe2CubeIDE download page.](https://www.st.com/en/development-tools/stm32cubeide.html),
   and click on the `[Get Software]` button for the `STM32CubeIDE-Lnx` Generic Linux installer.
   The instructions below are for the Generic Linux installer, **NOT** the Debian installer.

   Eventually, you will get your hands on a `.zip` file with a long ugly name
   that looks basically like this:

     en.st-stm32cubeide_1-X-Y_DDDD_DDDDDDDD_DDDD_amd64_sh.zip

   Please store the `.zip` file someplace, just in case you need to reinstall in the future.

6. Unzip the Downloaded `.zip` file:

   Now, unzip the file using the command immediately below:

     unzip -x en.en-st-stm32cubeide_1-X-Y_DDDD_DDDDDDDD_DDDD_amd64_sh.zip

   This should result in a `.sh` (shell) file that looks kind of like:

     st-stm32cubeide_1.X.Y_DDDD_DDDDDDDD_DDDD_amd64.sh

   The version information is always changing, so the X, Y, and D values change regularly.
   This is the unzipped installer file.

7. Execute the Installer script file: 

   It is going to ask you agree to a whole bunch of license agreements,
   install stuff in places, etc.
   There are also a few steps that need to be done in super user mode.

   Now execute the install script file:

     sh st-stm32cubeide_1.X.Y_DDDD_DDDDDDDD_DDDD_amd64.sh

   Roughly speaking the following output occurs.
   It will change over time, so do not be concerned if it looks a little different.
   You input response are marked with `<== ...`:

        Verifying archive integrity... All good.
        Uncompressing STM32CubeIDE 1.X.Y installer  100%
        STMicroelectronics Software License Agreement
        ...   <== Big long license agreement.  Type [Space] to scroll through it.
        I ACCEPT (y) / I DO NOT ACCEPT (N) [N/y]                        <== Type 'y'
        License accepted.
        Do you want to install STLinkServer and Udev rules for STLink and Jlink?
        Without these packages, you will not be able to use debugging feature.
        Install them? (you need sudo permission to do that) [Y/n]       <== Type 'Y'
        STM32CubeIDE install dir? [/home/HOME/st/stm32cubeide_1.X.Y/]   <== Type [Enter]
        Installing STM32CubeIDE into ...
        Java cacerts symlinked to /etc/ssl/certs/java/cacerts
        [sudo] password for USER:                                       <== Type your PASSWORD
        Creating directory root
        Verifying archive integrity... All good.
        Uncompressing STM udev rules installer  100%
        STMicroelectronics Software License Agreement
        ...   <= Big long license agreement.  Type [Space] to scroll through it
        I ACCEPT (y) / I DO NOT ACCEPT (N) [N/y]                        <== Type 'y'
        STMicroelectronics Software License Agreement
        ...   <== Another long license agreement.  Type [Space] to scroll through it
        I ACCEPT (y) / I DO NOT ACCEPT (N) [N/y]                        <== Type 'y'
        License accepted.
        stlink-server v2.Z.Z (ZZZZ-ZZ-ZZ-ZZ:ZZ) installation started.
        ...  <== More text that can be ignored.
        Do you want to install Segger J-Link udev rules? [Y/n]          <== Type 'n'
        STM32CubeIDE installed successfully

8. Put `stm32cubeide` in your PATH:

   Using your favorite text editor (e.g. `nano`),
   add the following to your `~/.bashrc` file near the bottom of the file:

     export PATH=$PATH:$HOME/st/stm32cubeide_1.X.Y/stm32cubeide

   Remember to use the correct values for X and Y.
   Now `source` the `~/.bashrc` file as follows:

       source ~/.bashrc

   The command:

        which stm32cubeide

   should return `/home/USER/st/stm32cubeide_1.X.Y/stm32cubeide`.
   If it does, you have successfully installed stm32cubeide.

It program is run for the first time further below in the
[Firmware Development](#firmware-development) section.
That concludes `STM32CubeIDE` software installation.

### Robot Computer Ubuntu Installation

This section is broken into the following sub-sections:

* [Command Line Notation](#command-line-notation):
  This briefly discusses the command-line notation rules used.

* [Bring Up Strategy](#bring-up-strategy):
  This discusses the two initial bring up strategies.
  You get be pick between display/keyboard bring up vs. headless bring up.

* [Install Ubuntu onto a Micro-SD Card](#install-ubuntu-onto-a-micro-sd-card):
  A program called `rpi-image` isntalled onto your development computer and
  is used to load the Ubuntu image for the Raspberry Pi4 onto a micro-SD card.

* [Raspberry Pi 4 Connector Locations](#raspberry-pi-4-connector-locations):
  This section describes the names and locations of the various Raspberry Pi 4 connectors.

* [Initial Bring Up](#initial-bring-up):
  The Raspberry Pi 4 is powered up to the point that it can be logged into.
  This is easy if you have a keyboard and display plugged into the Raspberry Pi 4
  and it is significantly more difficult if you are not.

* [Login and Configuration](#login-and-configuration):
  This takes you from login up to the point where is ready to do actual software development.

* [Remote USB Installation](#remote-usb-installation)
  Remote USB is used to provide access to the ST-Link on the robot mircocontroller.
  Code must be installed on both the robot computer (and the development computer.)

#### Command Line Notation

Most of the configuration is done via typing in commands to a console interpreter
(i.e. a Linux shell.)
When you see block of code like:

     # Example comment
     echo hello
     # `hello` should show up.
     
Just type in the code one line at a time to the console.
All characters after from sharp character (`#`) onward are a comment that does not need to be typed in.

Also, you will see some upper-case keywords in the scripts below.
You should substitute the values you choose for these keywords into the scripts below.
For example, when you `NEWHOSTNAME` you substitute in the new host name that you selected
(e.g. `myrobot`, `fun4me`, etc.)

These are the current upper-case substitutions:

* `NMAP_IP_RANGE`:
  This is a special internet protocol address that is used by the `nmap` program.
  This is for the headless install option only.

* `RPI4_NET_ADDRESS:
  This is a 4 decimal numbers separated by periods (`.`).
  This is the initial internet address of the robot.
  This instruct

* `NEWHOSTNAME`:
  The original host name provided by the downloaded Ubuntu image is `ubuntu`.
  This needs to be changed.

* `NEWUSERNAME`:
  This is a new user account name that you select.
  Many people just use their first name (e.g. `alice`, `bob`, `carol`, `dave`, etc.)

#### Bring Up Strategy

For the HR2, the nominal computer for the Robot Computer is a Raspberry Pi 4 with at least 4GB.
If necessary,
reread the [introduction](#introduction) to refresh your memory of what the robot computer is.

There are two installation strategies:

* Display/Keyboard:
  The display/keyboard method is the easiest, but it requires:

  1. Keyboard:
     A Keyboard with a USB-A connector is plugged into one of the USB ports on the Raspberry Pi 4.

  2. HDMI display:
     While resolution does not matter, having the HDMI input connector is required.

  3. Micro HDMI to regular HDMI cable:
     This cable adapts between the Raspberry Pi 4 and the HDMI cable.

* Headless:

  A computer without a keyboard/display is called "headless".
  It can only be administered via a network connection.
  For this method, the Ethernet cable is temporarily required.
  One end of the Ethernet cable is plugged into the Raspberry Pi 4 and
  the other end is plugged into your local area network.
  This is done with a RJ45 cable.
  If you normally use WiFi, you may have to search for the RJ45 connector on your cable modem.
  If you can not find a live RJ45 connector to plug into, you must use the display/keyboard method.

#### Install Ubuntu onto a Micro-SD Card

The instructions here are based on 
[Ubuntu Install](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4).
Since web pages change all the time, the steps are summarized below:

1. Install `rpi-imager`:
   Run the following command to install `rpi-imager`:

     `sudo apt install -y rpi-imager`

2. Install Micro-SD Card into the computer:
   Install the 32GB (or larger) micro-SD into the micro-SD card adapter.
   Insert the micro-SC card adapter into your development computer.
   If you are running Ubuntu under a virtual machine (i.e. VirtualBox).
   it may be necessary to import the micro-SD card into the virtual environment.
   Once the card is installed, dismiss any pop-up windows that show up.

3. Run `rpi-imager`:
   Type `rpi-imager` and a window should pop with 3 buttons on it.
   The first two buttons are `[Choose OS]` and `[Choose SD Card]`.
   The third button is a grayed out `[Write]` button.
   
5. Choose OS:
   These windows change from time-to-time, so the instructions below may get a little out of date.

   1. Click on `[Choose OS`].

   2. Select `[Other general purpose OS]`.

   3. Select `[Ubuntu]`.

   4. Scroll down to `Ubuntu Server 20.04.X LTS (RPI 3/4/400)`.
      This is the `64-bit server OS with long term support for arm64 architectures`.

6. Choose SD Card:

   It is extremely likely that only one card will be available.
   A 32GB micro-SD card might show up as 31.9 GB.
   Click on it.

7. Write the Card:

   The `[Write]` button should no longer be grayed out.
   Click on it.
   Click on `[YES]` in the pop-up.
   It may ask for your password, type it in and click `[OK]`.
   You may click on `[CANCEL WRITE]` at any time, but you will have to start all over again.
   This should take a few minutes.

8. Wait a while for the download to occur:
   While waiting, please remember that there are additional steps required,
   so do not unplug the micro-SD card adapter when you are done.
   When the `rpi-imager` is done, it will say `remove the SC card from the reader`.
   Do not remove the adapter,
   but do click on `[Continue]`

9. Close `rpi-imager`:
   Dismiss the `rpi-imager` program by click on the `[X]` in the upper right corner.

10. Remove the micro-SD card.

#### Raspberry Pi 4 Connector Locations

Before getting started please familiarize yourself with the locations of Raspberry Pi 4 connectors.

First, orient the Raspberry Pi 4 so that the 40-pin connector (2 rows of 20 pins)
is on top and to the left.

Now find the following connectors:   

   * RJ45 Ethernet connector:
     This is located on the right edge in the upper right.

   * 4 USB type A connectors:
     These are located on the right edge under the RJ45 ethernet connector.
     There are two USB connector in each of the two silver "bricks" under the RJ45 connector.

   * USB C Power Connector:
     The USB C connector is on the bottom edge to the far left.

   * 2 Micro HDMI Connectors:
     On the bottom edge, the two micro HDMI connectors are to the right USB C power connector.

   * 1 Micro-SD Card connector:
     On the left edge, but underneath the board you find the micro-SD card.

   * Red Power LED:
     There is a red power LED near the USB C Power Connector.
     It is the very small component on the left edge at the very bottom
     nearest the bottom left mounting holes
     You can not tell that it is red by looking at it.
     Just be aware that this component will light just as soon as power is applied
     to the Raspberry Pi 4.

   * Green Activity LED:
     There is a green activity LED immediately above the red power LED.
     Again, you can not tell that it is green.
     All you need to know is that it is there and turn on and off as various activies
     occur on the Raspberry Pi 4.

As a further note, there is no on/off switch on the Raspberry Pi 4.
What this means that as soon as the "power cable" is connected
between the battery pack and the Raspberry Pi 4, the Raspberry Pi 4 will probably start.
Do not plug in any power cable until explicitly told to do so in the instructions below.

While the Raspberry Pi does not have an on/off switch,
some battery packs do have an on/off switch.
So depending upon your battery pack, you may have to toggle a batter power switch to provide power.
Once power is applied, the Red Power LED will light up immediately and
the green activity LED will start to randomly turn on and off as the Ubuntu operating system powers up.
In addition, if a display is both powered up and connected,
it will start to display text on power up.

Please reread section on [keyboard/display vs headless install](robot-computer-ubuntu-installation).
Please follow the instructions for either
[Keyboard/Display Installation](#keyboard-display-installation) or
[Headless Installation](#headless_installation).

#### Initial Bring Up

This is where the initial Raspberry Pi 4 power up occurs.
This takes you as far as the `login:` prompt.

Perform the instructions below depending upon which [bring-up strategy](#bring-up-strategy) you selected:

* [Keyboard/Display Bring Up](#keyboarddisplay-bring-up)
  Perform these instructions for the keyboard/display power-up method.

* [Headless Bring-up](#headless-bring-up):
  Perform these instructions for the headless bring-up method.

##### Keyboard/Display Bring-up

For keyboard/display installation do the following:

1. Plug in keyboard:
   Plug your keyboard into one of the USB type A connectors.

2. Plug in micro HDMI-Cable:
   Plug one end of the micro-HDMI cable into either one of the Raspberry Pi 4 micro-HDMI connectors.

3. Plug into the HDMI Display:
   Plug the other end of the micro-HDMI cable into your HDMI display.

4. Plug micro-SD Card:
   Plug in the micro-SD card

4. Display Power Up:
   Turn on the HDMI display.

5. Before Power Up:
   Remember, once the power cable is connected,
   the Raspberry Pi will power up immediately since there is no on/off switch.
   
6. Connect Power Cable and Power Up:
   Make sure you have a cable that is compatible between your battery pack and
   the USB-C connector on the Raspberry Pi 4.
   Your battery pack probably came with one, so plug it in and watch the show.

7. When your display shows something with something like `ubuntu login:` it means you are ready to login
   and start configuring the Raspberry Pi.
   You can skip the section immediately below on headless install.

Proceed to the [Initial Bring up](#initial-bring-up) section.

##### Headless Bring-up

This section concerns headless install which is significantly more complicated than the headless 
The overall goal of a headless install is to login into the Raspberry Pi 4 remotely
via the internet.

What makes headless installation harder is that when the Raspberry Pi connects to the internet
for the very first time is it will ask the network to give it an internet address
via a protocol call DHCP -- Dynamic Host Configuration Protocol.
We need to figure out what address was assigned by DHCP
*before* you can log into the Raspberry Pi 4 from the development computer.

<!-- link to `ip` info: https://packetpushers.net/linux-ip-command-ostensive-definition/ -->

Please perform the following steps:

1. Install the following packages on your development computer:

   Install the following packages onto your computer:
     sudo apt install -y iproute2               # Get `ip` command
     sudo apt install -y nmap                   # Get `nmap` local net scanner.
     sudo apt install -y net-tools              # Get `arp` command, ping, ifconfig, etc.
     sudo apt install -y libnss-mdns mdns-scan  # For avhi (i.e. zero-conf)

   For you information:
   * The `ip` command is used to flush something called the ARP cache.
     ARP stands for Address Resolution Protocol.
   * The `nmap` command scans your local internet looking for active internet addresses.
   * The `arp` command lists all of the active internet addresses.
   * The `libmss-mdns` and `mdns-scan` packages provide functionality loosely called
     [zero-configuration networking](https://en.wikipedia.org/wiki/Zero-configuration_networking)
     or "zero-conf" for short.
     This allows us to login into a computer using its host name instead of a bunch of numbers.

2. Verify zero-conf is working:

   Run the following command:

     ping -c 3 `hostname`.local  # Note that the accent ("`") is used instead of the single quote ("'").
     
   It should print out a few lines as follows:
   
     PING ??? (192.168.xxx.yyy) ...

   There are three internet address ranges that are reserved for local area networks.
   * Class A: 192.168.yyy.zzz where yyy is normally 1 (i.e. 192.168.1.zzz).
   * Class B: 169.124.yyy.zzz.
   * Class C: 10.xxx.yyy.zzz, where xxx is typically 1 (i.e. 10.1.yyy.zzz).
   * Something else: These headless instructions will not work;
     use a keyboard/display install.

   For the `nmap` command, you will need to specify an address space range to scan.
   Please use the following:
   * Class A: 192.168.yyy.0-255 where yyy is the number from you ping.
   * Class B: 169.124.0-255.0-255 .
   * Class C: 10.xxx.0-255.0-255  where xxx is from your class C network.
   This is called NMAP_IP_RANGE below:

   These instructions will usually work for class A and class B networks,
   but will frequently fail for class C networks.

3. Determine the currently used internet addresses:

   Run the following commands:
   
        sudo ip neigh flush all     # Flush arp cache *BEFORE* powering up RPi4
        sudo nmap -p ssh -n NMAP_IP_RANGE  # Class A is ~10 seconds, others are about ~5 minutes.
        arp > /tmp/before.arp

4. Connect network cable:
   Plug one end of your RJ45 cable into the raspberry Pi and the other into your local network.
   
5. Before Power Up:
   Remember, once the power cable is connected,
   the Raspberry Pi will power up immediately since there is no on/off switch.
   
6. Connect Power Cable and Power Up:
   Make sure you have a cable that is compatible between your battery pack and
   the USB-C connector on the Raspberry Pi 4.
   Your battery pack probably came with one, so plug it in and watch the show.

7. Wait for 2 minutes:
   After these 2 minutes the Raspberry Pi should be up and running.

8. Rescan for currently used internet addresses:
  
   Run the following commands:
   
        sudo ip neigh flush all   # Flush arp cache *BEFORE* powering up RPi4
        sudo nmap -p ssh -n NMAP_IP_RANGE  # Class A is ~10 seconds, others are about ~5 minutes.
        arp > /tmp/after.arp     # Note this time `after.arp` is being used instead of `before.arp`

9. Find the new internet address:

   Run the following command:

        diff /tmp/before.arp /tmp/after.arp

   This should have on line that shows up that is the internet address of your Raspberry Pi 4.
   Call this RP4_NET_ADDRESS

10. Make sure Raspberry Pi 4 is alive:

         ping -c 3 RP4_NET_ADDRESS

    If you gets some output.
    If not, the RP4_NET_ADDRESS you have is not connected to your Raspberry Pi 4 computer

11. Use `ssh` to log into your Raspberry Pi 4:

    Run the following command.

         ssh ubuntu@RP4_NET_ADDRESS

    It might probably complain about a fingerprint issue.

    Please use the "yes" option.

    Sometimes it will complain about `REMOTE HOST IDENTIFICATION HAS CHANGED!`
    It will printout something:

        remove with:
	ssh-keygen -f "/home/YOUR_ACCOUNT/.ssh/known_hosts -R "AN_IP_ADDRESS"

    Just run the command and try the `ssh ubuntu@RPI_NET_ADDRESS` again.

12. Wait for login prompt:

    If everything worked right, you should get a login prompt
    
        ubuntu login:

    Headless login appears to have largely succeeded.

    It is now time to proceed to the next section immediately below.

#### Login and Configuration

This section takes you from the `login:` prompt to more complete Raspberry Pi configuration.

The basic steps are as follows:

1. [Change the Login Password](#change-the-login-password):
   The initial password is `ubuntu`, but the system forces you to change it to something else.

2. [Change the Host Name](#change-the-host-name):
   It is important the change the host name from `ubuntu` to something else.

3. [Create a New User Account](#create-a-new-user-account):
   It is useful to create a new user account with `sudo` access.

4. [Set Up Zero Configuration](#set-up-zero-configuration):
   This allows you to be able to connect to the robot with the robot name.

5. [Login into Robot from Development Computer](#login-into-robot-from-development-computer):
   Now you should be able to login to the computer from your development computer.

6. [Configure Wifi](#configure-wifi):
   Configure your WiFi access.

7. [Remote USB Installation](#remote-usb-installation):
   This configures a system call `usbip`
   for remote access to the ST-Link device on the Robot Microcontroller.

8. [Final Configuration Notes](#final-configuration-notes):
   Some final comments on configuration.

##### Change the login password:

When you first login, the system will insist that you change the password for the `ubuntu` account.
It will prompt you for the initial password (`ubuntu`) first, followed by your new password two times.
The reason for prompting for the new password two times is just in case there is a typo.
Upon success, the Raspberry Pi 4 will disconnect.
You have successfully changed the password.

You will have to login again.
On for the headless bring-up method,
this will require that you run the `ssh ubuntu@RPI_NET_ADDRESS` command again.
It should prompt for a password and just type in the new password.

When you are done with this step the password should be changed and
you should be logged into the Raspberry Pi 4.
You should get prompt that looks like `ubuntu@ubuntu:~$`.

##### Change the Host Name:

Right now you host name should be `ubuntu`.

If you type `hostname`, it should return `ubuntu`.
This verifies that the host name is still `ubuntu`.

It is important to change the host name to something other than `ubuntu`.
The reason why is because it is necessary to get each robot a unique name
to make it easier to connect to it.
If all robots were named `ubuntu`, you would have to keep typing the annoying internet address
(e. g. 192.168.1.yyy`) instead.

Please come up with a fun new host name.
It should be lowercase letters and digits only, with no punctuation
(`alice`, `robot4all`, `go2dance`, etc.)
In the command below `NEWHOSTNAME` is used in the commands,
please substitute your lowercase letters and digits for `NEWHOSTNAME` below:

Run the following commands:

     sudo hostnamectl set-hostname NEWHOSTNAME
     # The prompt should change to `ubuntu@NEWHOSTNAME:$`
     hostname
     # You should get back `NEWHOSTNAME`
                
##### Create a New User Account:

You are welcome to continue using the `ubuntu` account the Raspberry Pi 4,
but most people try to create their own personal accounts.
Most people make their account names all lower case letters with no numbers or punctuation
(`greatfun`, `fred`, etc.)

In the script below, substitute your new account name for `NEWUSER`:
   
     sudo adduser NEWUSER
     # Type in new password once
     # Type in the same password again
     # Fill in the remaining information.
     # Type `Y` when it asks `Is the information correct?

Give the new account super-user capability with the following command:

     sudo usermod -aG sudo NEWUSER

<!--
Issue: It did not set default directory to /home/NEWUSER .
-->

Now verify the user account with the commands below:

     su NEWUSER
     # Type in password
     # The prompt should change to `NEWUSER@NEWHOSTNAME@:~$`
     whoami
     # It should respond with NEWUSER
     sudo echo hello
     # It should prompt for the password again follow by `hello`
     exit

##### Set up zero configuration.

Zero configuration is a way to export your host name make it easily accessible
from you development computer.
This means you will no longer have to type in obscure numbers.

First, install some zero configuration packages with the command below:

     sudo apt install -y net-tools libnss-mdns mdns-scan

Now, verify that it works with the following command:

     ping -c 3 NEWHOSTNAME.local
     # You should get back several lines of code.

Now logout as follow:

     exit  # Force a logout

##### Login into Robot from Development Computer

It should be very easy to log into robot computer from your development computer:

     ssh NEWUSER@NEWHOSTNAME.local
     # It may complain about authenticity problems.
     # If so, run `ssh-keygen -f ...` command and try the `ssh NEWUSER@NEWHOSTNAME.local` again.
     # If it complains about fingerprints, type `Y`.
     # Type in the password.
     # You should get a prompt of `NEWUSER@NEWHOSTNAME@hr2b

<!-- Consider doing a `sudo apt update && sudo apt upgrade` -->

##### Configure Wifi

The next step requires access to a WiFi router connected to your local network.

<!-- https://kifarunix.com/connect-to-wifi-in-linux-using-nmcli-command/  -->

1. Install network manager:

        sudo apt install -y network-manager   

   You now have a program named `nmcli` (Network Manager Command Line Interface).

2. Verify WiFi is enabled:

   To verify that the wifi is enabled do the following:

        nmcli radio wifi
        # It should come back with `enabled`.
        # If `disabled` run `nmcli radio wifi on`

3. Search for WiFi access points with the the following command:

        nmcli dev wifi list
        # A list of WiFi access points should show up.

   The name of the access point is called an `SSID`.
   Find the SSID of the WiFi access point to connect to.
   In the scripts, below substitute the correct name for SSID.

4. Register a Wifi access point for logging in with the following command:

       sudo nmcli --ask dev wifi connect SSID
       # Type in your `sudo` password if prompted.
       # Type in WiFi password after the password prompt.

5. Verify active connection with the following command:

       nmcli con show --active

  You should see one active connection.

6. Run `ifconfig` (Interface Configure):

       ifconfig

   You should get network status for `eth0`, `lo`, and `wlan0`.
   `eth0` is the hardwired ethernet.
   `lo` is a  LOopback pseudo interface.
   `wanl0` is the wireless (WiFi) interface.
   Just in case, write down `inet` address.
   You can ignore the `netmask` and `broadcast` fields.

7. Reboot and Unplug Ethernet Cable with the following commands:

       sudo reboot -h now  # Force an immediate reboot
       # Now unplug the ethernet cable (for the headless bring up strategy)
       # Wait a for a couple of minutes

8. From your development computer, verify that the robot computer logged into the WiFi access point
   with the following command:

       ping -c 3 NEWHOSTNAME.local
       # You should get 3 ping responses

9. Login/logout to/from the robot computer with the following commands:

       ssh NEWACCOUNT@NEWHOSTNAME
       # Type in password for prompt
       # When logged in shut down Raspberry Pi 4
       sudo halt
       # Type in password for prompt
       # The robot should turn off


##### Remote USB Installation

The robot microcontroller has an ST-Link system on it that is used for firmware download and debug.
The robot microcontroller firmware is developed on the development computer.
The robot computer provides a bridge between the ST-Link on the robot microcontroller
and the development computer as shown immediately below:

                            IP                    USB
     Development Computer <====> Robot Computer <=====> Robot Microcontroller
        (usbip client)            (usbip host)

A system called `usbip` is used to provide remote access to USB devices.
A `usbip` client is installed on the development computer and a `usbip` host is installed
on the robot computer.
When `usbip` properly installed, the development computer accesses the remote ST-Link subusystem
as if it was locally plugged into the development computer.

This section discusses how install `usbip` on the robot computer.
It is broken into the following sections:

* [Install `usbip` Packages](#install-usbip-packages)

* [Install `usbip` Service](#install-usbip-service)

* [Verify `usbip` Installation](#verify-usbip-installation)

###### Install `usbip` Packages

Install the following packages:

     sudo apt install -y linux-tools-common
     sudo apt install -y linux-tools-raspi
     sudo apt install -y usbutils
     sudo apt install -y linux-cloud-tools-common  # Not sure about this one (seems to help)

Install the following symbolic link:

     sudo ln -s /usr/share/misc /usr/share/hwdata  # Only need to do once

The symbolic link deals with the fact that the USB ids file got moved from `/usr/share/hwdata`
to `/usr/share/misc`.
It is much easier to fix this problem with a symbolic link than it is to fix the code.

Once you have installed the packages and symbolic link, verify that everything is installed.

    which usbip   # Should return `/usr/bin/usbip`   # This is the `usbip` user level program
    which usbipd  # Should return `/usb/bin/usbipd`  # This is the `usbip` daemon.

###### Install `usbip` Service

The Robot computer needs to do three things on start-up.

1. Load a couple of kernel modules -- `usbip-core` and `usbip-host`.

2. Start the `usbipd` daemon.

3. Find the ST-Link and make it available for remote access.

This is done with two files:

* `/etc/systemd/system/usbip_host.service`:
  This is a `systemd` unit file that properly installs the kernel modules
  and runs the daemon.
  It also invokes the `usbip_host.service.sh` shell file.

* `/usr/local/bin/usbip_host.service.sh`:
  This determines if an ST-Link plugged into one of the robot computer USB ports and
  (if present) bind it such that it is available for external access.
  
The is installed as super user:

     sudo -s  # If prompted for a password, type it in.
     # Prompt character changes from `>` to `#` to remind you that your are super-user

Now the `/etc/systemd/system/usbip_host.service` is installed as follows:

     cat <<EOF > /etc/systemd/system/usbip_host.service
     [Unit]
     Description=usbip host daemon

     [Service]
     ExecStartPre=/usr/sbin/modprobe usbip-host
     ExecStart=/usr/bin/usbipd
     ExecStartPost=/usr/local/bin/usbip_host.service.sh
     
     [Install]
     WantedBy=multi-user.target
     EOF

Now the `/usr/local/bin/usbip_host.service.sh` file is installed as:

     cat <<EOF > /usr/local/bin/usbipd_host.service.sh
     #!/usr/bin/bash
     log_file=/tmp/usbip_bind.log
     echo "/usr/local/bin/usbipd.servce.sh started" > "$log_file"
     /usr/bin/usbip list -p -l 2>&1 >> "$log_file"
     st_link_bus_id="$(/usr/bin/usbip list -p -l)"
     echo "st_link_bus_id1=$st_link_bus_id" >> "$log_file"
     st_link_bus_id=`echo $st_link_bus_id | /usr/bin/grep 0483`
     echo "st_link_bus_id2=$st_link_bus_id" >> "$log_file"
     st_link_bus_id=`echo $st_link_bus_id | /usr/bin/sed s,busid=,,`
     echo "st_link_bus_id3=$st_link_bus_id" >> "$log_file"
     st_link_bus_id=`echo $st_link_bus_id | /usr/bin/sed s,#.*,,`
     echo "st_link_bus_id4=$st_link_bus_id" >> "$log_file"
     if [ "$st_link_bus_id" ]
     then
         echo "attempting bind" >> "$log_file"
         /usr/bin/usbip bind -b "$st_link_bus_id" 2>&1 >> "$log_file"
         echo "bind attempted" >> "$log_file"
     fi
     echo "/usr/local/bin/usbipd_host.servce.sh ended" >> "$log_file"
     EOF

Set the execute bit:

     chmod +x /usr/local/bin/usbipd.service.sh

Enable the usbip host service:

     systemctl enable usbipd_host.service
     systemctl start usbipd_host.service

This file has a lot of logging code that is used to figure if something went wrong.
The `echo` statements are used for debugging only.
What this file does is list the USB devices and their associated bus ids.
The ST-Link device is found (USB id `0483`) and the bus id is extracted.
If the bus id is found, the ST-link is bound by `usbip` and it is available for remote use.

Now exit super-user mode:

     exit

###### Verify `usbip` Installation

Verify that both files exist:

     ls -l /etc/systemd/system/usbip_host.service  # Should print a one line directory listing
     ls -l /usr/local/bin/usbipd_host.service.sh   # Should print a one line directory listing

Once you have installed both of these files you need to reboot and verify that everything is working:

     sudo reboot -h now

You will have to log in again.

Verify that the kernel modules are loaded:

     lsmod | grep usbip-host  # This should produce two lines

Verify that the `usbipd` daemon is listening on port 3242:

     netstat -tnlp | grep 3240  # Verify daemon is runnning, this should list at least one line


Plug a cable between one of the the Robot computer USB connectors and the ST-link connector
(the smaller of the 2 boards on Nucleo.)

Run the `usbip` command:

     usbip list -l  # List various local (i.e. `-l`) USB ports

If it shows up with something like:

      - busid 1-1.1 (0483:374b)
        STMicroelectronics : ST-LINK/V2.1 (0483:374b)

You have succeeded.

<!-- USBIP Notes:

`usbip` is a protocol for connecting 

There are USBIP two articles that the information below is derived from:

* [Linux Magazine](https://www.linux-magazine.com/Issues/2018/208/Tutorial-USB-IP)

* [Stack Overflow](https://unix.stackexchange.com/questions/528769/usbip-startup-with-systemd)

* [RPi](https://community.home-assistant.io/t/rpi-as-z-wave-zigbee-over-ip-server-for-hass/23006)

* [Cern CentOS usbip](https://clouddocs.web.cern.ch/advanced_topics/share_usb_devices.html)
Some commands:

     # On robot computer:
     # Do once
     sudo apt install linux-tools-common
     sudo apt install linux-tools-raspi
     sudo apt install usbutils
     sudo apt install linux-cloud-tools-common  # Not sure about this one (seems to help)
     sudo ln -s /usr/share/misc /usr/share/hwdata  # Only need to do once

     # Do on reboot:
     # Note that usbipd is in /usr/bin/usbipd  not  /usr/sbin/usbipd
     sudo modprobe usbip-core  # Not needed
     *sudo modprobe usbip-host
     *lsmod | grep usbip
     *sudo usbipd -D  # Start usbip daemon
     *netstat -tnlp | grep 3240  # Verify daemon is runnning
     usbip list -l  # Should list usb ports
     sudo usbip bind -b BUS_NAME  # 1-1.3 (for example)

     # On development computer:
     *sudo modprobe vhci_hcd
     *sudo modprobe usbip_core
     sudo usbip list -r hr2b.local  # Should show avail devices (there is only one exported for now)
     sudo usbip attach -r hr2b.local -b BUS_NAME
     lsusb | grep -i stm

     # Detaching
     sudo usbip port  # lists ports in use
     # Prints out each port binding.  
     sudo usbip detach -p 00  # Note that the port number is a string, not a number
     lsusb | grep -i stm   # no match

     # Unbind on the robot computer (Really quite optional)
     sudo usbip unbind -b 1-1.3 # (for example)

-->

##### Final Configuration Notes

In general, configuration basically never ends.
But this is a reasonable place to stop for now.
The next major step is to install ROS2 on the robot computer

### Robot Computer ROS2 Installation

The intial installaton of ROS2 is pretty straight forward.
The steps are:

1. [Connect to ROS2 PPA](#connect-to-ros2-ppa):
   The ROS2 organization maintains its own PPA (Personal Package Archive).

2. [Download ROS2 Foxy](#download-ros2-foxy):
   The latest ROS2 release is called "Foxy".

3. [Final ROS2 Configuration](#final-ros2-configuration):
   The final ROS2 configuration is pretty simple.

#### Connect to ROS2 PPA

The ROS2 instructions for connecting to the ROS2 PPA are a little more involved
than the instructions immediately below.
The insturctions use `apt-key` rather than `curl`:

     sudo apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
     sudo apt-add-repository http://packages.ros.org/ros2/ubuntu

#### Download ROS2 Foxy

Next install foxy:

     sudo apt install -y ros-foxy-ros-base
     sudo apt install -y  python3-argcomplete
     sudo apt install -y python3-colcon-common-extensions

#### Final ROS2 Configuration

Next some modifications are made to make ROS2 more permanent:

     source /opt/ros/foxy/setup.bash
     echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

## Firmware Development

Firmware development is a big topic.
This document only gets you to the point where you can download and existing program
and execute it on your microcontroller board (i.e. the NUCLEO-F7676ZI).
The program is called `blinky` and it blinks the LED on the Nucleo board.
A trivial change is done to the main program and the returned back to is initial state.

The `STM32CubeIDE` (Integrated Development Environment) uses the concepts of workspace and project:

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
  Instead, projects are stored in a different location (the HR2 `git` Repository).
  None-the-less, the workspace can work with these projects in various different directories.
  Projects can easily be added and removed from a workspace.

The terms workspace and project are heavily used,
so it is useful to have an idea of what they are beforehand.

Now, please follow the following steps:

1. Start `stm32cubeide`:

   This starts everything rolling.
   `stm32cubeide` should be in your path.
   So, typing `stm32cubeide` in a console window should work.

2. Select a workspace directory:

   `STM32CubeIDE` really wants a workspace.
   So the first thing you will see is a pop-up is window entitled `Select a directory as workspace`.
   The default workspace directory is `/home/USER/STM32CubeIDE/workspace_1.X.Y`.

   You can check `[x] Use this as the default and do not ask again`
   if you do no want see this pop-up again.

   Please click on the `[Launch]` button.
   
3. Main Screen.

   The main screen that pops up is pretty spartan at first.
   It has:

   * Title Bar:
     The standard title is on top.
     The current workspace (not selected yet!) is typically displayed in the title bar.

   * Command Bar:
     There is the standard bar of text commands `File`, `Edit`, ... , `Help`.
   
   * Tab Window:
     There is a tab window has the `[Information Center]` tab showing.
     It humorous to note that this tab is rather non-informative at first.
     Additional tabs are forthcoming.

   * Vertical Icon Bar:
     There is a vertical bar on left that has some inscrutable icons in it.
     You can hover your mouse over the icons to get slightly more descriptive names.

4. Open Project Explorer:

   Find the `[Restore]` icon in the vertical icon bar and click on it.

   Now the `[Project Explorer]` tab should be open.
   
   Click on `[Import projects...]`   

5. Project Import:

   The `[Import]` pop-up shows up.
   It wants you to `Select an import wizard:`.
   Please click just to the left of `[General]`,
   and some additional options should show up.
   
   Select on `[Existing Projects into Workspace]` and click on the `[Next>]` button.
   
6. Import Projects:

   The `[Import Projects]` window pops up.

   Now you get to `(x) Select  root directory` and  click on the `[Browse...]` button.
   Using a file chooser, select the directory `$HR2_DIRECTORY/software`,
   where `$HR2_DIRECTORY` is the directory that contains the local copy of the HR2 `git` repository.

7. Import `blinky`:

   A list of predefined HR2 projects should show up.
   Please click `[Deselect All]` followed by checking off on the one named `[x] blinky`.
   When you click on the `[Finish]` button, this will import the `blinky` project to your workspace.
   You should now have one project in your workspace.

8. Find `main.c`:

   The project explorer should show a `blinky` in it.
   This is a tree explorer.
   Open `blinky1` => `Core` => `Src` until `main.c` is showing.
   Double click on `main.c` to open up the `main.c` file.

9. `main.c` Tab:

    The `main.c` tab should show up.
    Scroll through it until you see the `int main(void)` function (approximately line 70.)
    This is the first function called by the program.

    Scroll down until you find the `Infinite loop` (approximately line 100.)
    You should see 3 `HAL_GPIO_TogglePin()` function calls
    followed by a `HAL_Delay()`.
    
    Eventually, you will edit the `HAL_Delay()` function call, but that is a little further below.


10. Plug in Nucleo Board:

    Now is a good time to plug the Nucleo-767ZI board into your development computer
    using a USB cable with male USB mini connector on it.
    There are two female USB connectors on the Nucleo-767ZI board.
    There is one next to the large RJ45 connector
    and there is one on the opposite end that is for the ST-Link module.
    Plug into the ST-Link module.
    A big red LED on ST-Link module should light up when you plug into it.
    In addition much smaller green power LED on the "main" board will light up.
    The big red LED is actually a bi-color LED that can switch between read and green.
    It changes colors as various things go on.
    
11. Verify Nucleo Board Found:

     When you run `lssub` in a console window, you should see a line that contains
    `STMicroelectronics ST-LINK/V2.1` or something quite similar to it.
    This means that your development computer sees the ST-Link board and
    it should be ready to use.
    
    If not, you have problems and need to figure what went wrong.

12. Download and Run Blinky:

    Now you are going to run the `blinky` program.

    Everything happens pretty fast once you do the steps below.
    The following things happen one after another:

    1. The code is going to be compiled into an downloadable executable.

    2. `STM32CubeIDE` opens a connection to the Nucleo board.
       The ST-Link LED which is normally red, turns green.

    3. While program is being downloaded,
       the ST-Link LED flashes between red and green

    4. The program starts running
       and flashing its green, blue, and red LED's on the main Nucleo board.

    5. The ST-Link LED returns to red it indicate that it is no longer connected.

    There is an icon bar across the top.
    One of the icons looks like a green bug.
    The boring one to the right is the `[Run]` icon
    Without clicking, hover over the `[Run]` button, to see if it shows `[Run main.c]`

    To the right of the button is black pull hierarchical menu triangle.
    The menu item you need to select is two levels deep.
    Everything happens when you release the mouse button.
    Select the following:

       "[Run main.c]` => `[Run As]` => `[STM32/Cortex M C/C++ Application]`.

    Now watch the light show.
    If you missed it the first time around, you can do it again.
    Smile!!! You just ran `blinky`.
    
13. Change Blink Rate.

    Got back to the `[main.c]` tab and find the `HAL_Delay()` function at approximately line 100.
    Edit it from `1500` to `50`.
    The delay is in millseconds, so the LED's will flash much more quickly.

    Do the same process as the previous step, but `[Save and Launch]` pop up window will show up.
    Click on `[OK]` and the new program will download with the slower blinking LED's.

14. Change Blink Rate Back.
    
    Change the `50` back to `1500` and download again.

15. Exit `STM32CubeIDE`.

    Using the `[File]` menu in the upper left, select `[Exit]`.
    The `STM32CubeIDE` should shut down.
    You may also unplug your Nucleo-767ZI board.

That is it for now.

<!--

###################################################################################################

This is all old text that will eventually be deleted.

Developing firmware is done using `STM32CubeIDE`.
`STM` stands for `ST Microsystems` (where `ST` stands for `SGS-Thomson`).
It is unlear what `Cube` stands for, it was probably cooked up by the marketing department.
`IDE` stands for `Integrated Development Environment`.
The program is named `stm32cubeide` and should be in your execution path.

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

## Next steps:

### Microcode download

1. Get usbip working between Desktop and SBC.
2. Force mode probes or use system-D.
3. Connect to IDE.
4. Plug Type-A to Micro cable between RPi4 and Nucleo.
5. Download a blinky from the IDE.

### Debug UART.

1. Configure Nucelo to output characters at 9600?/115200? baud to UART3 (ST-Link)
2. Get developement computer to see the character output.
3. Get double echo working for two communication between development computer and Nucleo

### RPi4 UART

1. Configure Nucleo RPI UART to output at 9600 baud
2. Get minicom configure for RPi.
3. Upgrade from software UART to hardware UART.
4. Get bi-directional code working.

### FreeRTOS

1. Get both UART's working at the same time using FreeRTOS.
2. Send characters down on UART and back up to the next UART.

### RPi Pin control.

1. Get the RPi digitial IO pins working.
   * Clear ESTOP
   * Alive
   * Shut down

### Motor control

1. Use the RPi pin to clear EStop
2. Start driving RPi motors

### Encoders

1. Read the encoder pins.
2. Hook the encoders up to quadrature encoders.

### Start developing Serial Port ROS protocol (ROS Serial, whatever.)

1. Get keyboard teleop working

### Servos

1. Get the ADC's working for the servos.
2. Output pulse widths

### Sonars



### LED's


### Lidar




<!--

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

[Visual Studio on RasPi](https://www.raspberrypi.org/blog/visual-studio-code-comes-to-raspberry-pi/)

[USBIP Tutorial](https://www.linux-magazine.com/Issues/2018/208/Tutorial-USB-IP)


-->
