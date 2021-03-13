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

## Software Download and Install

Each computer/microcontroller needs is its own software development software.
Please use hardwired network connections for all initial software installs.
Setting up Wifi properly is a discussed much further below.

This is broken into the following sections:

1. [Development Computer Ubuntu 20.04 Install](#development-computer-ubuntu-2004-install):
   Get the Ubuntu 20.04 Linux distribution running on your development computer

2. [Development Computer ROS2 Install](#development-computer-ros2-install):
   Install ROS2 (Foxy) on your development computer.

3. [Development Computer STM32CubeIDE Install](#development-computer-stm32cubeide-install):
   Install the STM32CubeIDE microcontroller software on development computer.

4. [Robot Computer Ubuntu 20.04 Install](#robot-computer-ros2-install)

5. [Robot Computer ROS2 Install](#robot-computer-ros2-install)

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

### Development Computer STM32CubeIDE Install

There are multiple cross platform IDE's (Integrated Development Environment) out there.
The `STM32CubeIDE` IDE is selected because it is directly supported by microcontroller vendor
(i.e ST Microelectronics.)
There are others (e.g. `vscode`, `Keil`, etc.) that could work as well,
but they are not directly supported by STM.

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

#### Purchases

Please make the following purchases:

* Raspberry Pi 4:
  Make sure it has at lead 4GB of RAM.
  8GB is available, but is not really needed.
  While most of the stuff is available on Amazon, New Egg, Best Buy,
  the Raspberry Pi 4 is usually cheaper at places like Digi-Key, Mouser, Newark, Arrow, etc.
  Shop around.

* Raspberry Pi 4 Heat sinks:
  Make sure you get some Raspberry Pi 4 heat sinks.
  They come in package of 4 -- 1 large for the processor, 1 medium for the memory, and
  2 smaller ones for the Ethernet controller and USB controller.
  *DO NOT* forget to install the heat sinks on to the Raspberry Pi 4.

* Micro-SD to USB Adapter:
  Make sure that you have a micro-SD to USB adapter.
  If not, buy one.

* Micro-SD card:
  32GB micro-SD cards are pretty affordable.
  Make sure it is `UHS-1`.
  SanDisk is a common vendor, but there are others.
  These cards are really small and easy to lose.
  So keep careful track of them.
  Also, it does not hurt to buy an extra just in case you misplace one.

* Power Supply:
  The Raspberry Pi4 is powered via a USB-C connector.
  Supplies that plug into the wall and produce power for a USB-C connector
  are readily available from Amazon, New Egg, etc.
  However, the HR2 is meant to run off of a battery.
  Instead is recommended that yous skip the wall supply and go directly to the battery.
  Batteries come and go.
  The one that is currently recommend is
  (CONXWAN Power Bank)[https://www.amazon.com/gp/product/B08F7XM5GN/ref=ppx_yo_dt_b_asin].
  This is has 3 3A output (1 USB-C and 2 USB-A).

### Robot Computer Ubuntu 20.04 Installation

For the HR2, the nominal computer for the Robot Computer
(see the very beginning of this document) is a Raspberry Pi 4 with at least 4GB.
The Ubuntu 20.04 distribution of the Linux Operating system
is installed on the micro-SD card (typically 32GB).
The micro-SD card is inserted into the Raspberry Pi 4 and powered up.

The directions below are for a headless WiFi connection and are based on the
[Ubuntu Install Tutotoral](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi)
for the Raspberry Pi.
These instructions are for a "headless" install.
What this means is that there is no need to plug in a display, keyboard, mouse, or network cable
into the Raspberry Pi4.

The only things required are:

* Development Computer

* Raspberry Pi 4 (see above)

* USB-C Power (see above)

* Micro-SD card with adapter (see above)

* WiFi router connected to your network.

For the WiFi router,
you are going to need both the SSID (i.e. the router name) and associated password.

The next caveat on these instructions is that only one Raspberry Pi 4 can be powered on
while the installation is occurring.
The reason for this is that there is one step in the process
that requires running the `arp` (Address Resolution Protocol) and
confusion ensues if more that Raspberry Pi 4 powered up at the same time.

The basic steps are:

1. Install Ubuntu 20.04 image onto a micro-SD card.

2. Make some small changes to the micro-SD card.

3. Install micro-SD card into Raspberry Pi4 and power it up.

4. Determine the internet address of the Raspberry Pi4.

5. Log into the Raspberry Pi 4 via the `ssh` (Secure Shell) program from your development computer.

6. Perform more configuration on the Raspberry Pi 4.

#### Install Ubuntu 20.04 on micro-SD Card

Please do the following steps.

1. Install `rpi-imager`:
   Run `sudo apt install rpi-imager`.
   It may ask you for permission (type 'Y`) followe your password (if requested.)

2. Verify installation:
   Type `which rpi-imager`.
   It should respond with something like `/usr/bin/rpi-imager`.

3. Insert micro-SD card into development computer:
   Insert the micro-SD card into micro-SD card adapter.
   Plug the adapter into a Development computer 
   Dismiss any pop-windows that show up.
   If the file browser starts up, you can close that as well.

4. Run `rpi-imager`:
   Type `rpi-imager` and a window should pop with 3 buttons on it.
   The first two buttons are `[Choose OS]` and `[Choose SD Card]`.
   The third button is a grayed out `[Write]` button.
   
5. Choose OS:
   These windows change from time-to-time, so the instructions below may get a little out of date.

   1. Click on `[Choose OS`].

   2. Select `[Other general purpos OS]`.

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

#### Make Micro-SD Card Modifications

This is the part of the instructions that modifies the micro-SD card image to support headless booting.

1. Find `system-boot`:
   Start your favorite file browser.
   There should be a section called `removable devices` (or something like that.)
   You are looking for `system-boot`, please click on it.
   It should list a whole bunch of stuff.

2. Move into `system-boot` directory:
   Edit `network-config`.


* Setup Raspberry Pi 4.

  * Install your heat sinks.

  * Find a Micro-HDMI to HDMI cable.
    * Plug the micro end into either one of the two Micro-HDMI ports on the long
      edge of the Raspberry Pi on the opposite side of from the 40-pin connector.
    * Plug the other end of the cable into a display.
    * Turn on the display.

  * Plug a network cable into the RJ45 jack.
    Plug the other end into your network.

  * Plug a keyboard and mouse into the USB ports.
    If either the keyboard or mouse are wireless,
    make sure that they have batteries and are switched "on".

  * Plug the micro-SD card into the micro-SD socket on the bottom the RPi4.
    The gold connector pins face upwards when sliding the card in.

### `ssh` into Raspberry Pi4

1. ssh into Raspberry Pi4 from development computer:
   Run `ssh ubuntu@ubuntu`.
   When prompted for `password:`, type in `ubuntu`.
   The `ubuntu` will not be visible to keep it secret.

2. Create a user account with a directory:
   Run `sudo adduser NEWUSER`.

3. `sudo usermod -aG sudo NEWUSER`

   (Issue, do we need to do anything special to setup minicom.)

4. `su NEWUSER`.  Fill in password.

5. `whoami` => NEWUSER

6. `cd && pwd` => '/home/NEWUSER`
    
7. `sudo passwd ubuntu`
   Type in new password

## Road map

# https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi

# On development machine:
# link to `ip` info: https://packetpushers.net/linux-ip-command-ostensive-definition/
sudo apt install iproute2   # Get `ip` command
sudo apt install nmap       # Get `nmap` local net scanner.
sudo apt install net-tools  # Get `arp` command
sudo ip neigh flush all     # Flush arp cache *BEFORE* powering up RPi4

# Plug in network cable into RPi4.
# Install micro-SD card into RPi4.
# Power up RPi4
# Wait a couple of minutes for first boot.

nmap -sP 192.168.1.0/24    # Force all addresses to be scanned.
arp -na | grep -E "b8:27:eb|dc:a6:32"  # Get IP address of RPi4 network connection
ssh ubuntu@192.168.1.N
Password: ubuntu
# It may be necessary to run the command below to clear the fingerprint issue.
ssh-keygen -f "/home/wayne/.ssh/known_hosts" -R "192.168.1.N"  # Remove fingerprint for IP address
# Change the password
ssh ubuntu@192.168.1.N
Password: NEWPASSWORD

# Now on RPi4

sudo echo NEWHOSTNAME > /etc/hostname
sudo apt install libnss-mdns mdns-scan  # Tools needed for zeroconf
ping NEWHOSTNAME.local  # Verify zero-conf is doing its thing.
exit RPi4

# On dev computer:
ping NEWHOSTNAME.local  # Verify that zero-conf is doing its thing

ssh ubuntu@NEWHOSTNAME.local
Password: ubuntu

# Back on RPi4:

# Use nmcli to configure wifi.
sudo apt install network-manager  # Get `nmcli` tool
# Configure wifi
# Use https://kifarunix.com/connect-to-wifi-in-linux-using-nmcli-command/
sudo atp install net-tools  # Bring in `ifconfig`
ifconfig  # Verify both interfaces are up.
sudo halt

# Unplug network cable.
# Repower RPi4

# Back on Development machine:
ping NEWHOSTNAME.local
ssh ubuntu@NEWHOSTNAME.local
password: ubuntu

# Back on RPi4

# Create user accounts, change passwords, add sudo, etc.
sudo update
sudo upgrade
# Install Foxy
sudo apt install build-essential


# Useful stuff

nmap -sP 192.168.1.0/24
ssh-keygen -f "/home/wayne/.ssh/known_hosts" -R "192.168.1.112"  # Remove fingerprint for IP address
arp | grep
arp -na | grep -E "b8:27:eb|dc:a6:32"

A wifi address will not actually have RasPi HWaddress, instead it will have the address of the router

https://kifarunix.com/connect-to-wifi-in-linux-using-nmcli-command/

* Power up.

  * Plug in a 5.0 volt supply with a Type C USB connector into the USB-C connector
    next to the left of the two micro-HDMI connectors.

  * If everything works stuff will show up on the screen.

  * Eventually it will prompt with `User:`.  Type in `ubuntu`.

  * Next it will prompt with `Password:`.  Type in `ubuntu`.

  * Next it tell you that the password needs to be changed.
    It will prompt with `password:` again.  Type in `ubuntu` again.

  * Next it will ask you for a `new password`.  Type it in.  Remember it.

  * Next it will ask you for `new passward again`.  Type in the same password again.

  * Finally, it will promtp with `ubuntu@ubuntu:~$`.  You are logged in.

* OS Setup.

  * Start with updating new packages:
    * `sudo apt update`.
      Type in your `password:` when prompted.
    * `sudo apt upgrade`.
      Type `Y` to continue.
      Wait for it to download and upgrade.
  * `sudo apt install` the following packages:
    * avahi-daemon
    * avahi-discover
    * avahi-utils
    * libnss-mdns mdns-scan  # For avhi (i.e. zero-conf)
    * linux-tools-common  # for usbip
    * linux-tools-generic  # for usbip
    * network-manager  # for nmcli
    * network-tools  # for iconfig
    * minicom
    * python-is-python3
    * openssh-server
  * Install xubuntu:
    * `sudo apt install tasksel`
    * `sudo tasksel install xubuntu-desktop`
      This takes a while.  Unfortunately, the computer can overheat.a
    


### Robot Computer ROS2 Install

There is a short [ROS2 Install Video](https://www.youtube.com/watch?v=AmuLiA840fA)
that walks you through the
[ROS2 Install Debians](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)
web page instructions.
Please follow these instructions.

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
