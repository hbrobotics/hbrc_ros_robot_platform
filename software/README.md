# HBRC ROS Robot Software


## Current Status: Working through Embedded Geek Videos:

I'm currently about half way through the first video:
1. [Command line](https://www.youtube.com/watch?v=PxQw5_7yI8Q)

     openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"


2. [VSCode](https://www.youtube.com/watch?v=xaC5oWwzOt0)

## Introduction

The original strategy was to use `stm32cubeide` as the HR2 firmware development.
`stm32cubeide` is a version of Eclipse software development environment
that has the STM32CubeMX code integrated in with it along with STLink stuff for download and debug.
Both Eclipse and `stm32cubide` are written in Java.

Unfortunately, Eclipse seems to evolved into a huge piece of software and seems to
be a bit sluggish even on pretty high end PC's.
Many people are switching over from Eclipse to `vscode`.
`vscode` is written in C#, is open source (like Eclipse) and is under the MIT license.
It seems to be less sluggish than Eclipse running on the same hardware.

Making a long story made short, the plan is to use `vscode` instead of Eclipse.
The STM32CubeMX software is still written in Java (closed source),
but it can still be run stand-alone separate from Eclipse.

## VSCodium

It is possible to download `vscode` from Microsoft.
The `vscode` downloaded from Microsoft is configured with some additional trackers installed.
There is a version of `vscode` called `vscodium` which is the pure `vscode` code under the
MIT license, but without the trackers.
The `vscodium` software can be downloaded and installed by following the download/install
instructions at the [VSCodiaum web site](https://vscodium.com/).
By the way, the actual program is called `codium`.

## STM32CubeMX

The STM32CubeMX is program developed by ST that is used to configure various ST microcontrollers.
ST has developed a HAL (Hardware Abstraction Library) that is shared across the entire product line.
The STM32CubeMX program generates code that makes calls to the HAL.
It is extremely useful and worth the annoyance and tedium of downloading and installing the software
in order to get access to this functionality.

The STM32CubeMX is not open source software.
STM32CubeMX appears to be written in Java and needs to run under the JRE (Java Runtime Environment).

In order to run this software the following steps need to occur:

1. Download and install the JRE (Java Runtime Environment.)

2. Download and install the STM32CubeMX.

3. Create a wrapper that calls STM3CubeMX.

The download and install process is pretty involved and is frankly pretty annoying.

### Download JRE

Currently, the easiest way to get the JRE is via the JDK (Java Development Kit):

     sudo apt install openjdk-8-jdk

Please note that the current owner of Java is Oracle
and it is making changes the JDK (Java Development Kit) license.
It seems quite likely that step above will cease to work at some point in the future,
due to licensing conflict issues.

### Download the STM32CubeMX:

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

### `stm32cubemx` Wrapper

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

## Cross compilers

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

## OpenOCD

`openocd` is the open source download and debug support program.

     sudo apt install openocd

Also, run

     sudo apt install build-essential
     sudo apt-get install libncurses5 libncurses5:i386  # Needed for `gdb`

to get the various build tools like `make`, etc.

Edit your `~.bashrc` (or equivalent) to do add the followings:
    
     # Fill in `...` as appropriate:
     export GCC_ARM_BIN="$HOME/.../gcc-arm-none-eabi/gcc-arm-none-eabi-10-2020-q4-major/bin"

Be sure to make it match the actual version that was downloaded, since they are updated quarterly.

## Verify Environment

If everything has gone right, the following programs should be in your path:

     gcc-arm-none-eabi-gdb --version
     openocd --version
     codium --version
     stm32cubemx

## Construct an `stm32cubemx` project.

Perform the following steps:

1. Create a "workspace" directory.  For now, this is in `$HR2_DIR/software/non_ide`

     mkdir -p $HR2_DIRTORY/software/non_ide

2. Run `stm33cubemx`.

       `stm32cubemx`

3. If you have already created a "project", you may skip this step.
   Othersise,

   1. Under the `New Project` heading, find `Start My project from ST Board`.
      (It may necessary to make the `stm32cubmx` full screen in order to see this text
      in its entirety.)
      Click on the `[Access To Board Selector]` button.
      A window shows up with multiple tabs.
.      The selected tab should be `[Board Selector]`.
  
   2. (Optional) There is a text entry box called `Commercial Part Number`
      into which you may type a partial part number.
      For example, if you type `767` in this constrain all everthing to parts that contain `767`.

   3. Select the `MCU/MPU Series`.
      For the `767` part, this is the `[x] STM32F7`.

   4. There will be a `Boards List` that lists the boards that match.
      In this case, we want `Nucleo-F767ZI`, so please click on it.

   5. Next, click on `[ ]--> Start Project`

   6. A Popup that says `? Initialize all perifpherals with their default Mode? [Yes] [No]`
      shows up.  Click on `[Yes]`.

   7. A window with 4 tabs shows up.
      The 4 tabs are:

      * `[Pinout & Configuration]`:
         This one shows the outline of the integrated circuit with various pins highlighted.
         This is the initally selected tab.
         For now, this tab is ignored.

      * `[Clock Configuration]`:
         This one shows the clock configuration diagram, which can be quite complicated.
         For now, leave this one alone.

      * `[Project Manager]`:
         This one is used for setting up the project.
         There are numerous things on this tab.
          The ones of interest are:

       * `[Tools]`:
         This is some advanced tools that are ignored for now.

    8. Select the `[Project Manager]` tab and fill in the following fields:

       * `Project Name`:
          Give the project a name.
          It is recommended that the name include at some fraction of the processor.
           For example, `blinky-f767zi`.

       * `Project Location`:
         This can be thought of as a workspace directory into which multiple projects
         can be inserted.
        
       * `Toolchain / IDE`:
         This is a selection box that is normally set to `[EWARM]`.
         Please change this to `[Makefile]`

    9. Select the `[Pinout & Configuration]` tab:

       * Select the `Connectivity >` drop down menu and select `ETH`.

       * A `Mode` side window shows up.
         It probably is showing a `Mode` of `RMII`.
	 Scroll up to the first entry of the drop down menu and select `Disable`.
	 This will disable the intialization of the Ethernet device.

    10. In the upper right hand corner there is a button labeled `[Generate Code]`.
        Please click this button. 
	After a few moments a popup window says
	`The Code is successfully generated under :`
	followed by the directory of the form `/...WORKSPACE/PROJECT`.
        Whole bunch of files have been created in this projectect directory.
	The most interesting ones are

	* `PROJECT.ioc`:
	   This file is the I/O configuration file.
	* `Makefile`:
	   This file is used to build the project using the `make` program.
	* `Core/Src/main.c`:
	   This is the main C program that initializes the microcontroller.

        Both the `Makefile` and the `main.c` file need to be edited.

    11. Edit the `Makefile` to have the following target immediately after the 
	
             flash: all
	             openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

    12. Add the blinky code to `main.c`.

    13. Plugin in Nucleo.

    14. Verify that it shows up in `lsusb`.

    15. Type `make flash`.  This should download and install the blinky code.

