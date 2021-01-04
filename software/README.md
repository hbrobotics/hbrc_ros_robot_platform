# HR2 Robot Software

## Introduction

There are three processors:

* Host Computer:
  The host computer is a 64-bit x86 architecture processor.
  This typically some version of the Ubuntu Linux distribution.

* Robot Computer:
  The robot computer is primarily used to run ROS 2 directly on the HR2.
  The nominal computer is the Raspberry Pi 4 which is a 64-bit Arm architecture processor.
  Again, it runs some version of the Ubuntu Linux distribution.

* Robot Microcontroller:
  The robot microcontroller is an STM 32-bit microcontroller.
  Currently, it is an STM32F767ZI.

The ultimate goal is to be able to develop, download, and debug code
from the host computer for all three of the processors.

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


