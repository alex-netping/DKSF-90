# DKSF-90
Firmware DKSF 90 for DKST 910.5-01 device

Folder structure:
================
/doc/ - documentation containing the description of 1-Wire protocol and device registers
/tools/ - gnu gcc arm toolchain installation executable for Windows
/src/ - firmware source code and build script (build.bat)

Building the firmware
=====================
1. If not already installed, install GNU gcc arm toolchain localed at folder /tools/gcc-arm-none-eabi-4_8-2014q3-20140805-win32.exe
At the end of installation at the last page of the UI installer, make sure to click the checkbox "Add environment path" and untick 
the check boxes "view readme file" and "run gccvar.bat"

The path to the bin folder of the toolchain then gets added to the Windows system PATH

NOTE: If you started the setup program as a NON ADMIN user (standard user) but gave it the admin rights when prompted, you need to manually add the path
to the gcc ARM toolchain as follows:(Example for Win 8):
- In the windows search, type "environment"
- Click on "Edit environment variables for your account"
- Under "User variables for <user>" either edit existing PATH variable or add new PATH varibale
- Put in the path to the bin folder of the toolchain, default C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2014q3\bin
- Click OK

2. Change to /src/ folder and run batch file build.bat
- Building for generator modified boards:
build.bat -generator

- Building for production 220V boards:
build.bat

3. The build results in two output files for the flash programmer, both found in /src/bin_output folder
dkst910.hex - hex output file
dkst910.bin - binary output file

Hardware Connection
===================
Firmware binary file in /src/bin_output is downladed into DKST 910.5-01 mcu flash using JetLink8 USB debugger.
SWD interface cable from the debugger connects into DKST 910.5-01 socket J3.
JetLink8 to DKST 910.5-01 J3 cable pin out is as follows (J3 Pin 1 marked by white arrow on the PCB):

Board J3 <====> Jetlink8
------------------------
3 <--> 7
4 <--> 9
6 <--> 15
7 <--> 1
8 <--> 4

See more info on the pinout here:
http://otladka.com.ua/wiki/doku.php?id=jetlink_3_5_6_7_8_pro_ultra_flash           

Connection order is as follows:
1. Connect target power +5V: Connect 1Wire RJ11 cable from the DKST 910.5-01 J4 socket into the host 1Wire socket, power up the host board
2. Connect Jetlink8 USB cable into the PC.
3. Go to next section (Downloading firmware binary to flash)

Downloading firmware binary to flash
====================================
1. If not already installed, install SEGGER J-Link V4.92 software suite located at /tools/Setup_JLink_V492.exe
On a 64-bit Windows 7 the Segger j-Link commander utility used for flash programming is installed by default at:
"c:\Program Files (x86)\SEGGER\Jlink_V492\Jlink.exe"
If the install path is different from default, edit /src/loadflash.bat and correct the full path to Jlink.exe

2. Change to /src/ folder and run batch file loadflash.bat
The loadflash.bat script calls J-Link commander utility passing it the command script dkst90.jlink which contains commands
to erash and program the device flash with the binary file bin_output/dkst910.bin

