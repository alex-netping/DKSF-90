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
At the end of installation at the last page of the UI installer, make sure to click the checkbox "Add environment path"
The path to the bin folder of the toolchain then gets added to the Windows system PATH

2. Change to /src/ folder and run batch file build.bat
- Building for generator modified boards:
build.bat -generator

- Building for production 220V boards:
build.bat

3. The build results in two output files for the flash programmer, both found in /src/bin_output folder
dkst910.hex - hex output file
dkst910.bin - binary output file

Downloading firmware binary to flash
====================================
1. If not already installed, install SEGGER J-Link V4.92 software suite located at /tools/Setup_JLink_V492.exe
On a 64-bit Windows 7 the Segger j-Link commander utility used for flash programming is installed by default at:
"c:\Program Files (x86)\SEGGER\Jlink_V492\Jlink.exe"
If the install path is different from default, edit /src/loadflash.bat and correct the full path to Jlink.exe

2. Change to /src/ folder and run batch file loadflash.bat
The loadflash.bat script calls J-Link commander utility passing it the command script dkst90.jlink which contains commands
to erash and program the device flash with the binary file bin_output/dkst910.bin

