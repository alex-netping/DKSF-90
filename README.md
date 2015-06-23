# DKSF-90
Firmware DKSF 90 for DKST 910.5-01 device

Folder structure:
================
/doc/ - documentation containing the description of 1-Wire protocol and device registers
/tools/ - gnu gcc arm toolchain installation executable for Windows
/src/ - firmware source code and build script (build.bat)

Building the firmware
=====================
1. If not already installed, install GNU gcc arm toolchain localed at folder /tools/
During the installation the path to the bin folder of the toolchain should get added to the Windows system PATH

2. Change to /src/ folder and run batch file build.bat

3. The build results in two output files for the flash programmer, both found in /src/bin_output folder
dkst910.hex - hex output file
dkst910.bin - binary output file
