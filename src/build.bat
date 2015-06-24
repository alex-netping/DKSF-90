echo "Cleaning before build..."
del /Q *.o
del /Q*.elf
del /Q bin_output\*.*
echo "Building DKST-90..."

arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Wall -ffunction-sections -g -O0 -c -DSTM32F030F4P6 -DSTM32F030X6 -Icmsis_boot -Icmsis_core flash.c
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Wall -ffunction-sections -g -O0 -c -DSTM32F030F4P6 -DSTM32F030X6 -Icmsis_boot -Icmsis_core crc.c
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Wall -ffunction-sections -g -O0 -c -DSTM32F030F4P6 -DSTM32F030X6 -Icmsis_boot -Icmsis_core main.c
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Wall -ffunction-sections -g -O0 -c -DSTM32F030F4P6 -DSTM32F030X6 cmsis_boot/startup/startup_stm32f0xx.s
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Wall -ffunction-sections -g -O0 -c -DSTM32F030F4P6 -DSTM32F030X6 -Icmsis_core cmsis_boot/system_stm32f0xx_temp.c

echo "Linking..."

arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -g -nostartfiles -Wl,-Map=dkst910.map -O0 -Wl,--gc-sections -Wl,-Tarm-gcc-link.ld -g -o dkst910.elf flash.o main.o startup_stm32f0xx.o system_stm32f0xx_temp.o crc.o

arm-none-eabi-objcopy.exe -O binary dkst910.elf bin_output/dkst910.bin
arm-none-eabi-objcopy.exe -O ihex dkst910.elf bin_output/dkst910.hex

del /Q *.o
del /Q *.map
del /Q *.elf

echo "Build done!"
