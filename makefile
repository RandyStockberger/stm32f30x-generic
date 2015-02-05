# Program specific
PROG = generic

# Compiler location defined in $PATH
GCC = arm-none-eabi-gcc.exe
OBJDUMP = arm-none-eabi-objdump.exe
OBJCOPY = arm-none-eabi-objcopy.exe
STLINK = "D:/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-Link Utility/ST-LINK_CLI.exe"

#
COPT = -Os -mthumb -mcpu=cortex-m4
LOPT = -T stm32f302r8.ld -nostartfiles

# Floating point options
NOFP =
SOFTFP = -mfloat-abi=softfp -mfpu=fpv4-sp-d16
HARDFP = -mfloat-abi=hard -mfpu=fpv4-sp-d16

$(PROG).bin:	$(PROG).elf
	$(OBJCOPY) -O binary $(PROG).elf $(PROG).bin
	$(OBJCOPY) -O srec $(PROG).elf $(PROG).srec

$(PROG).elf:	$(PROG).o reset.o isr.o
	$(GCC) $(COPT) $(NOFP) $(LOPT) -o $(PROG).elf $(PROG).o reset.o isr.o

$(PROG).o:	$(PROG).c $(PROG).h stm32f302r8.h
	$(GCC) $(COPT) -c $(PROG).c

reset.o:	reset.c stm32f302r8.h
	$(GCC) $(COPT) -c reset.c

isr.o:	isr.c isr.h stm32f302r8.h
	$(GCC) $(COPT) -c isr.c

help:
	$(GCC) --help

prog:	$(PROG).bin
	$(STLINK) -c SWD -ME -P $(PROG).bin 0x08000000 -V -Run

dis:	$(PROG).elf
	$(OBJDUMP) -d -S $(PROG).elf >$(PROG).dis

clean:
	rm -f $(PROG).elf $(PROG).srec $(PROG).bin
	rm -f $(PROG).dis
	rm -f *.o
#
# - Notes:
#
# GCC Command line:
# 	no FP
#		gcc -mthumb -mcpu=cortex-m4
#		gcc -mthumb -march=armv7e-m
# 	soft FP
#		gcc -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16
#		gcc -mthumb -march=armv7e-m -mfloat-abi=softfp -mfpu=fpv4-sp-d16
# 	hard FP
#		gcc -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
#		gcc -mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16
#
