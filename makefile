BINARY = sai

DBG_SCRIPT = debug.gdb
GDB_PORT = 4445

CONFIG_MAKE = config.mk

############
#
# Paths
#
############

toolprefix = arm-none-eabi
toolversion = 6.1.0
sourcedir = src
builddir = build
#basetoolsdir = /usr/local/gcc/${toolprefix}-toolchain-gcc-${toolversion}-hardfloat
basetoolsdir = /home/petera/toolchain/${toolprefix}-toolchain-gcc-${toolversion}-hardfloat
tools = ${basetoolsdir}/bin
sdk_dir = ../STM32Cube_FW_F7_V1.6.0
#sdk_dir = ../../poo/STM32Cube_FW_F7_V1.6.0
flashscript = flash.script

sdk = stm32f7cube
hfile = ${sourcedir}/config_header.h

CPATH =
SPATH =
INC =
SFILES =
CFILES =

#############
#
# Build tools
#
#############

CROSS_COMPILE=${tools}/arm-none-eabi-
CC = $(CROSS_COMPILE)gcc $(CFLAGS)
AS = $(CROSS_COMPILE)gcc $(AFLAGS)
LD = $(CROSS_COMPILE)ld
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size
MKDIR = mkdir -p

###############
#
# Build configs
#
###############

LD_SCRIPT = arm.ld
CFLAGS =  $(INC) $(FLAGS) 
CFLAGS += -mcpu=cortex-m7 -mno-thumb-interwork -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror -O3 -g3
CFLAGS += -gdwarf-2 -Wno-packed-bitfield-compat -Wno-unused-function
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
CFLAGS += -nostartfiles -nostdlib 
#CFLAGS += -MP -MD
AFLAGS =  $(CFLAGS)
LFLAGS = --gc-sections -cref
#LFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4
LFLAGS += -T $(LD_SCRIPT)
OFLAGS_HEX = -O ihex ${builddir}/$(BINARY).elf
OFLAGS_BIN = -O binary ${builddir}/$(BINARY).elf

CFLAGS += -DSTM32F7 -DSTM32F746xx #-DUSE_HAL_DRIVER
CFLAGS += -DHSE_VALUE=25000000

###############
#
# Files and libs
#
###############

CPATH		+= ${sourcedir}
SPATH		+= ${sourcedir}
INC			+= -I./${sourcedir}
CPATH		+= ${sdk}
SPATH		+= ${sdk}

gensysdir	= ./generic_embedded
include config.mk

SFILES		+= startup_stm32f746xx.s
CFILES		+= system_stm32f7xx.c
CFILES		+= main.c
CFILES		+= app.c
CFILES		+= sai.c
CFILES		+= dac.c

INC			+= -I./${sdk}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
INC			+= -I./${sdk}/Drivers/CMSIS/Include
INC			+= -I./${sdk}/Drivers/STM32F7xx_HAL_Driver/Inc

sdk_hal		= $(sdk)/Drivers/STM32F7xx_HAL_Driver
CFILES		+= $(filter-out $(wildcard $(sdk_hal)/Src/*template*.c), $(wildcard ${sdk_hal}/Src/*.c))

LIBS		= -L${basetoolsdir}/lib/gcc/${toolprefix}/${toolversion} -lgcc

############
#
# Tasks
#
############

vpath %.c $(CPATH)
vpath %.s $(SPATH)

INCLUDE_DIRECTIVES += $(INC)

SOBJFILES = $(SFILES:%.s=${builddir}/%.o)
OBJFILES = $(CFILES:%.c=${builddir}/%.o)

DEPFILES = $(CFILES:%.c=${builddir}/%.d)

ALLOBJFILES  = $(SOBJFILES)
ALLOBJFILES += $(OBJFILES)

.PHONY: clean

DEPENDENCIES = $(DEPFILES)

# link object files, create binary for flashing
${builddir}/$(BINARY): $(hfile) .mkdirs $(sdk) $(ALLOBJFILES)
	@echo "... linking"
	@${LD} $(LFLAGS) -Map ${builddir}/$(BINARY).map -o ${builddir}/$(BINARY).elf $(ALLOBJFILES) $(LIBS)
	@${OBJCOPY} $(OFLAGS_BIN) ${builddir}/$(BINARY).out
	@${OBJCOPY} $(OFLAGS_HEX) ${builddir}/$(BINARY).hex
	@${OBJDUMP} -hd -j .text -j.data -j .bss -j .bootloader_text -j .bootloader_data -d -S ${builddir}/$(BINARY).elf > ${builddir}/$(BINARY)_disasm.s
	@echo "${BINARY}.out is `du -b ${builddir}/${BINARY}.out | sed 's/\([0-9]*\).*/\1/g '` bytes on flash"

$(sdk):
	-@ln -s $(sdk_dir) $@
	@touch $@

ifneq ($(MAKECMDGOALS), clean)
-include $(DEPENDENCIES)
endif

# compile assembly files, arm
$(SOBJFILES) :${builddir}/%.o:%.s
	@echo "... assembly $@"
	@${MKDIR} $(@D);
	@${AS} -c -o $@ $<
		
# compile c files
$(OBJFILES) : ${builddir}/%.o:%.c
	@echo "... compile $@"
	@${MKDIR} $(@D);
	@${CC} -c -o $@ $<

# make dependencies
$(DEPFILES) : $(sdk)
$(DEPFILES) : ${builddir}/%.d:%.c
	@echo "... depend $@"; \
	rm -f $@; \
	${MKDIR} $(@D); \
	${CC} -M $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*, ${builddir}/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

# builds binary, installs softdevice and binary
all: ${builddir}/$(BINARY)

.mkdirs:
	-@${MKDIR} ${builddir}

clean:
	@echo ... clean
	@rm -rf ${builddir}
	@rm -f _$(flashscript)
	@rm -f $(sdk)

install: FLASHCMD=flash write_image erase $(shell readlink -f ${builddir}/${BINARY}.hex) 0x00000000 ihex
install: ${builddir}/$(BINARY)
	@echo ... flashing ${builddir}/${BINARY}
	@sed -e 's\FLASH\${FLASHCMD}\' $(flashscript) > _$(flashscript)
	@echo "script $(shell readlink -f _$(flashscript))" | nc localhost $(GDB_PORT)
	@${RM} _$(flashscript)

debug: ${builddir}/$(BINARY)
	@${GDB} ${builddir}/${BINARY}.elf -x $(DBG_SCRIPT)
	
	
${hfile}: ${CONFIG_MAKE}
	@echo "* Generating config header ${hfile}.."
	@echo "// Auto generated file, do not tamper" > ${hfile}
	@echo "#ifdef INCLUDE_CONFIG_HEADER" >> ${hfile}
	@echo "#ifndef _CONFIG_HEADER_H" >> ${hfile}
	@echo "#define _CONFIG_HEADER_H" >> ${hfile}
	@sed -nr 's/([^ \t]*)?[ \t]*=[ \t]*1/#define \1/p' ${CONFIG_MAKE} >> ${hfile}
	@echo "#endif" >> ${hfile}
	@echo "#endif" >> ${hfile}

info:
	@echo "* Toolchain path:    ${basetoolsdir}"
	@echo "* Building to:       ${builddir}"
	@echo "* Compiler options:  $(CFLAGS)" 
	@echo "* Assembler options: $(AFLAGS)" 
	@echo "* Linker options:    $(LFLAGS)" 
	@echo "* Linker script:     ${LD_SCRIPT}"
	
build-info:
	@echo "*** INCLUDE PATHS"
	@echo "${INC}"
	@echo "*** SOURCE PATHS"
	@echo "${CPATH}"
	@echo "*** ASSEMBLY PATHS"
	@echo "${SPATH}"
	@echo "*** SOURCE FILES"
	@echo "${CFILES}"
	@echo "*** ASSEMBLY FILES"
	@echo "${SFILES}"
	@echo "*** FLAGS"
	@echo "${FLAGS}"

