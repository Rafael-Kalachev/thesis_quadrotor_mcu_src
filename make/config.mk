
##
# LOGGING
##

LOGFILE	:= build/buildlog.log

## VERBOSITY
#	0	: Do NOT log
#	1	: Log only the supplied message

VERBOSITY	:= 1
VERBOSITY_ECHO	:= $(VERBOSITY)
VERBOSITY_LOG	:= $(VERBOSITY)

LOG_FLAGS	:=
LOG_FLAGS	+= --vecho=$(VERBOSITY_ECHO)
LOG_FLAGS	+= --vlog=$(VERBOSITY_LOG)
LOG_FLAGS	+= --logfile=$(LOGFILE)

##
# CTAGS
##

CTAGS :=	ctags
CTAGS_FLAGS := 	--recurse=yes
CTAGS_FLAGS +=	--exclude=.git
CTAGS_FLAGS +=	--exclude=BUILD
CTAGS_FLAGS +=	--exclude=\*.swp
CTAGS_FLAGS +=	--exclude=\*.o
CTAGS_FLAGS +=	--exclude=\*.a




##
# CONTROLLER
##

MCU_FLASH_MEMORY_START_ADDRESS	:= 0x08000000
MCU_MODEL	:= STM32F429_439xx

##
# COMPILATION
##


C_FLAGS  :=

# for debugging
C_FLAGS	+= -g
# optimization level
C_FLAGS	+= -O0
# configure for the cortex m4
C_FLAGS	+= -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
# configure FPU
C_FLAGS	+= -mfloat-abi=hard -mfpu=fpv4-sp-d16
# configure Warnings
C_FLAGS += -Wall -Wextra
# system includes
C_FLAGS += -I./startup
C_FLAGS	+= -I./std_perif/inc
C_FLAGS += -I./system
C_FLAGS += -I./cmsis/inc
# home includes
C_FLAGS += -I.
# controller model
C_FLAGS	+= -D$(MCU_MODEL)
C_FLAGS += -DUSE_STDPERIPH_DRIVER
C_FLAGS	+= -DUSE_FULL_ASSERT
C_FLAGS += -DARM_MATH_CM4



LD_FLAGS :=
# add the linker script
LD_FLAGS	+= -Tlinker/stm32_flash.ld
