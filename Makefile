##### Project #####

PROJECT        ?= app
# Top path of the template
TOP             = .
# The path for generated files
BUILD_DIR       = _build

##### Options #####

LDSCRIPT        = CORE/Ld/Link_58x.ld

# add INT_SOFT for non-wch gcc
LIB_FLAGS       = xDEBUG=0

# version == file with number looks like 1015
VERSION_FILE    = version
CONTENTS		= $$(cat $(VERSION_FILE))

ifndef SW_VERSION
# if sw_version is not written by param - then get it from file
SW_VERSION 	= $$(echo $(CONTENTS) | cut -d'.' -f1 | sed 's/^0*\([1-9]\)/\1/;s/^0*$$/0/')  
endif

DT_UNIX 	?= $(shell date +%s)

# GCC 12: riscv-none-elf-
# GCC 11 and below: riscv-none-embed-
GCC_PREFIX      ?= riscv-none-elf-

OPENOCD_PATH    ?= /opt/MRS_Community/toolchain/OpenOCD/bin

OPENOCD_CFG     = Misc/wch-riscv.cfg

# C source folders
CDIRS := \
  APP \
  CORE/RVMSIS \
  CORE/Startup \
  CORE/StdPeriphDriver \
  HAL \
  LIB \
  Profile \

# C source files (if there are any single ones)
CFILES  := 

# ASM source folders
ADIRS   := 
# ASM single files

AFILES  = CORE/Startup/startup_CH583.S
LIBS    = LIB/libCH58xBLE.a \
           CORE/StdPeriphDriver/libISP583.a

INCLUDES := CORE/StdPeriphDriver/inc \
 CORE/RVMSIS


# Include paths
INCLUDES += \
  APP \
  APP/include \
  HAL/include \
  LIB \
  Profile/include \

##### Optional Libraries ############

include ./rules.mk
