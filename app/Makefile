.PHONY: all clean
ROOT = ..
TARGET = app
######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Os

CPU = -mcpu=cortex-m4
# fpu
#FPU = -mfpu=fpv4-sp-d16
# float-abi
FLOAT-ABI = -mfloat-abi=soft
# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32WLE5xx

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

#C_INCLUDES
C_INCLUDES =  \
-I$(ROOT)/src/include \
-I$(ROOT)/src/CMSIS/Include \
-I$(ROOT)/src/CMSIS/Device/ST/STM32WLxx/Include \
-I$(ROOT)/src/FreeRTOS/include \
-I$(ROOT)/src/FreeRTOS/portable
ASM_INCLUDES = 
C_SOURCE_DIRS = \
$(ROOT)/src/CMSIS/Device/ST/STM32WLxx/Source/Templates \
$(ROOT)/src/FreeRTOS/portable \
$(ROOT)/src/FreeRTOS\
source \
$(CURDIR)
ASM_SOURCE_DIRS = \
$(ROOT)/src/startup 

PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(ASM_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections 

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections 

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
ASFLAGS += -g -gdwarf-2
endif


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(wildcard $(ROOT)/src/ld/*.ld)

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -T $(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs

ASM_SOURCES = $(foreach dir,$(ASM_SOURCE_DIRS),$(wildcard $(dir)/*.s))
OBJECTS = $(addprefix $(BUILD_DIR)/,$(subst ../,,$(ASM_SOURCES:.s=.o)))
C_SOURCES = $(foreach dir,$(C_SOURCE_DIRS),$(wildcard $(dir)/*.c))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(subst ../,,$(filter-out $(CURDIR)%,$(C_SOURCES:.c=.o))))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(filter $(CURDIR)%, $(C_SOURCES:.c=.o))))
BEBRA = 

OBJECT_DIRS = $(addprefix $(BUILD_DIR)/,$(filter-out $(CURDIR)%,$(subst ../,,$(C_SOURCE_DIRS))))
OBJECT_DIRS += $(addprefix $(BUILD_DIR)/,$(subst ../,, $(ASM_SOURCE_DIRS)))

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(SZ) $<

$(BUILD_DIR)/$(TARGET).elf: $(OBJECT_DIRS) $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS)
	
	
$(BUILD_DIR)/%.o: $(ROOT)/%.s 
	$(AS) -c $(ASFLAGS) $< -o $@
	
$(BUILD_DIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: $(ROOT)/%.c 
	$(CC) -c $(CFLAGS) $< -o $@
	
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf 
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf 
	$(BIN) $< $@

$(OBJECT_DIRS): 
	ifeq($(OS),Windows_NT)
	mkdir "$@"
	else
	mkdir --parents "$@"
	endif

clean:
	ifeq($(OS),Windows_NT)
	rq /s/q $(BUILD_DIR)
	else
	rm -r $(BUILD_DIR)
	endif

echos:
	echo $(OBJECTS)
	