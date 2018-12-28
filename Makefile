##########################
# IAR Makefile for STM8S #
##########################

PROJECT=Start7231

# Location of build tools and atomthreads sources
EWSTM8_DIR=C:\IAR\Embedded Workbench 7.3\stm8
EWSTM8_BIN=C:/IAR/Embedded\ Workbench\ 7.3/stm8/bin
CC=$(EWSTM8_BIN)/iccstm8
ASM=$(EWSTM8_BIN)/iasmstm8
LINK=$(EWSTM8_BIN)/ilinkstm8
HEX=$(EWSTM8_BIN)/ielftool
FLASHTOOL=C:/MCU/STMicroelectronics/st_toolset/stvp/STVP_CmdLine.exe
FLASHOPT=-Device=STM8S003F3

DEFINES = -D USE_STDPERIPH_DRIVER
DEFINES += -D STM8S003
DEFINES += -D USE_RTOS
DEFINES += -D I2C_FAST

# Sources paths
APP_SRC = src
LIB_SRC = lib

vpath %.c $(APP_SRC)
vpath %.c $(LIB_SRC)
vpath %.c StdPerphDrv/src

# Include paths
INCLUDES = -I inc
INCLUDES += -I lib
INCLUDES += -I StdPerphDrv/inc


# CPU part number
PART=stm8s003f3
ICF=lnkstm8s003f3.icf
LIB_MODEL = dlstm8ssn.h

# Directory for built objects
OUT_DIR = build
OBJ_DIR = $(OUT_DIR)/Obj
LIST_DIR = $(OUT_DIR)/List

# Models
CODE_MODEL = small
DATA_MODEL = small

# Application object files
APP_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(APP_SRC)/*.c)))
# Libraries object files
LIB_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(LIB_SRC)/*.c)))

# STM8S Peripheral driver object files
#PERIPH_OBJECTS = stm8s_adc1.o
#PERIPH_OBJECTS += stm8s_awu.o
#PERIPH_OBJECTS += stm8s_beep.o
#PERIPH_OBJECTS += stm8s_clk.o
#PERIPH_OBJECTS += stm8s_exti.o
#PERIPH_OBJECTS += stm8s_flash.o
#PERIPH_OBJECTS += stm8s_gpio.o
#PERIPH_OBJECTS += stm8s_i2c.o
#PERIPH_OBJECTS += stm8s_itc.o
#PERIPH_OBJECTS += stm8s_iwdg.o
#PERIPH_OBJECTS += stm8s_rst.o
#PERIPH_OBJECTS += stm8s_spi.o
#PERIPH_OBJECTS += stm8s_tim1.o
#PERIPH_OBJECTS += stm8s_tim2.o
#PERIPH_OBJECTS += stm8s_tim4.o
#PERIPH_OBJECTS += stm8s_uart1.o
#PERIPH_OBJECTS += stm8s_wwdg.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(APP_OBJECTS) $(LIB_OBJECTS) $(PERIPH_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(OUT_DIR)/%,$(ALL_OBJECTS))

# Target application filenames (.elf) for object
PROJ_ELFS = $(PROJECT).elf
PROJ_S19S = $(PROJECT).s19
PROJ_IHEX = $(PROJECT).hex

# Search build/output directory for dependencies
vpath %.o .\$(OBJ_DIR)
vpath %.elf .\$(OUT_DIR)
vpath %.hex .\$(OUT_DIR)

# Compiler/Assembler flags
CFLAGS = -e -Om $(DEFINES) -D NDEBUG
CFLAGS += --code_model $(CODE_MODEL) --data_model $(DATA_MODEL)
CFLAGS += --dlib_config "$(EWSTM8_DIR)\lib\$(LIB_MODEL)"
CFLAGS += -lCN $(LIST_DIR) -lBN $(LIST_DIR)
CFLAGS += --diag_suppress Pa050 --guard_calls
# --silent

DBG_CFLAGS = -e -Ol $(DEFINES) --no_cse --no_unroll --no_inline --no_code_motion --no_tbaa
DBG_CFLAGS += --no_cross_call --debug --code_model $(CODE_MODEL) --data_model $(DATA_MODEL)
DBG_CFLAGS += --dlib_config "$(EWSTM8_DIR)\lib\$(LIB_MODEL)"
DBG_CFLAGS += -lC $(LIST_DIR) -lB $(LIST_DIR) 
DBG_CFLAGS += --diag_suppress Pa050

ASMFLAGS = -M'<>' -ld $(OUT_DIR)\list --diag_suppress Pa050
ASMFLAGS += --code_model $(CODE_MODEL) --data_model $(DATA_MODEL)

DBG_ASMFLAGS = -M'<>' -r -ld $(OUT_DIR)\list --diag_suppress Pa050
DBG_ASMFLAGS += --code_model $(CODE_MODEL) --data_model $(DATA_MODEL)

LINKFLAGS = --redirect _Printf=_PrintfTinyNoMb
LINKFLAGS += --redirect _Scanf=_ScanfSmallNoMb
LINKFLAGS += --config "$(EWSTM8_DIR)\config\lnk$(PART).icf"
LINKFLAGS += --config_def _CSTACK_SIZE=0x100
LINKFLAGS += --config_def _HEAP_SIZE=0x100
LINKFLAGS += --map $(OUT_DIR)
LINKFLAGS += --entry __iar_program_start
LINKFLAGS += --merge_duplicate_sections
LINKFLAGS += --strip
LINKFLAGS += -f "$(EWSTM8_DIR)\config\math_small.xcl"

DBG_LINKFLAGS = --redirect _Printf=_PrintfTinyNoMb --redirect _Scanf=_ScanfSmallNoMb
DBG_LINKFLAGS += --config "$(EWSTM8_DIR)\config\$(ICF)" --config_def
DBG_LINKFLAGS += _CSTACK_SIZE=0x100 --config_def _HEAP_SIZE=0x100
DBG_LINKFLAGS += --entry __iar_program_start

#################
# Build targets #
#################

# All tests
all: $(OUT_DIR) $(PROJ_S19S) $(PROJ_IHEX)
Release: all
flash: all
cleanRelease: clean

# Make build/output directory
$(OUT_DIR):
	@mkdir $(OUT_DIR)
	@mkdir $(OBJ_DIR)
	@mkdir $(LIST_DIR)

# Test HEX files (one application build for each test)
$(PROJ_S19S): %.s19: $(PROJECT).elf
	@echo
	@echo Building $@
	@$(HEX) $(OUT_DIR)/$(notdir $<) $(OUT_DIR)/$@ --srec --silent

$(PROJ_IHEX): %.hex: $(PROJECT).elf
	@echo Building $@
	@$(HEX) $(OUT_DIR)/$(notdir $<) $(OUT_DIR)/$@ --ihex --silent

# Test ELF files (one application build for each test)
$(PROJ_ELFS): %.elf: $(PERIPH_OBJECTS) $(LIB_OBJECTS) $(APP_OBJECTS)
	@echo
	@echo Linking $@
	@$(LINK) $(OBJ_DIR)/*.o $(LINKFLAGS) -o $(OUT_DIR)/$@

# All C objects builder
$(ALL_OBJECTS): %.o: %.c
	@echo
	@echo Compiling $@
	@$(CC) $< $(CFLAGS) $(INCLUDES) -o $(OBJ_DIR)

# Clean
clean:
	rm -f *.o *.elf *.map *.hex *.bin *.lst *.stm8 *.s19 *.out *.s *.lst
	rm -rf $(LIST_DIR)
	rm -rf $(OBJ_DIR)
	rm -rf $(OUT_DIR)

# Flash
flash:
	@$(FLASHTOOL) $(FLASHOPT) -FileProg=$(OUT_DIR)\\$(PROJ_IHEX)
