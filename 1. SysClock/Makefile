# Makefile for STM32F103C8T6 RCC Comprehensive Example
# This Makefile compiles the RCC management example using GCC ARM toolchain
# and STM32F10x Standard Peripheral Library

# Project name
PROJECT = rcc_comprehensive_example

# Target MCU
MCU = STM32F103C8T6
MCU_FAMILY = STM32F10X_MD

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# Directories
BUILD_DIR = build
STM32_LIB_DIR = ../../../STM32F10x_StdPeriph_Lib_V3.5.0
CMSIS_DIR = $(STM32_LIB_DIR)/Libraries/CMSIS/CM3
SPL_DIR = $(STM32_LIB_DIR)/Libraries/STM32F10x_StdPeriph_Driver
STARTUP_DIR = $(STM32_LIB_DIR)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7

# Source files
C_SOURCES = \
main.c \
rcc_clock_management.c \
$(SPL_DIR)/src/stm32f10x_rcc.c \
$(SPL_DIR)/src/stm32f10x_gpio.c \
$(SPL_DIR)/src/stm32f10x_usart.c \
$(SPL_DIR)/src/stm32f10x_flash.c \
$(SPL_DIR)/src/misc.c \
$(CMSIS_DIR)/DeviceSupport/ST/STM32F10x/system_stm32f10x.c

# Assembly sources
ASM_SOURCES = \
$(STARTUP_DIR)/startup_stm32f10x_md.s

# Include directories
C_INCLUDES = \
-I. \
-I$(CMSIS_DIR)/CoreSupport \
-I$(CMSIS_DIR)/DeviceSupport/ST/STM32F10x \
-I$(SPL_DIR)/inc

# Compiler flags
CFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
CFLAGS += -DUSE_STDPERIPH_DRIVER -D$(MCU_FAMILY) -DSTM32F103xB
CFLAGS += -Wall -Wextra -Wno-unused-parameter
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -g -gdwarf-2

# Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -DDEBUG
else
CFLAGS += -Os -DNDEBUG
endif

# Assembler flags
ASFLAGS = $(CFLAGS)

# Linker script
LDSCRIPT = stm32f103c8t6.ld

# Linker flags
LDFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
LDFLAGS += -specs=nano.specs -specs=nosys.specs
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map,--cref
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -static
LDFLAGS += --specs=nano.specs

# Libraries
LIBS = -lc -lm -lnosys

# Default target
all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).bin

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Build directory
$(BUILD_DIR):
	mkdir -p $@

# Compile C sources
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $(C_INCLUDES) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Compile assembly sources
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

# Link
$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SZ) $@

# Generate hex file
$(BUILD_DIR)/$(PROJECT).hex: $(BUILD_DIR)/$(PROJECT).elf | $(BUILD_DIR)
	$(HEX) $< $@

# Generate binary file
$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf | $(BUILD_DIR)
	$(BIN) $< $@

# Clean
clean:
	-rm -fR $(BUILD_DIR)

# Flash using st-flash (STLink utility)
flash: $(BUILD_DIR)/$(PROJECT).bin
	st-flash write $< 0x8000000

# Flash using OpenOCD
flash-openocd: $(BUILD_DIR)/$(PROJECT).hex
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program $< verify reset exit"

# Debug with GDB
debug: $(BUILD_DIR)/$(PROJECT).elf
	arm-none-eabi-gdb -ex "target remote localhost:3333" -ex "monitor reset halt" $<

# Size analysis
size: $(BUILD_DIR)/$(PROJECT).elf
	$(SZ) --format=berkeley $<
	@echo "Flash usage:"
	@$(SZ) --format=sysv $< | grep "\.text\|\.data\|\.rodata" | awk '{sum += $$2} END {printf "  Used: %d bytes\n", sum}'
	@echo "  Available: 65536 bytes (64KB)"
	@echo "RAM usage:"
	@$(SZ) --format=sysv $< | grep "\.data\|\.bss" | awk '{sum += $$2} END {printf "  Used: %d bytes\n", sum}'
	@echo "  Available: 20480 bytes (20KB)"

# Disassembly
disasm: $(BUILD_DIR)/$(PROJECT).elf
	$(PREFIX)objdump -d $< > $(BUILD_DIR)/$(PROJECT).dis

# Create linker script if it doesn't exist
$(LDSCRIPT):
	@echo "Creating linker script for STM32F103C8T6..."
	@cat > $@ << 'EOF'
/* Linker script for STM32F103C8T6 */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = 0x20005000;    /* end of RAM */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400;     /* required amount of stack */

/* Sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
EOF

# Help
help:
	@echo "Available targets:"
	@echo "  all          - Build the project"
	@echo "  clean        - Clean build files"
	@echo "  flash        - Flash using st-flash"
	@echo "  flash-openocd- Flash using OpenOCD"
	@echo "  debug        - Start GDB debug session"
	@echo "  size         - Show memory usage"
	@echo "  disasm       - Generate disassembly"
	@echo "  help         - Show this help"

.PHONY: all clean flash flash-openocd debug size disasm help

# Dependencies
-include $(wildcard $(BUILD_DIR)/*.d)
