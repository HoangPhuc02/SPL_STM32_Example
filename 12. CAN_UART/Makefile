################################################################################
# STM32F103 UART Driver Makefile
# Author: hoangphuc540202@gmail.com
# Date: 22/9/2025
################################################################################

# Project name
PROJECT_NAME = STM32F103_UART_CAN_Driver

# Target microcontroller
TARGET = STM32F103C8T6

# Build directory
BUILD_DIR = build
SPL_DIR = SPL
# Tools
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
OBJDUMP = arm-none-eabi-objdump

# Compiler flags
CFLAGS = -mcpu=cortex-m3
CFLAGS += -mthumb
CFLAGS += -std=gnu99
CFLAGS += -Wall -Wextra
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -Os
CFLAGS += -g3

# Defines
CFLAGS += -DSTM32F10X_MD
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -DHSE_VALUE=8000000

# Include paths
INCLUDES = -I.
INCLUDES += -IConfig
INCLUDES += -IDrivers/Uart
INCLUDES += -IDrivers/Systick
INCLUDES += -ISPL/inc
INCLUDES += -ICMSIS
INCLUDES += -ICore

# Linker flags
LDFLAGS = -mcpu=cortex-m3
LDFLAGS += -mthumb
LDFLAGS += -specs=nano.specs
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(PROJECT_NAME).map,--cref
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -static
LDFLAGS += -Wl,--start-group -lc -lm -Wl,--end-group

# Linker script
LINKER_SCRIPT = Linker/stm32f103.ld

# Source files
SOURCES = main.c
SOURCES += Drivers/Uart/uart.c
SOURCES += Drivers/Systick/systick.c
SOURCES += $(wildcard $(SPL_DIR)/Src/*.c)
SOURCES += CMSIS/system_stm32f10x.c
SOURCES += Startup/startup_stm32f10x_md.s

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(basename $(SOURCES))))

# Default target
all: $(BUILD_DIR)/$(PROJECT_NAME).elf $(BUILD_DIR)/$(PROJECT_NAME).hex $(BUILD_DIR)/$(PROJECT_NAME).bin size

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)/Drivers/Uart
	mkdir -p $(BUILD_DIR)/Drivers/Systick
	mkdir -p $(BUILD_DIR)/SPL/src
	mkdir -p $(BUILD_DIR)/CMSIS
	mkdir -p $(BUILD_DIR)/Startup

# Compile C source files
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "Compiling $<"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Compile assembly source files
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "Assembling $<"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Link
$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJECTS)
	@echo "Linking $@"
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# Create HEX file
$(BUILD_DIR)/$(PROJECT_NAME).hex: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Creating HEX file $@"
	$(OBJCOPY) -O ihex $< $@

# Create BIN file
$(BUILD_DIR)/$(PROJECT_NAME).bin: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Creating BIN file $@"
	$(OBJCOPY) -O binary $< $@

# Show size information
size: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo ""
	@echo "=== Size Information ==="
	$(SIZE) $<
	@echo ""

# Generate disassembly
disasm: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Generating disassembly..."
	$(OBJDUMP) -d $< > $(BUILD_DIR)/$(PROJECT_NAME).dis

# Flash using st-link
flash: $(BUILD_DIR)/$(PROJECT_NAME).bin
	@echo "Flashing $(TARGET)..."
	st-flash write $(BUILD_DIR)/$(PROJECT_NAME).bin 0x8000000

# Flash using STM32CubeProgrammer
flash-cube: $(BUILD_DIR)/$(PROJECT_NAME).hex
	@echo "Flashing with STM32CubeProgrammer..."
	STM32_Programmer_CLI -c port=SWD -w $(BUILD_DIR)/$(PROJECT_NAME).hex -v -rst

# Erase flash
erase:
	@echo "Erasing flash..."
	st-flash erase

# Debug with OpenOCD and GDB
debug: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Starting debug session..."
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg &
	arm-none-eabi-gdb -ex "target remote localhost:3333" -ex "load" $(BUILD_DIR)/$(PROJECT_NAME).elf

# Clean build files
clean:
	@echo "Cleaning build files..."
	rm -rf $(BUILD_DIR)

# Create release package
release: all
	@echo "Creating release package..."
	mkdir -p Release
	cp $(BUILD_DIR)/$(PROJECT_NAME).hex Release/
	cp $(BUILD_DIR)/$(PROJECT_NAME).bin Release/
	cp $(BUILD_DIR)/$(PROJECT_NAME).elf Release/
	cp README.md Release/ 2>/dev/null || echo "README.md not found"
	@echo "Release package created in Release/ directory"

# Show help
help:
	@echo "STM32F103 UART Driver Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  all       - Build the project (default)"
	@echo "  clean     - Clean build files"
	@echo "  flash     - Flash using st-link"
	@echo "  flash-cube- Flash using STM32CubeProgrammer"
	@echo "  erase     - Erase flash memory"
	@echo "  debug     - Start debug session with OpenOCD/GDB"
	@echo "  size      - Show size information"
	@echo "  disasm    - Generate disassembly file"
	@echo "  release   - Create release package"
	@echo "  help      - Show this help"
	@echo ""
	@echo "Configuration:"
	@echo "  Project: $(PROJECT_NAME)"
	@echo "  Target:  $(TARGET)"
	@echo "  Build:   $(BUILD_DIR)/"
	@echo ""

# Show project info
info:
	@echo "=== Project Information ==="
	@echo "Project Name: $(PROJECT_NAME)"
	@echo "Target MCU:   $(TARGET)"
	@echo "Build Dir:    $(BUILD_DIR)"
	@echo "Compiler:     $(CC)"
	@echo "Linker Script:$(LINKER_SCRIPT)"
	@echo ""
	@echo "=== Source Files ==="
	@$(foreach src,$(SOURCES),echo "  $(src)";)
	@echo ""
	@echo "=== Compiler Flags ==="
	@echo "$(CFLAGS)"
	@echo ""
	@echo "=== Include Paths ==="
	@echo "$(INCLUDES)"
	@echo ""

# Testing targets
test-polling:
	@echo "Testing UART in polling mode..."
	@echo "Connect to UART at 115200 baud and select option 1"

test-interrupt:
	@echo "Testing UART in interrupt mode..."
	@echo "Connect to UART at 115200 baud and select option 2"

test-dma:
	@echo "Testing UART in DMA mode..."
	@echo "Connect to UART at 115200 baud and select option 3"

# Phony targets
.PHONY: all clean flash flash-cube erase debug size disasm release help info
.PHONY: test-polling test-interrupt test-dma

################################################################################
# Usage Examples:
#
# Build project:           make
# Clean and rebuild:       make clean all
# Flash to MCU:           make flash
# Start debug session:    make debug
# Create release:         make release
# Show size info:         make size
# Show help:              make help
#
################################################################################