# STM32F103 CAN Example Makefile
# Target MCU: STM32F103C8T6
# Toolchain: arm-none-eabi-gcc

# Project name
PROJECT = CAN_example

# Source files
SRCS = main.c \
       SPL/src/misc.c \
       SPL/src/stm32f10x_can.c \
       SPL/src/stm32f10x_gpio.c \
       SPL/src/stm32f10x_rcc.c

# Include directories
INCS = -I. \
       -ISPL/inc \
       -ICore

# Linker script
LDSCRIPT = stm32f103.ld

# Startup file
STARTUP = startup_stm32f103.s

# Toolchain commands
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# Flags
CFLAGS = -mcpu=cortex-m3 -mthumb -O2 -Wall $(INCS)
LDFLAGS = -T$(LDSCRIPT) -nostartfiles

# Output directories
BUILD_DIR = build

# Objects
OBJS = $(addprefix $(BUILD_DIR)/, $(SRCS:.c=.o)) $(BUILD_DIR)/startup_stm32f103.o

# Default target
all: $(BUILD_DIR) $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).bin

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Compile C files
$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup file
$(BUILD_DIR)/startup_stm32f103.o: $(STARTUP)
	$(AS) -mcpu=cortex-m3 -mthumb $< -o $@

# Link
$(BUILD_DIR)/$(PROJECT).elf: $(OBJS)
	$(CC) -mcpu=cortex-m3 -mthumb -O2 -nostdlib $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

# Create binary
$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

# Flash (using openocd)
flash: all
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $(BUILD_DIR)/$(PROJECT).bin 0x08000000 verify reset exit"
# Clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean flash
