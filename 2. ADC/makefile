
# Makefile để build main.c và các file liên quan cho STM32F103 (no HAL/SPL)

TARGET = ADC
BUILD_DIR = build
SPL_DIR = SPL

# Include Paths
INCLUDES = -I$(SPL_DIR)/Inc -ICore

# Source files
SRCS_C = main.c $(wildcard $(SPL_DIR)/Src/*.c)
SRCS_S = startup_stm32f103.s

OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(SRCS_C))) \
	   $(patsubst %.s,$(BUILD_DIR)/%.o,$(notdir $(SRCS_S)))

# Compiler and flags
CC      = arm-none-eabi-gcc
CFLAGS  = -mcpu=cortex-m3 -mthumb -O0 -g -Wall -ffreestanding -nostdlib \
		  $(INCLUDES) -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
LDFLAGS = -T stm32f103.ld -nostdlib -Wl,--gc-sections


# CC      = arm-none-eabi-gcc
# CFLAGS  = -mcpu=cortex-m3 -mthumb -O0 -g -Wall -ffreestanding \
# 		  $(INCLUDES) -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
# LDFLAGS = -T stm32f103.ld -Wl,--gc-sections -lc -lm -lnosys --specs=nano.specs --specs=nosys.specs

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Default target
all: $(BUILD_DIR) $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@echo "Build completed successfully."

# Compile C files
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# SPL sources
$(BUILD_DIR)/%.o: $(SPL_DIR)/Src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble file ASM into build directory
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link ELF
$(BUILD_DIR)/$(TARGET).elf: $(OBJS) stm32f103.ld
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@

# Create .bin from .elf
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	arm-none-eabi-objcopy -O binary $< $@

# Create .hex from .elf
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	arm-none-eabi-objcopy -O ihex $< $@

# Flash
flash: all
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

# Clean build directory
clean:
	@echo "Cleaning build directory..."
	rm -rf $(BUILD_DIR)/*
	@echo "Clean completed."

# Help
help:
	@echo "Usage:"
	@echo "  make all        # Build main.c and SPL sources"
	@echo "  make flash      # Build and flash main program"
	@echo "  make clean      # Clean build directory"
	@echo "  make help       # Show this help message"

# Add hex target
hex: $(BUILD_DIR)/$(TARGET).hex

.PHONY: all flash hex clean help