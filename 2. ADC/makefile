# Makefile để build dự án Blink LED cho STM32F103 (no HAL/SPL)

# Tên file đầu ra
TARGET = ADC
BUILD_DIR = build

# Thư mục chứa file nguồn
SPL_DIR = SPL
EXAMPLES_DIR = Examples

# Include Paths
INCLUDES = -I$(SPL_DIR)/Inc \
           -ICore \
           -I$(EXAMPLES_DIR) \

# Example target (can be set from command line)
EXAMPLE ?= 

# Get list of example files (strip path and extension)
EXAMPLE_FILES := $(patsubst $(EXAMPLES_DIR)/%.c,%,$(wildcard $(EXAMPLES_DIR)/*.c))



SPL_SOURCES = $(wildcard $(SPL_DIR)/Src/*.c)



# Conditional source selection - ONLY check EXAMPLE when building, not for utility targets
ifeq ($(findstring clean list,$(MAKECMDGOALS)),)
    # Not running clean or list, so check for EXAMPLE
    ifeq ($(EXAMPLE),)
        ifneq ($(findstring flash build-example all,$(MAKECMDGOALS)),)
            $(error No example specified. Usage: make build-example EXAMPLE=filename (without .c extension))
        endif
    else
        SRCS_C = $(EXAMPLES_DIR)/$(EXAMPLE).c $(SPL_SOURCES)
    endif
else
    # For clean and list, don't check EXAMPLE
    SRCS_C = $(SPL_SOURCES)  # Set a default value for OBJS dependency
endif

SRCS_S = startup_stm32f103.s

OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(SRCS_C))) \
       $(patsubst %.s,$(BUILD_DIR)/%.o,$(notdir $(SRCS_S)))

# Trình biên dịch và các flags
CC      = arm-none-eabi-gcc
CFLAGS  = -mcpu=cortex-m3 -mthumb -O0 -g -Wall -ffreestanding -nostdlib \
			$(INCLUDES) \
			-DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
LDFLAGS = -T stm32f103.ld -nostdlib -Wl,--gc-sections

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)



# Mục tiêu mặc định
all: $(BUILD_DIR) $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@echo "Build completed successfully."

# List available examples
list:
	@echo "Available examples:"
	@echo "-----------------"
	@for example in $(EXAMPLE_FILES); do \
		echo "$$example"; \
	done
	@echo "-----------------"
	@echo "Usage: make example EXAMPLE=example_name"



# Example target - directly build the example without involving 'all'
example:
	@if [ ! -f "$(EXAMPLES_DIR)/$(EXAMPLE).c" ]; then \
		echo "Error: Example file '$(EXAMPLES_DIR)/$(EXAMPLE).c' does not exist."; \
		exit 1; \
	fi
	@echo "Building example: $(EXAMPLE).c"
	@mkdir -p $(BUILD_DIR)
	$(MAKE) $(BUILD_DIR)/$(TARGET).bin
	@echo "Build completed successfully."

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Handle example files
$(BUILD_DIR)/%.o: $(EXAMPLES_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# SPL sources
$(BUILD_DIR)/%.o: $(SPL_DIR)/Src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

#  Assemble file ASM into build directory
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

#  Link thành ELF in build directory
$(BUILD_DIR)/$(TARGET).elf: $(OBJS) stm32f103.ld
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@

#  Tạo file .bin từ .elf in build directory
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	arm-none-eabi-objcopy -O binary $< $@

#  Tạo file .hex từ .elf (bonus)
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	arm-none-eabi-objcopy -O ihex $< $@


# Direct build example and flash commands
build-example:
	@if [ ! -f "$(EXAMPLES_DIR)/$(EXAMPLE).c" ]; then \
		echo "Error: Example file '$(EXAMPLES_DIR)/$(EXAMPLE).c' does not exist."; \
		exit 1; \
	fi
	@echo "Building example: $(EXAMPLE).c"
	@mkdir -p $(BUILD_DIR)
	$(MAKE) $(BUILD_DIR)/$(TARGET).bin
	@echo "Build completed successfully."

flash: build-example
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

#  Clean build directory
clean:
	@echo "Cleaning build directory..."
	rm -rf $(BUILD_DIR)/*
	@echo "Clean completed."

help:
	@echo "Usage:"
	@echo "  make build-example EXAMPLE=example_name  # Build example"
	@echo "  make flash EXAMPLE=example_name          # Build and flash example" 
	@echo "  make clean                               # Clean build directory"
	@echo "  make list                                # List available examples"
	@echo "  make help                                # Show this help message"

# Add hex target
hex: $(BUILD_DIR)/$(TARGET).hex

.PHONY: all flash hex build-example clean list help