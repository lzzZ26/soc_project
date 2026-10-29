# ============================================================
# MES50HP SoC Firmware Build
# arm-none-eabi-gcc (expected in PATH)
#
# Targets:
#   make          - board build (9600 baud, echo mode)
#   make sim      - simulation build (-DSIMULATION, HSTM UART)
#   make clean
# ============================================================

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

# Directories (relative to software/)
CMSIS_DIR  = cmsis
DEVICE_DIR = $(CMSIS_DIR)/Device/ARM/CMSDK_CM0
CORE_DIR   = $(CMSIS_DIR)/CMSIS/Include
LD_DIR     = ld
COMMON_DIR = common/retarget

TARGET  = hello_soc
STARTUP = $(DEVICE_DIR)/Source/GCC/startup_CMSDK_CM0.s
SYSTEM  = $(DEVICE_DIR)/Source/system_CMSDK_CM0.c
DRIVER  = $(DEVICE_DIR)/Source/CMSDK_driver.c
RETARGET = $(COMMON_DIR)/retarget.c
STDOUT  = uart_stdout.c

CPU   = -mcpu=cortex-m0 -mthumb
INC   = -I$(DEVICE_DIR)/Include -I$(CORE_DIR) -I$(COMMON_DIR)
CFLAGS = -g -O2 $(CPU) $(INC) -DCORTEX_M0
LDFLAGS = $(CPU) -L$(LD_DIR) -T$(LD_DIR)/cmsdk_cm0.ld \
          -D__STACK_SIZE=0x200 -D__HEAP_SIZE=0x1000 \
          --specs=nosys.specs

SRCS = $(STARTUP) $(TARGET).c $(SYSTEM) $(DRIVER) $(RETARGET) $(STDOUT)

all: $(TARGET).hex

# ---- board build ----
$(TARGET).elf: $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) $(LDFLAGS) -o $@

# ---- simulation build ----
sim: CFLAGS += -DSIMULATION
sim: $(TARGET).elf
	$(OBJCOPY) -O verilog $(TARGET).elf $(TARGET).hex
	copy /y $(TARGET).hex ..\image.hex
	@echo "SIM firmware -> ../image.hex (HSTM, sim only - rebuild board before any bitstream!)"

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O verilog $(TARGET).elf $(TARGET).hex

$(TARGET).lst: $(TARGET).elf
	$(OBJDUMP) -S $(TARGET).elf > $(TARGET).lst

clean:
	rm -f *.elf *.hex *.lst *.o ../image.hex

.PHONY: all sim clean
