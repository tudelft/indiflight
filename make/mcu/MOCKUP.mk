
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(ROOT)/lib/main/dyad

MCU_COMMON_SRC  := $(ROOT)/lib/main/dyad/dyad.c

#Flags
ARCH_FLAGS      =
DEVICE_FLAGS    =
LD_SCRIPT       = src/main/target/SITL/pg.ld
STARTUP_SRC     =

TARGET_FLAGS    = -D$(TARGET) -Wno-double-promotion -Wno-misleading-indentation
MCU_FLASH_SIZE  := 2048

ARM_SDK_PREFIX  =

MCU_EXCLUDES = \
            drivers/adc.c \
            drivers/bus_i2c.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/dma.c \
			drivers/dma_common.c \
            drivers/pwm_output.c \
            drivers/timer.c \
            drivers/system.c \
            drivers/rcc.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/rx/rx_xn297.c \
            drivers/display_ug2864hsweg01.c \
            telemetry/crsf.c \
            telemetry/ghst.c \
            telemetry/srxl.c \
            io/displayport_oled.c \
			main.c \
			common/scalar_mult_f32_asm.S

TARGET_MAP  = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

CFLAGS += -fPIC \
			-ffunction-sections

LD_FLAGS    := \
			  -L/opt/ros/humble/lib \
			  -Llib/main/micro_ros_build/lib \
			  -lrclc -lrcl -lrcutils -lrmw_microxrcedds \
              -lm \
              -lpthread \
              -lc \
              -lrt \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
			  -Wl,--no-undefined \
              -T$(LD_SCRIPT)

# Add libraries for uros message types manually
MESSAGE_TYPES = action_msgs actionlib_msgs diagnostic_msgs geometry_msgs \
                lifecycle_msgs nav_msgs rosgraph_msgs sensor_msgs shape_msgs \
                statistics_msgs std_msgs stereo_msgs test_msgs tf2_msgs \
				trajectory_msgs unique_identifier_msgs visualization_msgs
# control_msgs service_msgs

define add_flags
  LD_FLAGS += -l$(1)__rosidl_typesupport_c
endef

# Iterate over MESSAGE_TYPES to add flags for each message type
$(foreach type,$(MESSAGE_TYPES),$(eval $(call add_flags,$(type))))

#ifneq ($(filter SITL_STATIC,$(OPTIONS)),)
#LD_FLAGS     += \
#              -static \
#              -static-libgcc
#endif

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Ofast
OPTIMISE_SPEED      := -Ofast
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif
