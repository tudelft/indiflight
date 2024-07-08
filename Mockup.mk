
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

OPTIONS := \
	USE_INDI \
	USE_GYRO \
	USE_ACC

FORKNAME = indiflight

TARGET_NAME := MOCKUP
TARGET = $(TARGET_NAME)
OBJECT_DIR = obj/main

include $(ROOT)/make/targets.mk
include $(ROOT)/make/mcu/$(TARGET_MCU).mk

SRC_DIR  := $(ROOT)/src/main

TARGET_DIR     = $(ROOT)/src/main/target/$(TARGET)
TARGET_DIR_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

INCLUDE_DIRS    := $(SRC_DIR) \
                   $(ROOT)/src/main/target \
                   $(ROOT)/src/main/startup \
				   $(TARGET_DIR)

include $(ROOT)/src/main/mockup/src.mk

TARGET_OBJ_DIR  = $(OBJECT_DIR)/$(TARGET_NAME)
TARGET_OBJS = $(addsuffix .o,$(addprefix $(TARGET_OBJ_DIR)/,$(basename $(SRC))))

#WARNING_FLAGS := -Wall -Wextra -Werror -Wpedantic -Wunsafe-loop-optimizations -Wold-style-definition -Wdouble-promotion
#WARNING_FLAGS := -Wall -Wextra -Wpedantic -Wunsafe-loop-optimizations -Wold-style-definition -Wdouble-promotion
WARNING_FLAGS := -w

CFLAGS += $(addprefix -D,$(OPTIONS)) \
			$(addprefix -I,$(INCLUDE_DIRS)) \
			-std=gnu17 \
			$(TARGET_FLAGS) \
			$(WARNING_FLAGS)


CLEAN_TARGETS = $(TARGET_OBJ_DIR)

.DEFAULT_GOAL := mockup
mockup : $(TARGET_OBJS)
	@echo $()
	@echo $(SRC)
	@echo "Linking objects"
	@$(CC) -o $^ $(LD_FLAGS)

$(PI_GEN_FILES) : $(PI_DIR)/pi-protocol.c $(PI_DIR)/../config.yaml $(wildcard $(PI_DIR)/../messages/*) $(wildcard $(PI_DIR)/../templates/*.j2) $(PI_DIR)/../python/generate.py
	@echo "generating pi protocol headers"
	cd lib/main/pi-protocol/ && make generate CONFIG=config.yaml
	@echo "done"

$(TARGET_OBJ_DIR)/%.o : %.c
	@mkdir -p $(dir $@)
	$(CC) -c -o $@ $(CFLAGS) $<

$(TARGET_OBJS) : $(PI_GEN_FILES)

clean :
	rm -rf $(TARGET_OBJ_DIR)