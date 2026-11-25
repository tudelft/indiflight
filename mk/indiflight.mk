# Indiflight-owned sources and libraries, included from mk/source.mk.
# Every file here is wrapped in its own feature #ifdef, so listing it
# unconditionally costs nothing when the feature is off.

INDIFLIGHT_SRC = \
            common/maths_indi.c \
            common/filter_biquad.c \
            flight/ahrs_compat.c \
            flight/indi.c \
            flight/indi_init.c \
            flight/indi_glue.c

COMMON_SRC += $(INDIFLIGHT_SRC)
SPEED_OPTIMISED_SRC += \
            common/maths_indi.c \
            common/filter_biquad.c \
            flight/indi.c \
            flight/indi_glue.c

ifneq ($(findstring -DUSE_INDI,$(EXTRA_FLAGS)),)
# ActiveSetCtlAlloc (submodule lib/main/ActiveSetCtlAlloc), control allocation for INDI
AS_DIR = ActiveSetCtlAlloc/src
INCLUDE_DIRS += $(LIB_MAIN_DIR)/$(AS_DIR) \
                $(LIB_MAIN_DIR)/$(AS_DIR)/common \
                $(LIB_MAIN_DIR)/$(AS_DIR)/lib
AS_SRC = \
            $(AS_DIR)/common/solveActiveSet.c \
            $(AS_DIR)/common/setupWLS.c \
            $(AS_DIR)/solveActiveSet_chol.c \
            $(AS_DIR)/solveActiveSet_qr.c \
            $(AS_DIR)/solveActiveSet_qr_naive.c \
            $(AS_DIR)/lib/chol_math.c \
            $(AS_DIR)/lib/qr_updates.c \
            $(AS_DIR)/lib/qr_wrapper.c \
            $(AS_DIR)/lib/qr_solve/qr_solve.c \
            $(AS_DIR)/lib/qr_solve/r8lib_min.c \
            $(AS_DIR)/lib/sparse_math.c
COMMON_SRC += $(AS_SRC)
SPEED_OPTIMISED_SRC += $(AS_SRC)
OPTIONS += AS_N_U=8 AS_N_V=6 AS_SINGLE_FLOAT AS_COST_TRUNCATE AS_RECORD_COST AS_RECORD_COST_N=5

# third-party lib is written against double literals; upstream promotes
# -Wdouble-promotion to an error, so silence it only for these files
$(foreach f,$(AS_SRC),$(eval SRC_CFLAGS_$(notdir $(f)) += -Wno-double-promotion))
endif
