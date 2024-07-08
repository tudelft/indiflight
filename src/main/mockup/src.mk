MAIN_SRC = \
	mockup/main.c \
	flight/indi.c \
	$(TARGET_DIR_SRC)

SRC = $(addprefix $(SRC_DIR)/, $(MAIN_SRC))


### flags

ifeq ($(SIMULATOR_BUILD),yes)
TARGET_FLAGS := -DSIMULATOR_BUILD $(TARGET_FLAGS)
endif

#################### external submodules ######################

PI_DIR = $(ROOT)/lib/main/pi-protocol/src
PI_GEN_FILES = $(PI_DIR)/pi-protocol.h $(PI_DIR)/pi-messages.h $(PI_DIR)/pi-messages.c

ifneq ($(PI_DIR),)
INCLUDE_DIRS += $(PI_DIR)
SRC += $(PI_DIR)/pi-protocol.c
SRC += $(PI_DIR)/pi-messages.c
#SIZE_OPTIMISED_SRC += $(PI_DIR)/pi-protocol.c
#SIZE_OPTIMISED_SRC += $(PI_DIR)/pi-messages.c
endif



# Do the same for the ActiveSetCtlAlloc
AS_SRC_DIR = $(ROOT)/lib/main/ActiveSetCtlAlloc/src

ifneq ($(AS_SRC_DIR),)
INCLUDE_DIRS += $(AS_SRC_DIR)
INCLUDE_DIRS += $(AS_SRC_DIR)/common/
INCLUDE_DIRS += $(AS_SRC_DIR)/lib/
AS_SRC = $(AS_SRC_DIR)/common/solveActiveSet.c
AS_SRC += $(AS_SRC_DIR)/common/setupWLS.c
AS_SRC += $(AS_SRC_DIR)/solveActiveSet_chol.c
AS_SRC += $(AS_SRC_DIR)/solveActiveSet_qr.c
AS_SRC += $(AS_SRC_DIR)/solveActiveSet_qr_naive.c
AS_SRC += $(AS_SRC_DIR)/lib/chol_math.c
AS_SRC += $(AS_SRC_DIR)/lib/qr_updates.c
AS_SRC += $(AS_SRC_DIR)/lib/qr_wrapper.c
AS_SRC += $(AS_SRC_DIR)/lib/qr_solve/qr_solve.c
AS_SRC += $(AS_SRC_DIR)/lib/qr_solve/r8lib_min.c
AS_SRC += $(AS_SRC_DIR)/lib/sparse_math.c
SRC += $(AS_SRC)
SPEED_OPTIMISED_SRC += $(AS_SRC)
OPTIONS += "AS_N_U=8"
OPTIONS += "AS_N_V=6"
OPTIONS += "AS_SINGLE_FLOAT"
OPTIONS += "AS_COST_TRUNCATE"
OPTIONS += "AS_RECORD_COST"
OPTIONS += "AS_RECORD_COST_N=5"
endif


# to get this to work do this to the extracted clapack.tgz
# 1. cd lib/main/clapack/F2CLIBS/libf2c and then
# 2. make f2c.h signal1.h sysdep1.h arith.h
LAPACK_DIR = $(ROOT)/lib/main/clapack
LAPACK_SOURCE = 
LAPACK_SOURCE_OPTIM = 
LAPACK_SOURCE_NO_OPTIM = 

ifneq ($(LAPACK_DIR),)
INCLUDE_DIRS += $(LAPACK_DIR)/INCLUDE

### OPTIMIZED SOURCES
# just compile the entire BLAS
LAPACK_SOURCE_OPTIM += $(wildcard $(LAPACK_DIR)/BLAS/SRC/*.c)

# compile all integer LAPACK functions
LAPACK_SOURCE_OPTIM += $(wildcard $(LAPACK_DIR)/SRC/i*.c)

# compile whats necessary for SGEQP3 (blocked pivoting QR factorization)
# and SORG2R (retrieving the explicit QR from SGEQP3 householder factors)
# download the dependencies from netlib.org/lapack/explore-html
# find . -name "*.f" | sed -n "s/^\.\/\([^ix].*\)\.f$/\1.c /gp" | tr -d "\n" in the src dir
LAPACK_SOURCE_SGEQP3 := sgeqp3.c sgeqrf.c sgemm.c slarf.c snrm2.c slapy2.c scopy.c sorm2r.c lsame.c slaqps.c slarfg.c sswap.c strmm.c sger.c slarft.c slaqp2.c strmv.c sscal.c slaisnan.c sgemv.c sisnan.c slarfb.c sormqr.c sgeqr2.c
LAPACK_SOURCE_OPTIM += $(wildcard $(addprefix $(LAPACK_DIR)/SRC/, $(LAPACK_SOURCE_SGEQP3)))
LAPACK_SOURCE_SORG2R := sorg2r.c slarf.c lsame.c sger.c sscal.c sgemv.c
LAPACK_SOURCE_OPTIM += $(wildcard $(addprefix $(LAPACK_DIR)/SRC/, $(LAPACK_SOURCE_SORG2R)))

### AUX PROGRAMS, NOT OPTIMIZED
LAPACK_FILTER_OUT := arithchk.c main.c uninit.c backspac.c
LAPACK_SOURCE_F2CLIB = $(filter-out $(addprefix $(LAPACK_DIR)/F2CLIBS/libf2c/, $(LAPACK_FILTER_OUT)), $(wildcard $(LAPACK_DIR)/F2CLIBS/libf2c/*.c))
LAPACK_SOURCE_NO_OPTIM += $(LAPACK_SOURCE_F2CLIB)

LAPACK_SOURCE_INSTALL = dlamch.c dsecnd.c second.c slamch.c
#LAPACK_SOURCE_NO_OPTIM += $(addprefix $(LAPACK_DIR)/INSTALL/, $(LAPACK_SOURCE_INSTALL))

### ADD SOURCES
LAPACK_SOURCE += $(LAPACK_SOURCE_OPTIM) $(LAPACK_SOURCE_NO_OPTIM)
SRC += $(LAPACK_SOURCE_OPTIM)
SRC += $(LAPACK_SOURCE_NO_OPTIM)
SPEED_OPTIMISED_SRC += $(LAPACK_SOURCE_OPTIM)

### OPTIONS
OPTIONS += "NO_LONG_LONG"
OPTIONS += "NO_BLAS_WRAP"
OPTIONS += "NO_TRUNCATE"
OPTIONS += "INTEGER_STAR_8"
endif
