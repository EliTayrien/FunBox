# Project Name
TARGET = template

USE_DAISYSP_LGPL = 1

#APP_TYPE = BOOT_SRAM

# Compiler options
OPT = -O1
#OPT=-Os


# Sources
CPP_SOURCES = io.cpp
CPP_SOURCES += ../../mc_common/asym_clipper.cpp
CPP_SOURCES += ../../mc_common/mc_common.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile


# Include funbox.h
C_INCLUDES += -I../../include

# Include mc_common
C_INCLUDES += -I../../mc_common
