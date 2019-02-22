TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += nGEM

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install nGEM.dbd into <top>/dbd
DBD += nGEM.dbd

# specify all source files to be compiled and added to the library
#nGEM_SRCS += xxx

nGEM_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

