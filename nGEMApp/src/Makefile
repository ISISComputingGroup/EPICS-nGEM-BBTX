TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Build the IOC support library

USR_INCLUDES += -I"$(TOP)/../../libraries/boost/include" -I"$(AREA_DETECTOR)"
USR_CXXFLAGS_WIN32 += /DNOMINMAX

LIBRARY_IOC += nGEM
#GP2Camera_SRCS += GP2CameraDriver.cpp

nGEM_LIBS += asyn oncrpc utilities zlib pcrecpp pcre pugixml
nGEM_LIBS += $(EPICS_BASE_IOC_LIBS)

DBD += nGEM.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
