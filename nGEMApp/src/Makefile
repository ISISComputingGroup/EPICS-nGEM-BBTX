TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

USR_INCLUDES += -I"$(TOP)/../../libraries/boost/include" -I"$(AREA_DETECTOR)"

## We use -EHa so windows structured exceptions are catachable with C++ catch(...) 
USR_CXXFLAGS_WIN32 += -DNOMINMAX

LIBRARY_IOC += nGEM

nGEM_SRCS += nGEMDriver.cpp

nGEM_LIBS += webget htmltidy asyn utilities zlib pcrecpp pcre pugixml
nGEM_LIBS += $(EPICS_BASE_IOC_LIBS)

nGEM_LIBS_WIN32 += oncrpc

nGEM_SYS_LIBS_WIN32 += psapi

# isisdae.dbd will be created and installed
# see isisdaeInclude.dbd
DBD += nGEM.dbd

ifeq ($(findstring static,$(EPICS_HOST_ARCH)),)
ifeq ($(findstring linux,$(EPICS_HOST_ARCH)),)
PROD_IOC += nGEMFileReader
nGEMFileReader_SRCS += nGEMFileReader.cpp
nGEMFileReader_LIBS += NeXus hdf5
nGEMFileReader_LIBS += $(EPICS_BASE_IOC_LIBS)
endif
endif

include $(ADCORE)/ADApp/commonLibraryMakefile

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
