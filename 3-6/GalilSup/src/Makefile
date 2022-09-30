TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += GalilSupport
  
# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += GalilSupport.dbd
  
# For sCalcPostfix.h
USR_INCLUDES += -I$(CALC)/calcApp/src

# The following are compiled and added to the Support library
ifeq ($(HAVE_C++11), YES)
  #Require C++ 2011 standard compatibility
  USR_CXXFLAGS_Linux += -std=c++11
  GalilSupport_SRCS += GalilController.cpp GalilAxis.cpp GalilCSAxis.cpp GalilConnector.cpp GalilPoller.cpp
else
  # We build a dummy library so we can use the same dbd file on systems without C++11.
  GalilSupport_SRCS += GalilDummy.cpp
endif

GalilSupport_LIBS += asyn motor calc sscan autosave busy
GalilSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
