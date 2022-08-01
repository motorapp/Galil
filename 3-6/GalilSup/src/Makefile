TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC_DEFAULT += GalilSupport
LIBRARY_IOC_vxWorks += -nil-

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += GalilSupport.dbd

#Require C++ 2011 standard compatibility
USR_CXXFLAGS_Linux += -std=c++11

# For sCalcPostfix.h
USR_INCLUDES += -I$(CALC)/calcApp/src

# The following are compiled and added to the Support library
GalilSupport_SRCS += GalilController.cpp GalilAxis.cpp GalilCSAxis.cpp GalilConnector.cpp GalilPoller.cpp

GalilSupport_LIBS += asyn motor calc sscan autosave busy
GalilSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
