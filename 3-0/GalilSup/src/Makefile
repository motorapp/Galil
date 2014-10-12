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
GalilSupport_SRCS += GalilController.cpp GalilAxis.cpp GalilCSAxis.cpp GalilConnector.cpp GalilPoller.cpp

# Galil communications library from their website
GalilSupport_SYS_LIBS_Linux += Galil
# Galil1.dll is delay loaded/imported on Windows
GalilSupport_SYS_LIBS_WIN32 += Galil1 delayimp
GalilSupport_LDFLAGS_WIN32 += /DELAYLOAD:galil1.dll

GalilSupport_LIBS += asyn motor calc sscan autosave busy
GalilSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
