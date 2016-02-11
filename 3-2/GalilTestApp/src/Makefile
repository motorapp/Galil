TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

PROD_IOC = GalilTestApp

# <name>.dbd will be created from <name>Include.dbd
DBD += GalilTestApp.dbd
GalilTestApp_DBD += base.dbd
GalilTestApp_DBD += asyn.dbd
GalilTestApp_DBD += drvAsynSerialPort.dbd
GalilTestApp_DBD += drvAsynIPPort.dbd
GalilTestApp_DBD += motorSupport.dbd
GalilTestApp_DBD += calcSupport.dbd
GalilTestApp_DBD += sscanSupport.dbd
GalilTestApp_DBD += asSupport.dbd
GalilTestApp_DBD += busySupport.dbd
GalilTestApp_DBD += GalilSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
GalilTestApp_SRCS += GalilTestApp_registerRecordDeviceDriver.cpp
GalilTestApp_SRCS_DEFAULT += GalilTestAppMain.cpp

#add a definition for each support application used by this application
GalilTestApp_LIBS += asyn motor calc sscan autosave busy
GalilTestApp_LIBS += GalilSupport
GalilTestApp_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
