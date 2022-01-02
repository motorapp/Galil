#!../../bin/linux-x86_64/GalilTest

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase("dbd/GalilTest.dbd",0,0)
GalilTest_registerRecordDeviceDriver(pdbbase)

cd ${TOP}/iocBoot/${IOC}

### Scan-support software
# crate-resident scan.  This executes 1D, 2D, 3D, and 4D scans, and caches
# 1D data, but it doesn't store anything to disk.  (See 'saveData' below for that.)
dbLoadRecords("$(SSCAN)/sscanApp/Db/standardScans.db","P=IOC01:,MAXPTS1=8000,MAXPTS2=1000,MAXPTS3=10,MAXPTS4=10,MAXPTSH=8000")
dbLoadRecords("$(SSCAN)/sscanApp/Db/saveData.db","P=IOC01:")

# Configure an example controller
< galil.cmd

< autosave.cmd

# Start the IOC
iocInit()

# Initialize saveData for step scans
saveData_Init("saveData.req", "P=IOC01:")

# Save motor positions every 5 seconds
create_monitor_set("GalilTest_positions.req", 5,"P=DMC01:")
# Save motor settings every 30 seconds
create_monitor_set("GalilTest_settings.req", 30,"IOC=IOC01:,P1=DMC01:, P2=RIO01:")
# Save kinematics every 30 seconds
create_monitor_set("GalilTest_kinematics.req", 30,"P1=DMC01:")
