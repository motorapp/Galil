#!../../bin/linux-x86/GalilTestApp

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase("dbd/GalilTestApp.dbd",0,0)
GalilTestApp_registerRecordDeviceDriver(pdbbase)

cd ${TOP}/iocBoot/${IOC}

### Scan-support software
# crate-resident scan.  This executes 1D, 2D, 3D, and 4D scans, and caches
# 1D data, but it doesn't store anything to disk.  (See 'saveData' below for that.)
dbLoadRecords("$(SSCAN)/sscanApp/Db/scan.db","P=IOC01:,MAXPTS1=8000,MAXPTS2=1000,MAXPTS3=10,MAXPTS4=10,MAXPTSH=8000")

## uncomment to see every command sent to galil
#epicsEnvSet("GALIL_DEBUG_FILE", "galil_debug.txt")

# Configure an example controller
< galil.cmd

< autosave.cmd

# Start the IOC
iocInit()

# Save motor positions every 5 seconds
create_monitor_set("galilTestApp_positions.req", 5,"P=DMC01:")
# Save motor settings every 30 seconds
create_monitor_set("galilTestApp_settings.req", 30,"P1=DMC01:, P2=RIO01:")

