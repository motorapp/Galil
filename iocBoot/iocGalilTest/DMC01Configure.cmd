# Configure an example DMC (digital motor controller)

##################################################################################################
# Configuration settings
# Configure these settings for site

## uncomment to see every command sent to galil
#epicsEnvSet("GALIL_DEBUG_FILE", "galil_debug.txt")

# Asyn port name (eg. DMC01, DMC02, RIO01)
epicsEnvSet("PORT", "DMC01")

# Controller address (IP address, serial port)
epicsEnvSet("ADDRESS", "192.168.0.67")

# Controller update period Unit = millisecond
# Positive value tries UDP first, falls back to TCP
# Negative value for TCP only
# Range 2-200
epicsEnvSet("UPDPERIOD", "8")

##################################################################################################
# Derived configuration settings

# Record prefix
# currently derived from asyn port name - but can be changed manually
epicsEnvSet("P", "$(PORT):")

##################################################################################################

#Load motor records for real and coordinate system (CS) motors
#Motor record version 6-9 and below
# dbLoadTemplate("$(TOP)/GalilTestApp/Db/$(PORT)_motors-v6-9down.substitutions", "P=$(P), PORT=$(PORT)")
#Motor record version 6-10 and up
dbLoadTemplate("$(TOP)/GalilTestApp/Db/$(PORT)_motors-v6-10up.substitutions", "P=$(P), PORT=$(PORT)")

#Load DMC controller features (eg.  Limit switch type, home switch type, output compare, message consoles)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_dmc_ctrl.substitutions", "P=$(P), PORT=$(PORT)")

#Load Amplifier status monitors (eg. Overtemperature, OverVoltage, ELO, etc)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_quadAmpStatus.substitutions", "P=$(P), PORT=$(PORT)")

#Load extra features for real axis/motors (eg. Motor type, encoder type)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/$(PORT)_motor_extras.substitutions", "P=$(P), PORT=$(PORT)")

#Load extra features for CS axis/motors (eg. Setpoint monitor)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_csmotor_extras.substitutions", "P=$(P), PORT=$(PORT)")

#Load kinematics for CS axis/motors (eg. Forward and reverse kinematics, kinematic variables)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_csmotor_kinematics.substitutions", "P=$(P), PORT=$(PORT)")

#Load coordinate system features (eg. Coordinate system S and T status, motor list, segments processed, moving status)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_coordinate_systems.substitutions", "P=$(P), PORT=$(PORT)")

#Load digital IO databases
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_dmc_digital_ports.substitutions", "P=$(P), PORT=$(PORT)")

#Load analog IO databases
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_dmc_analog_ports.substitutions", "P=$(P), PORT=$(PORT)")

#Load user defined functions
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_userdef_records.substitutions", "P=$(P), PORT=$(PORT)")

#Load user defined array support
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_user_array.substitutions", "P=$(P), PORT=$(PORT)")

#Load profiles
dbLoadTemplate("$(TOP)/GalilTestApp/Db/$(PORT)_profileMoveController.substitutions", "P=$(P), PORT=$(PORT)")
dbLoadTemplate("$(TOP)/GalilTestApp/Db/$(PORT)_profileMoveAxis.substitutions", "P=$(P), PORT=$(PORT)")

# GalilCreateController command parameters are:
#
# 1. Const char *portName 	- The name of the asyn port that will be created for this controller
# 2. Const char *address 	- The address of the controller
# 3. double updatePeriod	- The time in ms between datarecords 2ms min, 200ms max.  Async if controller + bus supports it, otherwise is polled/synchronous.
#                       	- Recommend 50ms or less for ethernet
#                       	- Specify negative updatePeriod < 0 to force synchronous tcp poll period.  Otherwise will try async udp mode first

# Create a Galil controller
GalilCreateController("$(PORT)", "$(ADDRESS)", "$(UPDPERIOD)")

# GalilCreateAxis command parameters are:
#
# 1. char *portName Asyn port for controller
# 2. char  axis A-H,
# 3. char  *Motor interlock digital port number 1 to 8 eg. "1,2,4".  1st 8 bits are supported
# 4. int   Interlock switch type 0 active when opto active, all other values switch type active when opto inactive

# Create the axis
GalilCreateAxis("$(PORT)","A","",1)
GalilCreateAxis("$(PORT)","B","",1)
GalilCreateAxis("$(PORT)","C","",1)
GalilCreateAxis("$(PORT)","D","",1)
GalilCreateAxis("$(PORT)","E","",1)
GalilCreateAxis("$(PORT)","F","",1)
GalilCreateAxis("$(PORT)","G","",1)
GalilCreateAxis("$(PORT)","H","",1)

# GalilAddCode command parameters are:
# Add custom code to generated code
# 1. char *portName Asyn port for controller
# 2. int section = code section to add custom code into 0 = card code, 1 = thread code, 2 = limits code, 3 = digital code
# 3. char *code_file custom code file
# GalilAddCode("$(PORT)", 1, "customcode.dmc")

# GalilReplaceHomeCode command parameters are:
# Replace generated axis home code with custom code
# 1. char *portName Asyn port for controller
# 2. char *Axis A-H
# 3. char *code_file custom code file
# GalilReplaceHomeCode("$(PORT)", "A", "homeA.dmc")

# GalilCreateCSAxes command parameters are:
#
# 1. char *portName Asyn port for controller

#Create all CS axes (ie. I-P axis)
GalilCreateCSAxes("$(PORT)")

# GalilStartController command parameters are:
#
# 1. char *portName Asyn port for controller
# 2. char *code file(s) to deliver to the controller we are starting. "" = use generated code (recommended)
#             Specify a single file or to use templates use: headerfile;bodyfile1!bodyfile2!bodyfileN;footerfile
# 3. int   Burn program to EEPROM conditions
#             0 = transfer code if differs from eeprom, dont burn code to eeprom, then finally execute code thread 0
#             1 = transfer code if differs from eeprom, burn code to eeprom, then finally execute code thread 0
#             It is asssumed thread 0 starts all other required threads
# 4. int   Thread mask.  Check these threads are running after controller code start.  Bit 0 = thread 0 and so on
#             if thread mask < 0 nothing is checked
#             if thread mask = 0 and GalilCreateAxis appears > 0 then threads 0 to number of GalilCreateAxis is checked (good when using the generated code)

# Start the controller
GalilStartController("$(PORT)", "", 1, 0)

# Start the controller
# Example using homing routine template assembly
#GalilStartController("$(PORT)", "$(GALIL)/GalilSup/Db/galil_Default_Header.dmc;$(GALIL)/GalilSup/Db/galil_Home_RevLimit.dmc!$(GALIL)/GalilSup/Db/galil_Home_ForwLimit.dmc!$(GALIL)/GalilSup/Db/galil_Home_Home.dmc!$(GALIL)/GalilSup/Db/galil_Home_ForwLimit.dmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.dmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.dmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.dmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.dmc;$(GALIL)/GalilSup/Db/galil_Default_Footer.dmc", 0, 0, 3)

# GalilCreateProfile command parameters are:
#
# 1. char *portName Asyn port for controller
# 2. Int maxPoints in trajectory

# Create trajectory profiles
GalilCreateProfile("$(PORT)", 2000)

# DMC autosave restore configuration
# restore settings in pass 0 so encoder ratio is set correctly for position restore in device support init
set_pass0_restoreFile("GalilTest_$(PORT)Settings.sav")
# restore positions in pass 0 so motors don't move
set_pass0_restoreFile("GalilTest_$(PORT)Positions.sav")
# restore kinematic equation character arrays in pass 1
set_pass1_restoreFile("GalilTest_$(PORT)Kinematics.sav")

