# Configure an example RIO (Remote IO PLC controller)

##################################################################################################
# Configuration settings
# Configure these settings for site

## uncomment to see every command sent to galil
#epicsEnvSet("GALIL_DEBUG_FILE", "galil_debug.txt")

# Asyn port name (eg. DMC01, DMC02, RIO01)
epicsEnvSet("PORT", "RIO01")

# Controller address (IP address, serial port)
epicsEnvSet("ADDRESS", "192.168.0.110")

# Controller update period Unit = millisecond
# Positive value tries UDP first, falls back to TCP
# Negative value for TCP only
# Range 2-200
epicsEnvSet("UPDPERIOD", "8")

##################################################################################################
# Derived configuration settings

# Record prefix derived from asyn port name
epicsEnvSet("P", "$(PORT):")

##################################################################################################

#Load RIO controller features (eg.  Model, IP address, message consoles)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_rio_ctrl.substitutions", "P=$(P), PORT=$(PORT)")

#Load digital IO databases
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_rio_digital_ports.substitutions", "P=$(P), PORT=$(PORT)")

#Load analog IO databases
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_rio_analog_ports.substitutions", "P=$(P), PORT=$(PORT)")

#Load user defined functions
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_userdef_records.substitutions", "P=$(P), PORT=$(PORT)")

#Load user defined array support
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_user_array.substitutions", "P=$(P), PORT=$(PORT)")

# GalilCreateController command parameters are:
#
# 1. Const char *portName 	- The name of the asyn port that will be created for this controller
# 2. Const char *address 	- The address of the controller
# 3. double updatePeriod	- The time in ms between datarecords 2ms min, 200ms max.  Async if controller + bus supports it, otherwise is polled/synchronous.
#                       	- Recommend 50ms or less for ethernet
#                       	- Specify negative updatePeriod < 0 to force synchronous tcp poll period.  Otherwise will try async udp mode first

# Create a Galil controller
GalilCreateController("$(PORT)", "$(ADDRESS)", "$(UPDPERIOD)")

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
GalilStartController("$(PORT)", "rio.dmc", 1, 0)

# RIO autosave restore configuration
# restore settings in pass 0
set_pass0_restoreFile("GalilTest_$(PORT)Settings.sav")

