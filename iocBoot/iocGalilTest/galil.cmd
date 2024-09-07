## uncomment to see every command sent to galil
#epicsEnvSet("GALIL_DEBUG_FILE", "galil_debug.txt")

#Load motor records for real and coordinate system (CS) motors
#Motor record version 6-9 and below
#dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_motors-v6-9down.substitutions")
#Motor record version 6-10 and up
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_motors-v6-10up.substitutions")

#Load DMC controller features (eg.  Limit switch type, home switch type, output compare, message consoles)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_dmc_ctrl.substitutions")

#Load RIO controller features (eg.  Model, IP address, message consoles)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_rio_ctrl.substitutions")

#Load extra features for real axis/motors (eg. Motor type, encoder type)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_motor_extras.substitutions")

#Load extra features for CS axis/motors (eg. Setpoint monitor)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_csmotor_extras.substitutions")

#Load kinematics for CS axis/motors (eg. Forward and reverse kinematics, kinematic variables)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_csmotor_kinematics.substitutions")

#Load coordinate system features (eg. Coordinate system S and T status, motor list, segments processed, moving status)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_coordinate_systems.substitutions")

#Load digital IO databases
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_digital_ports.substitutions")

#Load analog IO databases
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_analog_ports.substitutions")

#Load user defined functions
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_userdef_records.substitutions")

#Load user defined array support
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_user_array.substitutions")

#Load profiles
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_profileMoveController.substitutions")
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_profileMoveAxis.substitutions")

# GalilCreateController command parameters are:
#
# 1. Const char *portName 	- The name of the asyn port that will be created for this controller
# 2. Const char *address 	- The address of the controller
# 3. double updatePeriod	- The time in ms between datarecords 2ms min, 200ms max.  Async if controller + bus supports it, otherwise is polled/synchronous.
#                       	- Recommend 50ms or less for ethernet
#                       	- Specify negative updatePeriod < 0 to force synchronous tcp poll period.  Otherwise will try async udp mode first

# Create a Galil controller
GalilCreateController("Galil", "192.168.0.67", 8)

# Create a Galil controller
GalilCreateController("RIO", "192.168.0.110", 2)

# GalilCreateAxis command parameters are:
#
# 1. char *portName Asyn port for controller
# 2. char  axis A-H,
# 3. int   limits_as_home (0 off 1 on), 
# 4. char  *Motor interlock digital port number 1 to 8 eg. "1,2,4".  1st 8 bits are supported
# 5. int   Interlock switch type 0 normally open, all other values is normally closed interlock switch type

# Create the axis
GalilCreateAxis("Galil","A",1,"",1)
GalilCreateAxis("Galil","B",1,"",1)
GalilCreateAxis("Galil","C",1,"",1)
GalilCreateAxis("Galil","D",1,"",1)
GalilCreateAxis("Galil","E",1,"",1)
GalilCreateAxis("Galil","F",1,"",1)
GalilCreateAxis("Galil","G",1,"",1)
GalilCreateAxis("Galil","H",1,"",1)

# GalilAddCode command parameters are:
# Add custom code to generated code
# 1. char *portName Asyn port for controller
# 2. int section = code section to add custom code into 0 = card code, 1 = thread code, 2 = limits code, 3 = digital code
# 3. char *code_file custom code file
# GalilAddCode("Galil", 1, "customcode.dmc")

# GalilReplaceHomeCode command parameters are:
# Replace generated axis home code with custom code
# 1. char *portName Asyn port for controller
# 2. char *Axis A-H
# 3. char *code_file custom code file
# GalilReplaceHomeCode("Galil", "C", "customhoming.dmc")

# GalilCreateCSAxes command parameters are:
#
# 1. char *portName Asyn port for controller

#Create all CS axes (ie. I-P axis)
GalilCreateCSAxes("Galil")

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
GalilStartController("Galil", "", 1, 0)

# Start the controller
# Example using homing routine template assembly
#GalilStartController("Galil", "$(GALIL)/GalilSup/Db/galil_Default_Header.dmc;$(GALIL)/GalilSup/Db/galil_Home_RevLimit.dmc!$(GALIL)/GalilSup/Db/galil_Home_ForwLimit.dmc!$(GALIL)/GalilSup/Db/galil_Home_Home.dmc!$(GALIL)/GalilSup/Db/galil_Home_ForwLimit.dmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.dmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.dmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.dmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.dmc;$(GALIL)/GalilSup/Db/galil_Default_Footer.dmc", 0, 0, 3)

# Start the controller
GalilStartController("RIO", "", 1, 0)

# GalilCreateProfile command parameters are:
#
# 1. char *portName Asyn port for controller
# 2. Int maxPoints in trajectory

# Create trajectory profiles
GalilCreateProfile("Galil", 2000)

