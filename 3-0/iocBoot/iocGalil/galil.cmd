#Load motor records
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_motors.substitutions")

#Load extra features that have controller wide scope (eg.  Limit switch type, home switch type)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_ctrl_extras.substitutions")

#Load extra features for axis/motors (eg. Motor type, encoder type)
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_motor_extras.substitutions")

#Load coordinate system features
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_coordinate_systems.substitutions")

#Load digital IO databases
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_digital_ports.substitutions")

#Load analog IO databases
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_analog_ports.substitutions")

#Load user defined functions
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_userdef_records.substitutions")

#Load profiles
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_profileMoveController.substitutions")
dbLoadTemplate("$(TOP)/GalilTestApp/Db/galil_profileMoveAxis.substitutions")

# GalilCreateController command parameters are:
#
# 1. Const char *portName 	- The name of the asyn port that will be created for this controller
# 2. Const char *address 	- The address of the controller
# 3. double updatePeriod	- The time in ms between datarecords 8ms minimum.  Async if controller + bus supports it, otherwise is polled/synchronous.

# Create a Galil controller
GalilCreateController("Galil", "192.168.0.55", 8)

# Create a Galil controller
GalilCreateController("RIO", "192.168.0.101", 8)

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

# GalilCreateCSAxis command parameters are:
#
# 1. char *portName Asyn port for controller
# 2. char *forward Comma separated list of forward kinematic transforms eg. I=(A+B)/2,J=B-A
# 3. char *reverse Comma separated list of reverse kinematic transforms eg. A=I-J/2,B=I+J/2
#
#    Real axis are: A-H
#    Coordinate axis are: I-P
#    Variables are: Q-Z
#    16 Args maximum per equation
#
#    Real, csaxis, and variables are allowed in forward transform equation
#    Real, csaxis, and variables are allowed in reverse transform equation
#
# Create example coordinate system axis
# 2Slit example using real motors A, and B
# I = Slit centre coordinate system axis
# J = Slit width coordinate system axis

#Normal slit example
GalilCreateCSAxes("Galil","I=(A+B)/2,J=B-A","A=I-J/2,B=I+J/2")

#Example to lock slit motor B position, and move only A slit
#GalilCreateCSAxes("Galil","I=(A+B)/2,J=B-A","A=B-J,B=B")

#Example to swap slit blades using kinematic variable Z
#GalilCreateCSAxes("Galil","I=(A+B)/2,J=(Z==0)?B-A:A-B","A=(Z==0)?I-J/2:I+J/2,B=(Z==0)?I+J/2:I-J/2")

# GalilStartController command parameters are:
#
# 1. char *portName Asyn port for controller
# 2. char *code file to deliver to the controller we are starting. "" = use generated code
# 3. int   Burn program to EEPROM conditions
#             0 = transfer code if differs from eeprom, dont burn code to eeprom, then finally execute code thread 0.
#             1 = transfer code if differs from eeprom, burn code to eeprom, then finally execute code thread 0.
#             It is asssumed thread 0 starts all other required threads
#
# 4. int   display code. Set bit 1 to display generated code and or the code file specified.  Set bit 2 to display uploaded code
# 5. int   Thread mask.  Check these threads are running after controller code start.  Bit 0 = thread 0 and so on
#          if 0 and GalilCreateAxis appears > 0 then threads 0 to number of GalilCreateAxis is checked (good when using the generated code)

# Start the controller
GalilStartController("Galil", "", 1, 0, 0)

# Start the controller
# Example using homing routine template assembly
# GalilStartController("Galil", "$(GALIL)/GalilSup/Db/galil_Default_Header.gmc;$(GALIL)/GalilSup/Db/galil_Home_RevLimit.gmc!$(GALIL)/GalilSup/Db/galil_Home_ForwLimit.gmc!$(GALIL)/GalilSup/Db/galil_Home_Home.gmc!$(GALIL)/GalilSup/Db/galil_Home_ForwLimit.gmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.gmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.gmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.gmc!$(GALIL)/GalilSup/Db/galil_Piezo_Home.gmc;$(GALIL)/GalilSup/Db/galil_Default_Footer.gmc", 0, 0, 3)

# Start the controller
GalilStartController("RIO", "rio.gmc", 1, 0, 0)

# GalilCreateProfile command parameters are:
#
# 1. char *portName Asyn port for controller
# 2. Int maxPoints in trajectory

# Create trajectory profiles
GalilCreateProfile("Galil", 2000)

