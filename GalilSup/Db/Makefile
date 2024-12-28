TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this

DB += $(patsubst ../%, %, $(wildcard ../*.template))
DB += $(patsubst ../%, %, $(wildcard ../*.dmc))

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

# Galil program code for e.g. homing, installed to $(TOP)/gmc

# Tips for writing your own code from scratch
# 1. Recommend altering generated code instead of writing code from scratch.  Generated code is dumped to file at iocBoot
# 2. Homing programs should send home<Axis> = 0 at end of homing process (home process flag) (eg. MG "homeA", homeA)
# 3. Homing programs should send homed<Axis> = 1 at end of successful homing process (homed status flag)
# 4. LIMSWI programs for axis should send home<Axis> = 0 and should send homed<Axis> = 0 at software limit violation
# 5. LIMSWI programs for axis using limits as home must not send home<Axis> = 0 at hardware limits violation
# 6. Thread programs for axis not using limits as home should send home<Axis> = 0 at hardware limits violation
# 7. If using single thread avoid use of blocking commands (eg. WT100, AD, etc)

# Example galil code templates
#GMC += galil_Default_Footer.gmc  galil_Default_Header.gmc  galil_Home_Home.gmc  galil_Home_ForwLimit.gmc
#GMC += galil_Home_RevLimit.gmc  galil_Piezo_Home.gmc

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

