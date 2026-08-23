#!/bin/bash

#Extract paths to modules that provide medm screens from configure/RELEASE.local

# 1. Extract the raw value of SUPPORT (e.g., SUPPORT=/path/to/support)
raw_support=$(grep "SUPPORT=" configure/RELEASE.local | cut -d'=' -f2)
export SUPPORT=${raw_support%$'\r'}

# 2. Extract paths
motorpath=$(grep "MOTOR=" configure/RELEASE.local | cut -d'=' -f2)
sscanpath=$(grep "SSCAN=" configure/RELEASE.local | cut -d'=' -f2)
galilpath=$(grep "GALIL=" configure/RELEASE.local | cut -d'=' -f2)

# 3. Strip Windows carriage returns (\r)
motorpath=${motorpath%$'\r'}
sscanpath=${sscanpath%$'\r'}
galilpath=${galilpath%$'\r'}

# 4. MACRO SUBSTITUTION: Replace the literal '$(SUPPORT)' text with the $SUPPORT variable value
export motorpath="${motorpath//\$\(SUPPORT\)/$SUPPORT}"
export sscanpath="${sscanpath//\$\(SUPPORT\)/$SUPPORT}"
export galilpath="${galilpath//\$\(SUPPORT\)/$SUPPORT}"

# 5. Append relative MEDM screen offsets
export motorpath="$motorpath/motorApp/op/adl"
export sscanpath="$sscanpath/sscanApp/op/adl"
export galilpath="$galilpath/GalilSup/op/adl"

#Check provided arguments for record prefix
if [ -z "$1" ]
then
      # No args provided, default
      export RECPREFIX=RIO01:
else
      # Record prefix provided as argument 1
      export RECPREFIX=$1
fi

export EPICS_DISPLAY_PATH=$motorpath:$galilpath:$sscanpath
# R = Record name for digital IO not including byte/word, and bit number
# Digital IO naming
# $(DMC)$(R)<Byte or word num><Type Bo or Bi><Bit>
medm -x -macro "R=Galil,RIO=$RECPREFIX" galil_rio_ctrl.adl &
