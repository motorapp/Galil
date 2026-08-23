#!/bin/bash

#Extract paths to modules that provide QEGUI screens from configure/RELEASE.local

# 1. Extract the raw value of SUPPORT (e.g., SUPPORT=/path/to/support)
raw_support=$(grep "SUPPORT=" configure/RELEASE.local | cut -d'=' -f2)
export SUPPORT=${raw_support%$'\r'}

# 2. Extract paths
galilpath=$(grep "GALIL=" configure/RELEASE.local | cut -d'=' -f2)

# 3. Strip Windows carriage returns (\r)
galilpath=${galilpath%$'\r'}

# 4. MACRO SUBSTITUTION: Replace the literal '$(SUPPORT)' text with the $SUPPORT variable value
export galilpath="${galilpath//\$\(SUPPORT\)/$SUPPORT}"

# 5. Append relative QEGUI screen offsets
export galilpath="$galilpath/GalilSup/op/ui"

#QEGUI path to screens
export QE_UI_PATH=$galilpath

#Determine Qt version
export QTVERSION=`qmake -v | tail --lines 1 | cut -d " " -f4 | cut -c1`

#Determine Qt style to use from version
if [[ $QTVERSION = "4" ]]; then
#Qt4 detected
   export QTSTYLE="plastique"
elif [[ $QTVERSION = "5" ]]; then
#Qt5 detected
   export QTSTYLE="fusion"
else
#Unknown assume Qt5
   export QTSTYLE="fusion"
fi

#Check provided arguments for record prefix
if [ -z "$1" ]
then
      # No args provided, default
      export RECPREFIX=RIO01:
else
      # Record prefix provided as argument 1
      export RECPREFIX=$1
fi

#Invoke QEGUI
qegui -style ${QTSTYLE} -m "RIO=$RECPREFIX" -e galil_rio_ctrl.ui &

