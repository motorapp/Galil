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

#Determine QEGUI version
export QEGUIVERSION=`qegui --version | head --lines 1 | cut -d " " -f7 | cut -c1`

#Create galil_motors.ui given QEGUI version
if [[ $QEGUIVERSION = "3" ]]; then
#QEGUI 3.x detected
   cp GalilSup/op/ui/galil_motors_v3.ui GalilSup/op/ui/galil_motors.ui
elif [[ $QEGUIVERSION = "4" ]]; then
#QEGUI 4.x detected
   cp GalilSup/op/ui/galil_motors_v4.ui GalilSup/op/ui/galil_motors.ui
fi

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
      export RECPREFIX=DMC01:
else
      # Record prefix provided as argument 1
      export RECPREFIX=$1
fi

#Invoke QEGUI
qegui -style ${QTSTYLE} -m "DMC=$RECPREFIX,M1=A,M2=B,M3=C,M4=D,M5=E,M6=F,M7=G,M8=H,M9=I,M10=J,M11=K,M12=L,M13=M,M14=N,M15=O,M16=P,AMP1=AD,AMP2=EH" -e galil_dmc_ctrl.ui &

