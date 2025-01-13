#!/bin/bash

#From config/MASTER_RELEASE extract paths to modules that provide qegui screens
export galilpath=`grep "GALIL" config/RELEASE.local | cut -d'=' -f2`

#From module top, add offset path to qegui screens
export galilpath=$galilpath/GalilSup/op/ui

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
qegui -style ${QTSTYLE} -e -m "DMC=$RECPREFIX,M1=A,M2=B,M3=C,M4=D,M5=E,M6=F,M7=G,M8=H,M9=I,M10=J,M11=K,M12=L,M13=M,M14=N,M15=O,M16=P" galil_dmc_ctrl.ui &

