#!/bin/bash

#From config/MASTER_RELEASE extract paths to modules that provide qegui screens
export galilpath=`grep "GALIL" config/GALILRELEASE | cut -d'=' -f2`

#From module top, add offset path to qegui screens
export galilpath=$galilpath/GalilSup/op/ui

#QEGUI path to screens
export QE_UI_PATH=$galilpath

#Determine Qt version
export QTVERSION=`echo $QTDIR | cut -b 14`

#Determine Qt style to use from version
if [ ${QTVERSION} = "4" ]; then
#Qt4 detected
   export QTSTYLE="plastique"
elif [ ${QTVERSION} = "5" ]; then
#Qt5 detected
   export QTSTYLE="fusion"
else
#Unknown assume Qt4
   export QTSTYLE="plastique"
fi

#Invoke QEGUI
qegui -style ${QTSTYLE} -e -m "DMC=DMC01:,M1=A,M2=B,M3=C,M4=D,M5=E,M6=F,M7=G,M8=H,M9=I,M10=J,M11=K,M12=L,M13=M,M14=N,M15=O,M16=P" galil_dmc_ctrl.ui &

