#!/bin/bash

#From config/MASTER_RELEASE extract paths to modules that provide qegui screens
export galilpath=`grep "GALIL" config/GALILRELEASE | cut -d'=' -f2`

#From module top, add offset path to qegui screens
export galilpath=$galilpath/GalilSup/op/ui

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
qegui -style ${QTSTYLE} -e -m "RIO=$RECPREFIX" galil_rio_ctrl.ui &

