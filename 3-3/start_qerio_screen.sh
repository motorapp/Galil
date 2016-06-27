#!/bin/bash

#From config/MASTER_RELEASE extract paths to modules that provide qegui screens
export galilpath=`grep "GALIL" config/GALILRELEASE | cut -d'=' -f2`

#From module top, add offset path to qegui screens
export galilpath=$galilpath/GalilSup/op/ui

export QE_UI_PATH=$motorpath:$galilpath:$sscanpath
qegui -style plastique -e -m "R=Galil,RIO=RIO01:" galil_rio_ctrl.ui &

