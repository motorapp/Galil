#!/bin/bash

#From config/MASTER_RELEASE extract paths to modules that provide medm screens
export motorpath=`grep "MOTOR" config/GALILRELEASE | cut -d'=' -f2`
export sscanpath=`grep "SSCAN" config/GALILRELEASE | cut -d'=' -f2`
export galilpath=`grep "GALIL" config/GALILRELEASE | cut -d'=' -f2`

#From module top, add offset path to medm screens
export motorpath=$motorpath/motorApp/op/adl
export sscanpath=$sscanpath/sscanApp/op/adl
export galilpath=$galilpath/GalilSup/op/adl

export EPICS_DISPLAY_PATH=$motorpath:$galilpath:$sscanpath
medm -x -macro "P=RIO01:, R=Galil" galil_ctrl_io.adl &
