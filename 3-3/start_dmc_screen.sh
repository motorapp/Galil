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
# R = Record name for digital IO not including byte/word, and bit number
# Digital IO naming
# $(DMC)$(R)<Byte or word num><Type Bo or Bi><Bit>
medm -x -macro "R=Galil,DMC=DMC01:,IOC=IOC01:,M1=A,M2=B,M3=C,M4=D,M5=E,M6=F,M7=G,M8=H,M9=I,M10=J,M11=K,M12=L,M13=M,M14=N,M15=O,M16=P" galil_dmc_ctrl.adl &
