#!/bin/bash

#From config/MASTER_RELEASE extract paths to modules that provide medm screens
export motorpath=`grep "MOTOR" config/RELEASE.local | cut -d'=' -f2`
export sscanpath=`grep "SSCAN" config/RELEASE.local | cut -d'=' -f2`
export galilpath=`grep "GALIL" config/RELEASE.local | cut -d'=' -f2`

#From module top, add offset path to medm screens
export motorpath=$motorpath/motorApp/op/adl
export sscanpath=$sscanpath/sscanApp/op/adl
export galilpath=$galilpath/GalilSup/op/adl

#Check provided arguments for record prefix
if [ -z "$1" ]
then
      # No args provided, default
      export RECPREFIX=DMC01:
else
      # Record prefix provided as argument 1
      export RECPREFIX=$1
fi

export EPICS_DISPLAY_PATH=$motorpath:$galilpath:$sscanpath
# R = Record name for digital IO not including byte/word, and bit number
# Digital IO naming
# $(DMC)$(R)<Byte or word num><Type Bo or Bi><Bit>
medm -x -macro "R=Galil,DMC=$RECPREFIX,IOC=IOC01:,M1=A,M2=B,M3=C,M4=D,M5=E,M6=F,M7=G,M8=H,M9=I,M10=J,M11=K,M12=L,M13=M,M14=N,M15=O,M16=P, AMP1=AD, AMP2=EH" galil_dmc_ctrl.adl &
