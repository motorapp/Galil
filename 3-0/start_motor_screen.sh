#!/bin/bash
export EPICS_DISPLAY_PATH=/beamline/perforce/Dev/SBS/4_Controls/4_3_Network_Infrastructure/4_3_1_Comms_Common_Services/sw/epics/motorR6-8/motorApp/op/adl:/beamline/perforce/Dev/SBS/4_Controls/4_3_Network_Infrastructure/4_3_1_Comms_Common_Services/sw/device_drivers/Galil/3-0/GalilSup/op/adl:/beamline/perforce/Dev/SBS/4_Controls/4_3_Network_Infrastructure/4_3_1_Comms_Common_Services/sw/epics/sscan-2-8-1/sscanApp/op/adl
medm -x -macro "R=Galil,DMC=DMC01:,IOC=IOC01:,M1=A,M2=B,M3=C,M4=D,M5=E,M6=F,M7=G,M8=H,M9=I,M10=J,M11=K,M12=L,M13=M,M14=N,M15=O,M16=P" galil_ctrl_extras.adl &

