# Galil-3-6


ASYN based EPICS driver for Galil products

# Notes

If using RS232 communication on Microsoft Windows, need XON/XOFF flow control enabled via switches on Galil controller or else uploading
a program to the controller times out and fails. However, all other read/write communication works fine without flow control enabled.

## AUTO-GENERATED HOME STEPS
1. Driver starts jog in direction indicated by HOMR, HOMF
2. AutoGen Galil home code jogs off limit switch, or skip
3. AutoGen Galil home code jogs to find home switch active, or skip
4. AutoGen Galil home code jogs to find requested home switch edge, or skip
5. AutoGen Galil home code jogs to find encoder index, or skip
6. AutoGen Galil home code messages controller when home completed successfully

UseSwitch (Limits&home are switches) = Yes (For stages with limits, home switches)   
Begins on step 2   
Home search direction away from limit   
Find index direction away from limit  
UseSwitch = No (For rotary stages without limits)  
Find index direction indicated by HOMR, HOMF  
Begins on step 5  
useIndex = Yes  
Includes step 5  
useIndex = No  
Excludes step 5  

Use Limits as Home Switch = Yes  
Home switch is not used, limits used as home instead  
Skip steps 3, 4  

Use Limits as Home Switch = No  
Home switch is used  

MOTOR/LIMIT DIRECTION CONSISTENCY & WRONG LIMIT PROTECTION (WLP)
================================================================
1. Commisioning partly involves verifying motor direction is consistent with limit orientation  
   When the motor is moving forward, the stage must be travelling toward the forward limit  
   When the motor is moving reverse, the stage must be travelling toward the reverse limit  
2. Verifying motor/limit direction consistency involves both the hardware (wiring) and software  
   (motor, encoder selection) configuration  
3. For hardware and software configurations where the motor/limit direction consistency is not known,  
   it is NOT SAFE to rely on WLP to avoid stage damage when the ioc is started with the stage already  
   on a limit  
4. For hardware and software configurations where the motor/limit direction consistency is not known,  
   it IS SAFE to rely on WLP to avoid stage damage when the ioc is started with the stage clear of  
   both limits  
5. The motor/limit consistency has the states unknown, consistent and not consistent  
6. The motor/limit consistency check PV is $(P)$(M)_LIMITCONSISTENT_STATUS it is in motor extras db  
7. At IOC start, the motor/limit consistency for an axis is set to unknown  
8. At stage interaction with limits, the motor/limit consistency will be set to  
   consistent or not consistent.  The motor/limit consistency check works with switch transitions,  
   not switch states (refer point 3).  Reversing direction off a limit is a common operation that  
   must be allowed at ioc start, before motor/limit consistency is confirmed by this driver software   
9. If enabled, wrong limit protection will stop a motor when the motor/limit consistency is set  
   to not consistent, and a limit is active and enabled  
10. WLP can be enabled at all times with no interactions with normal or home operations   
    (refer to 8)  
