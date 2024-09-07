##################################################################################################
# Create an autosave monitor set for a DMC instance
##################################################################################################
# Save motor positions every 5 seconds
create_monitor_set("GalilTest_$(PORT)Positions.req", 5,"P=$(P)")
# Save motor settings every 30 seconds
create_monitor_set("GalilTest_$(PORT)Settings.req", 30,"IOC=$(IOCPREFIX),P=$(P)")
# Save kinematics every 30 seconds
create_monitor_set("GalilTest_$(PORT)Kinematics.req", 30,"P=$(P)")
