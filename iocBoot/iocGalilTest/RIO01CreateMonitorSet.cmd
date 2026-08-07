# Autosave monitor sets for RIO (remote I/O plc)

##################################################################################################
# Configuration settings
# Configure these settings for site

# Asyn port name used in autosave file name generation
epicsEnvSet("PORT", "RIO01")

##################################################################################################
# Derived configuration settings

# Record prefix derived from asyn port name
epicsEnvSet("P", "$(PORT):")

##################################################################################################

# Save rio settings every 30 seconds
create_monitor_set("GalilTest_$(PORT)Settings.req", 30,"P=$(P)")

