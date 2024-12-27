# Autosave monitor sets for DMC (digital motor controllers)

##################################################################################################
# Configuration settings
# Configure these settings for site

# Asyn port name used in autosave file name generation
epicsEnvSet("PORT", "DMC01")

##################################################################################################
# Derived configuration settings

# Record prefix derived from asyn port name
epicsEnvSet("P", "$(PORT):")

##################################################################################################

< createDMCMonitorSet.cmd

