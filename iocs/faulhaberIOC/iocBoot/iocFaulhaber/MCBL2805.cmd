drvAsynIPPortConfigure("P5", "nport1:4005",0,0,0)
# The MCDC2805 driver does not set end of string (EOS).
asynOctetSetInputEos("P5",0,"\r")
asynOctetSetOutputEos("P5",0,"\r")

###
# MCBL2805CreateController(
#    motor port (will be created),
#    asyn port (must already exist),
#    controller ID,
#    moving poll period (ms),
#    idle poll period (ms)
###
MCBL2805CreateController("mcbl2805", "P5", 0, 250, 0)

dbLoadTemplate("MCBL2805.substitutions")
