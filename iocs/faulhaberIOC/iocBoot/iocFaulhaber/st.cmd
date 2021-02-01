#!../../bin/linux-x86_64/faulhaber

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/faulhaber.dbd"
faulhaber_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=faulhaber:")

## 
< MCDC2805.cmd
#< MCBL2805.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("faulhaber:")

# Boot complete

## outcomment this for the MCBL2805 controller:
## use the "In-Position" mode
#dbpf "faulhaber:m1.RMOD","3"
## RDBD at least 1 in order to avoid retrys
#dbpf "faulhaber:m1.RDBD","1"
## 3 seconds delay for the motor to reckon a movement as completed
#dbpf "faulhaber:m1.DLY","3"
