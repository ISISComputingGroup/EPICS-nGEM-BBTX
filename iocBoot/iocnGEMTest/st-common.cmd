
##ISIS## Run IOC initialisation 
< $(IOCSTARTUP)/init.cmd

## Device simulation mode IP configuration
drvAsynIPPortConfigure("IP1", "$(IPADDR=localhost:61000)")
asynOctetSetOutputEos("IP1",0,"\n")
# we do not set an input EOS as this varies with command

nGEMConfigure("P1","IP1")

## Load record instances

##ISIS## Load common DB records 
< $(IOCSTARTUP)/dbload.cmd

## Load our record instances
dbLoadRecords("$(TOP)/db/nGEM.db","P=$(MYPVPREFIX),Q=$(IOCNAME):,RECSIM=$(RECSIM=0),DISABLE=$(DISABLE=0),PORT=P1")
dbLoadRecords("$(TOP)/db/nGEM_settings.db","P=$(MYPVPREFIX),Q=$(IOCNAME):,RECSIM=$(RECSIM=0),DISABLE=$(DISABLE=0),PORT=P1")
dbLoadRecords("$(TOP)/db/nGEM_stats.db","P=$(MYPVPREFIX),Q=$(IOCNAME):,RECSIM=$(RECSIM=0),DISABLE=$(DISABLE=0),PORT=P1")

# Load asyn record
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=$(MYPVPREFIX),R=ASYN,PORT=IP1,ADDR=0,OMAX=256,IMAX=256")


##ISIS## Stuff that needs to be done after all records are loaded but before iocInit is called 
< $(IOCSTARTUP)/preiocinit.cmd

cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start any sequence programs
#seq sncxxx,"user=faa59"

##ISIS## Stuff that needs to be done after iocInit is called e.g. sequence programs 
< $(IOCSTARTUP)/postiocinit.cmd

