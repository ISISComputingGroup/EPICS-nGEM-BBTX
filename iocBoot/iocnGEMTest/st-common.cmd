
##ISIS## Run IOC initialisation 
< $(IOCSTARTUP)/init.cmd

epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "500000")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")
asynSetMinTimerPeriod(0.001)

## Device simulation mode IP configuration
drvAsynIPPortConfigure("NGEMIP","$(IPADDR=localhost:61000)")
asynOctetSetOutputEos("NGEMIP",0,"\n")
## we do not set an input EOS, we look to match ngem> in the driver
nGEMConfigure("NGEM","NGEMIP")

## Load record instances
dbLoadRecords("$(TOP)/db/nGEMAD.template","P=$(MYPVPREFIX),R=$(IOCNAME):,PORT=NGEM,ADDR=0,TIMEOUT=1")

##ISIS## Load common DB records 
< $(IOCSTARTUP)/dbload.cmd

## Load our record instances
dbLoadRecords("$(TOP)/db/nGEM.db","P=$(MYPVPREFIX),Q=$(IOCNAME):,RECSIM=$(RECSIM=0),DISABLE=$(DISABLE=0),PORT=NGEM")
dbLoadRecords("$(TOP)/db/nGEM_settings.db","P=$(MYPVPREFIX),Q=$(IOCNAME):,RECSIM=$(RECSIM=0),DISABLE=$(DISABLE=0),PORT=NGEM")
dbLoadRecords("$(TOP)/db/nGEM_stats.db","P=$(MYPVPREFIX),Q=$(IOCNAME):,RECSIM=$(RECSIM=0),DISABLE=$(DISABLE=0),PORT=NGEM")

# Load asyn record
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=$(MYPVPREFIX),R=ASYN,PORT=NGEMIP,ADDR=0,OMAX=256,IMAX=256")

NDStdArraysConfigure("Image1", 3, 0, "NGEM", 0, 0)
dbLoadRecords("NDStdArrays.template", "P=$(MYPVPREFIX),R=$(IOCNAME):image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=NGEM,TYPE=Int16,FTVL=SHORT,NELEMENTS=150000,ENABLED=1")

NDPvaConfigure("PVA", 3, 0, "NGEM", 0, "v4pvname")
dbLoadRecords("NDPva.template", "P=$(MYPVPREFIX),R=$(IOCNAME):V4:,PORT=PVA,ADDR=0,TIMEOUT=1,NDARRAY_PORT=NGEM,ENABLED=1")


##ISIS## Stuff that needs to be done after all records are loaded but before iocInit is called 
< $(IOCSTARTUP)/preiocinit.cmd

cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start any sequence programs
#seq sncxxx,"user=faa59"

##ISIS## Stuff that needs to be done after iocInit is called e.g. sequence programs 
< $(IOCSTARTUP)/postiocinit.cmd

