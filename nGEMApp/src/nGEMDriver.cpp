#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <map>
#include <iomanip>
#include <sstream>
#include <sys/timeb.h>
#include <numeric>

#ifdef _WIN32
#include <process.h>
#endif /* _WIN32 */

#include <boost/algorithm/string.hpp>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <initHooks.h>

#include <macLib.h>
#include <epicsGuard.h>

#include "pcrecpp.h"

#include "envDefs.h"
#include "errlog.h"

#include "utilities.h"
#include "pugixml.hpp"

#include <asynOctetSyncIO.h>

#include "nGEMDriver.h"

#include <epicsExport.h>

static volatile bool iocRunning = false;

static void myHookFunction(initHookState state)
{
  switch(state) {
    case initHookAfterIocRunning:
      iocRunning = true;
      break;

    default:
      break;
  }
}

int nGEMDriver::addParam(const char* name, ParamGroup group, asynParamType type)
{
	int id;
	std::map<std::string,nGEMParam*>* pMap;
	std::string db_name(name), suffix;
	if (group == GroupSettings)
	{
		pMap = &m_settings;
		suffix = "_SETTING";
	}
	else if (group == GroupStats)
	{
		pMap = &m_stats;	
		suffix = "_STAT";
	}
	else
	{
		throw std::runtime_error("unknown");
	}
	boost::to_upper(db_name);
	db_name += suffix;
	std::replace(db_name.begin(), db_name.end(), ' ', '_');
	std::replace(db_name.begin(), db_name.end(), '/', '_');
	if (createParam(db_name.c_str(), type, &id) == asynSuccess)
	{
	    (*pMap)[name] = new nGEMParam(this, id, type);
	}
	else
	{
		throw std::runtime_error("problem");
	}
	return id;
}

static const char *driverName="nGEMDriver";

void nGEMDriver::setADAcquire(int acquire)
{
    int adstatus;
    int acquiring;
    int imageMode;
    asynStatus status = asynSuccess;

    /* Ensure that ADStatus is set correctly before we set ADAcquire.*/
    getIntegerParam(ADStatus, &adstatus);
    getIntegerParam(ADAcquire, &acquiring);
    getIntegerParam(ADImageMode, &imageMode);
      if (acquire && !acquiring) {
        setStringParam(ADStatusMessage, "Acquiring data");
        setIntegerParam(ADStatus, ADStatusAcquire); 
        setIntegerParam(ADAcquire, 1); 
      }
      if (!acquire && acquiring) {
        setIntegerParam(ADAcquire, 0); 
        setStringParam(ADStatusMessage, "Acquisition stopped");
        if (imageMode == ADImageContinuous) {
          setIntegerParam(ADStatus, ADStatusIdle);
        } else {
          setIntegerParam(ADStatus, ADStatusAborted);
        }
      }
}

/// Constructor for the isisdaeDriver class.
/// Calls constructor for the asynPortDriver base class.
/// \param[in] dcomint DCOM interface pointer created by lvDCOMConfigure()
/// \param[in] portName @copydoc initArg0
nGEMDriver::nGEMDriver(const char *portName, const char* ipPortName)
	: ADDriver(portName,
		1, /* maxAddr */
		NUM_NGEM_PARAMS,
		0, // maxBuffers
		0, // maxMemory
		asynInt32Mask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
		asynInt32Mask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
		ASYN_CANBLOCK, /* asynFlags.  This driver can block but it is not multi-device */
		1, /* Autoconnect */
		0, /* Default priority */
		0),	/* Default stack size*/
	m_pRaw(NULL), m_old_acquiring(0)
					
{					
	int status;
    const char *functionName = "nGEMDriver";
	strcpy(m_instRunNumber, "00000000");
//    status = pasynOctetSyncIO->connect(ipPortName, 0, &m_detPort, NULL);
//    if (status) {
//      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
//      "%s: cannot connect to nGEM detector\n",
//      functionName);
//    }	

	m_det = new asynOctetClient(ipPortName, 0, (const char*)0, 0.01);
	
	createParam(P_startString, asynParamInt32, &P_start); // FIRST_NGEM_PARAM
	createParam(P_stopString, asynParamInt32, &P_stop);
	createParam(P_updatetofString, asynParamInt32, &P_updatetof);
	createParam(P_commandString, asynParamOctet, &P_command);
	createParam(P_commandReplyString, asynParamOctet, &P_commandReply);
	createParam(P_1dtofString, asynParamFloat64Array, &P_1dtof);
	createParam(P_1dsxString, asynParamFloat64Array, &P_1dsx);
	createParam(P_1dsyString, asynParamFloat64Array, &P_1dsy);
	createParam(P_2dfxyString, asynParamFloat64Array, &P_2dfxy);
	createParam(P_2dlxyString, asynParamFloat64Array, &P_2dlxy);
	createParam(P_2davgString, asynParamFloat64Array, &P_2davg);
	createParam(P_2drngString, asynParamFloat64Array, &P_2drng);
	createParam(P_displayString, asynParamInt32, &P_display);
	createParam(P_statsString, asynParamInt32, &P_stats);
	createParam(P_settingsString, asynParamInt32, &P_settings);
	createParam(P_1dsxtString, asynParamFloat64Array, &P_1dsxt);
	createParam(P_1dsytString, asynParamFloat64Array, &P_1dsyt);
	createParam(P_tofString, asynParamFloat64Array, &P_tof);
	createParam(P_ntofString, asynParamInt32, &P_ntof);
	createParam(P_dataModeString, asynParamInt32, &P_dataMode);
	createParam(P_instRunNumberString, asynParamOctet, &P_instRunNumber);
	createParam(P_instNameString, asynParamOctet, &P_instName);
	createParam(P_archiveDirString, asynParamOctet, &P_archiveDir);
    createParam(P_fileDirString, asynParamOctet, &P_fileDir);
	createParam(P_dirString, asynParamOctet, &P_dir); // LAST_NGEM_BASE_PARAM
	
    // need to do all explicit createParam before any addParam
    
	// settings
    P_inst = addParam("inst", GroupSettings, asynParamOctet);
    P_basepath = addParam("basepath", GroupSettings, asynParamOctet);
    addParam("logging", GroupSettings, asynParamOctet); // on or off
    addParam("host", GroupSettings, asynParamOctet);
    addParam("tcp", GroupSettings, asynParamInt32);
    P_tofmin = addParam("tofmin", GroupSettings, asynParamFloat64);
    P_tofmax = addParam("tofmax", GroupSettings, asynParamFloat64);
    P_tofwidth = addParam("tofwidth", GroupSettings, asynParamFloat64);
    addParam("fdtofmin", GroupSettings, asynParamInt32);
    addParam("fdtofmax", GroupSettings, asynParamInt32);
    addParam("f2dabsxmin", GroupSettings, asynParamInt32);
    addParam("f2dabsymin", GroupSettings, asynParamInt32);
    addParam("f2dabsxmax", GroupSettings, asynParamInt32);
    addParam("f2dabsymax", GroupSettings, asynParamInt32);
    addParam("fdxmin", GroupSettings, asynParamInt32);
    addParam("fdxmax", GroupSettings, asynParamInt32);
    addParam("fdymin", GroupSettings, asynParamInt32);
    addParam("fdymax", GroupSettings, asynParamInt32);
    addParam("fsxmin", GroupSettings, asynParamInt32);
    addParam("fsxmax", GroupSettings, asynParamInt32);
    addParam("fsymin", GroupSettings, asynParamInt32);
    addParam("fsymax", GroupSettings, asynParamInt32);
    addParam("normalize2drng", GroupSettings, asynParamOctet); // on or off

	// stats
    P_daqStatus = addParam("Daq Status", GroupStats, asynParamOctet); // Stop
    addParam("Load Status", GroupStats, asynParamOctet); // Stop
    P_runNo = addParam("Run No", GroupStats, asynParamInt32);
    addParam("Logging Prefix", GroupStats, asynParamOctet);
    addParam("Loading Run No", GroupStats, asynParamInt32);
    addParam("Loading CPU", GroupStats, asynParamInt32);
    addParam("Loading Module", GroupStats, asynParamInt32);
    addParam("Loading Detail", GroupStats, asynParamInt32);
    addParam("Loading file", GroupStats, asynParamOctet);
    addParam("Total Events", GroupStats, asynParamInt32);
    addParam("T0 Events", GroupStats, asynParamInt32);
    addParam("T0 Frame Loss Accumulated", GroupStats, asynParamInt32);
    addParam("T0 Frame Loss Maximum", GroupStats, asynParamInt32);
    addParam("T0 Frame Loss Detected", GroupStats, asynParamInt32);
    addParam("T0 Event Loss Accumulated", GroupStats, asynParamInt32);
    addParam("T0 Event Loss Maximum", GroupStats, asynParamInt32);
    addParam("T0 Event Loss Detected", GroupStats, asynParamInt32);
    addParam("T0 FIFO Event Detected", GroupStats, asynParamInt32);
    addParam("T0 FIFO Event Maximum", GroupStats, asynParamInt32);
    addParam("T0 Total Loss Count", GroupStats, asynParamInt32);
    addParam("Time Events", GroupStats, asynParamInt32);
    addParam("Pulse Events", GroupStats, asynParamInt32);
    addParam("Coincidence Events", GroupStats, asynParamInt32);
    addParam("Unknown Events", GroupStats, asynParamInt32);
    addParam("TOF Data Out Of Rnage", GroupStats, asynParamInt32);
    addParam("Filter TOF", GroupStats, asynParamInt32);
    addParam("Filter ABS", GroupStats, asynParamInt32);
    addParam("Filter DX/DY", GroupStats, asynParamInt32);
    addParam("Filter SX/SY", GroupStats, asynParamInt32);

    // area detector defaults
	int maxSizeX = 128, maxSizeY = 128;
	NDDataType_t dataType = NDFloat64; // data type for each frame
    status =  setStringParam (ADManufacturer, "nGEM");
    status |= setStringParam (ADModel, "nGEM");
    status |= setIntegerParam(ADMaxSizeX, maxSizeX);
    status |= setIntegerParam(ADMaxSizeY, maxSizeY);
    status |= setIntegerParam(ADMinX, 0);
    status |= setIntegerParam(ADMinY, 0);
    status |= setIntegerParam(ADBinX, 1);
    status |= setIntegerParam(ADBinY, 1);
    status |= setIntegerParam(ADReverseX, 0);
    status |= setIntegerParam(ADReverseY, 0);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeY, maxSizeY);
    status |= setIntegerParam(NDArraySizeX, maxSizeX);
    status |= setIntegerParam(NDArraySizeY, maxSizeY);
    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType, dataType);
    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setDoubleParam (ADAcquireTime, .001);
    status |= setDoubleParam (ADAcquirePeriod, .005);
    status |= setIntegerParam(ADNumImages, 100);

	setIntegerParam(ADAcquire, 0);
    setStringParam(P_dir, "");
    setStringParam(P_archiveDir, "");
    setStringParam(P_inst, "");
    setStringParam(P_basepath, "");
    setStringParam(P_fileDir, "");
	setIntegerParam(P_dataMode, 0);
	setStringParam(P_instRunNumber, "00000000");
	setStringParam(P_instName, "UNKNOWN");	

    if (status) {
        printf("%s: unable to set nGEM parameters\n", functionName);
        return;
    }
	readSettings();
	readStats();
	computeTOF();

    // Create the thread for background tasks (not used at present, could be used for I/O intr scanning) 
    if (epicsThreadCreate("nGEMPoller1",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollerThreadC1, this) == 0)
    {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
}

void nGEMDriver::computeTOF()
{
    double tofmin, tofmax, tofwidth;
	int ntof;
    asynStatus status;
    // ntof is used to monitor alarm status of comms. 
    if (getParamStatus(P_tofmin, &status) == asynSuccess && status != asynSuccess)
    {
        setParamStatus(P_ntof, status);
        return;        
    }
	getDoubleParam(P_tofmin, &tofmin);
	getDoubleParam(P_tofmax, &tofmax);
	getDoubleParam(P_tofwidth, &tofwidth);
	if (tofmax > tofmin && tofwidth != 0.0)
	{
        ntof = (tofmax - tofmin) / tofwidth;
	    setIntegerParam(P_ntof, ntof);
		m_tof.resize(ntof);
		for(int i=0; i<ntof; ++i)
		{
			m_tof[i] = tofmin + i * tofwidth;
		}
		if (iocRunning && m_tof != m_tof_old)
		{
			m_tof_old = m_tof;
		    doCallbacksFloat64Array(&(m_tof[0]), m_tof.size(), P_tof, 0);
		}
	}
}	
	
asynStatus nGEMDriver::writeData(const char* output, int timeout)
{
  size_t nwrite = 0;
  asynStatus status = m_det->write(output, strlen(output), &nwrite);
  return status;
}

asynStatus nGEMDriver::writeReadData(const char* output, char* input, size_t inputLen)
{
  std::string ngem_term("ngem>");
  int eomReason;
  size_t nwrite = 0, nread = 0, nread_total = 0;
  input[0] = '\0';
  asynStatus status = m_det->write(output, strlen(output), &nwrite);
  while(status == asynSuccess || status == asynTimeout) {
	  nread = 0;
      status = m_det->read(input + nread_total, inputLen - nread_total, &nread, &eomReason);
	  nread_total += nread;
	  if ( nread_total >= ngem_term.size() && !strncmp(input + nread_total - ngem_term.size(), ngem_term.c_str(), ngem_term.size()) )
	  {
		  break;
	  }
  }
//  std::cout << "OUT:\"" << output << "\" nout=" << nwrite << std::endl;
  if (status != asynSuccess && status != asynTimeout)
  {
	  return status;
  }
  if (nread_total < inputLen)
  {
	  input[nread_total] = '\0';
  }
  else
  {
      input[inputLen-1] = '\0';
  }
//  std::cerr << "IN:\"" << input << "\" nin=" << nread_total << std::endl;
//  std::cout << "IN: nin=" << nread_total << std::endl;
  return asynSuccess;
}

asynStatus nGEMDriver::readSettings()
{
	return readPairs("setting", m_settings);
}

asynStatus nGEMDriver::readStats()
{
	return readPairs("stat", m_stats);
}


void nGEMDriver::getFileDir()
{
	char basepath[512], dir[128];
	getStringParam(P_dir, sizeof(dir), dir);
	getStringParam(P_basepath, sizeof(basepath), basepath);
    pcrecpp::RE re("/cygdrive/([a-zA-Z])/(.*)");
    std::string drive, basedir;
    re.FullMatch(basepath, &drive, &basedir);
	std::replace(basedir.begin(), basedir.end(), '/', '\\');
	std::string basepath_s = drive + ":\\" + basedir + "\\" + dir;
    setStringParam(P_fileDir, basepath_s.c_str());
}

// copyData() doesn't work on linux
void nGEMDriver::copyData()
{
	static const char* copycmd_ = getenv("NGEMCMD");
	static const char* comspec = getenv("COMSPEC");
	char dir[128], inst[10], basepath[512], instname[20], archiveDir[128];
	std::string copycmd;
    dir[0] = archiveDir[0] = '\0';
	if (copycmd_ == NULL) 
	{
		return;
	}
	if ( (getStringParam(P_dir, sizeof(dir), dir) != asynSuccess) || (strlen(dir) == 0) )
	{
		return;
	}
	if ( (getStringParam(P_archiveDir, sizeof(archiveDir), archiveDir) != asynSuccess) || (strlen(archiveDir) == 0) )
	{
		return;
	}
	copycmd = copycmd_;
	getStringParam(P_inst, sizeof(inst), inst);
	getStringParam(P_basepath, sizeof(basepath), basepath);
	getStringParam(P_instName, sizeof(instname), instname);
    pcrecpp::RE re("/cygdrive/([a-zA-Z])/(.*)");
    std::string drive, basedir, archivedir(archiveDir);
    re.FullMatch(basepath, &drive, &basedir);
	std::replace(basedir.begin(), basedir.end(), '/', '\\');
	std::replace(copycmd.begin(), copycmd.end(), '/', '\\');
	std::replace(copycmd.begin(), copycmd.end(), '/', '\\');
	std::replace(archivedir.begin(), archivedir.end(), '/', '\\');
	std::string basepath_s = drive + ":\\" + basedir;
	if (basedir.size() == 0)
	{
		return;
	}
	std::cerr << "Running " << copycmd << " for " << dir << " in " << basepath_s << std::endl;
#ifdef _WIN32
    _spawnl(_P_NOWAIT, comspec, comspec, "/c", copycmd.c_str(), basepath_s.c_str(), dir, instname, m_instRunNumber, archivedir.c_str(), NULL);
#endif /* _WIN32 */
}

asynStatus nGEMDriver::readPairs(const char* command, map_t& map)
{
  char input[1024];
  asynStatus status = writeReadData(command, input, sizeof(input));
  if (status != asynSuccess)
  {
      for(map_t::const_iterator it = map.begin(); it != map.end(); ++it)
      {
          setParamStatus(it->second->param_id, asynError);
      }
      return asynError;
  }
  std::vector<std::string> SplitVec;
  std::vector<std::string> PairVec;
  std::string str_input(input);
  boost::erase_all(str_input, "ngem>"); // remove prompt that can get embedded in reply
  boost::erase_all(str_input, "OK:"); // remove initial answer to stat command
  // now split into lines and then split each line looking for name:value
  boost::split(SplitVec, str_input, boost::is_any_of("\r\n"), boost::token_compress_on );
  for(int i=0; i<SplitVec.size(); ++i)
  {
      boost::split(PairVec, SplitVec[i], boost::is_any_of(":"), boost::token_compress_on);
	  if (PairVec.size() != 2)
	      continue;
	  const std::string& name = PairVec[0];
	  if (map.find(name) != map.end())
	  {
		  nGEMParam* param = map[name];
		  param->setValue(PairVec[1]);
	  }
	  else
	  {
		  std::cerr << "Unknown setting/stat: \"" << name << "\" from command \"" << command << "\"" << std::endl; 
	  }
  }
  return asynSuccess;
}

void nGEMDriver::applyDataMode(double* data, size_t nelements)
{
	int dataMode = 0;
	getIntegerParam(P_dataMode, &dataMode);
	for(int i=0; i<nelements; ++i)
	{
	    switch(dataMode)
	    {
		    case 0:
			    break;
			case 1:
			    data[i] = sqrt(data[i]);
				break;
			case 2:
				data[i] = log(1 + data[i]);
				break;
			default:
			    break;
		}
	}
}

// format is int32 (number of following bytes) then double precision values
asynStatus nGEMDriver::convertData(void* buffer, double* data, int nin, size_t& nout)
{
	int nbytes = *(int*)buffer;
	if (nbytes % 8 != 0)
	{
		std::cerr << "array size mismatch " << nout << std::endl;
		return asynError;
	}
	nout = nbytes / 8;
	if (nin < nout)
	{
		nout = nin;
	}
//	std::cerr << "read array size " << nout << std::endl;
	memcpy(data, (char*)buffer + 4, nout * 8);
	return asynSuccess;
}	

asynStatus nGEMDriver::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn)
{
	static char buffer[1000000];
	int function = pasynUser->reason;
	size_t n;
	asynStatus status = asynSuccess;
	bool convert = true;
	if (function < FIRST_NGEM_PARAM)
	{
		return ADDriver::readFloat64Array(pasynUser, value, nElements, nIn);
	}
	if (function == P_tof) {
		n = (nElements < m_tof.size() ? nElements : m_tof.size());
	    std::copy(value, value + n, &(m_tof[0]));
		*nIn = n;
		convert = false;
	} else if (function == P_1dsxt) {
		n = (nElements < 1024 ? nElements : 1024);
		for(int i=0; i<n; ++i)
		{
			value[i] = i;
		}
		*nIn = n;
		convert = false;
	} else if (function == P_1dsyt) {
		n = (nElements < 1024 ? nElements : 1024);
		for(int i=0; i<n; ++i)
		{
			value[i] = i;
		}
		*nIn = n;
		convert = false;
	} else if (function == P_1dsx) {
		status = writeReadData("1dsx b", buffer, sizeof(buffer));
	} else if (function == P_1dsy) {
		status = writeReadData("1dsy b", buffer, sizeof(buffer));		
	} else if (function == P_1dtof) {
		status = writeReadData("1dtof b", buffer, sizeof(buffer));		
	} else if (function == P_2dfxy) {
		status = writeReadData("2dfxy b", buffer, sizeof(buffer));		
	} else if (function == P_2dlxy) {
		status = writeReadData("2dlxy b", buffer, sizeof(buffer));		
	} else if (function == P_2davg) {
		status = writeReadData("2davg b", buffer, sizeof(buffer));		
	} else if (function == P_2drng) {
		status = writeReadData("2drng b", buffer, sizeof(buffer));		
	} else {
		return asynError;
	}
	if ( convert && (status == asynSuccess) )
	{
	    status = convertData(buffer, value, nElements, *nIn);
		applyDataMode(value, *nIn);
	}
    if (status == asynSuccess)
	{
		doCallbacksFloat64Array(value, *nIn, function, 0);
	}
    return status;
}

void nGEMDriver::pollerThreadC1(void* arg)
{
	epicsThreadSleep(1.0);	// let constructor complete
    nGEMDriver* driver = (nGEMDriver*)arg;
	if (driver != NULL)
	{
	    driver->pollerThread1();
	}
}

// -1 = unknow, 0 = stopped, 1 = running
int nGEMDriver::daqStatus()
{
	char daqStatus[20];
	if (getStringParam(P_daqStatus, sizeof(daqStatus), daqStatus) != asynSuccess)
	{
		return -1;
	}
	if (!strcmp("Running", daqStatus))
	{
		return 1;
	} 
	else if (!strcmp("Stop", daqStatus))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

void nGEMDriver::pollerThread1()
{
    static const char* functionName = "nGEMDriverPoller1";
	int acquiring = 0;
    asynStatus status;
	while(true)
	{
		epicsThreadSleep(2.0);
		lock();
		readSettings(); // check for external changes?
		readStats();
		computeTOF();
		getIntegerParam(ADAcquire, &acquiring);
        getFileDir();
		if (daqStatus() == 1 && acquiring == 0)
		{
			getStringParam(P_instRunNumber, sizeof(m_instRunNumber), m_instRunNumber);
			setADAcquire(1);
		}
		else if (daqStatus() == 0 && acquiring == 1)
		{
			setADAcquire(0);
		}
		processData();
		callParamCallbacks();
		unlock();
	}
}

asynStatus nGEMDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;
//	const char* paramName;
//	getParamName
	char output[128], input[128];
	asynStatus status = asynSuccess;
	if (function < FIRST_NGEM_PARAM)
	{
		return ADDriver::writeInt32(pasynUser, value);
	}
	else if (function == P_start)
	{
		int runNo;
		char inst[10];
		getStringParam(P_instRunNumber, sizeof(m_instRunNumber), m_instRunNumber); // record at start to avoid race condition on end
		status = writeReadData("updatetof", input, sizeof(input));
		epicsThreadSleep(0.1);
		getIntegerParam(P_runNo, &runNo); // current GEM number
		getStringParam(P_inst, sizeof(inst), inst);
		runNo += 1;
		sprintf(output, "start %d", runNo);
		status = writeReadData(output, input, sizeof(input));
		if (strncmp(input, "OK:Started", strlen("OK:Started")) != 0)
		{
			std::cerr << input << std::endl;
			return asynError;
		}
		time_t start_time;
		time(&start_time);
		char tbuffer[64], dirbuffer[256];
		strftime(tbuffer, sizeof(tbuffer), "%Y%m%d", localtime(&start_time));
		epicsSnprintf(dirbuffer, sizeof(dirbuffer), "%s%06d_%s", inst, runNo, tbuffer);
		setStringParam(P_dir, dirbuffer);
		readStats();
		while(daqStatus() != 1)
		{
			epicsThreadSleep(1.0);
			readStats();
		}
		setADAcquire(1);
		std::cerr << "START: run " << runNo << " in directory " << dirbuffer << std::endl;
	}
	else if (function == P_stop)
	{
		int runNo;
		getIntegerParam(P_runNo, &runNo); // current number
		status = writeReadData("stop", input, sizeof(input));
		if (strncmp(input, "OK:", 3) != 0)
		{
			return asynError;
		}
		readStats();
		while(daqStatus() != 0)
		{
			epicsThreadSleep(1.0);
			readStats();
		}
		setADAcquire(0);
		std::cerr << "STOP: run " << runNo << std::endl;
		copyData();
	}
	else if (function == P_updatetof)
	{
		status = writeReadData("updatetof", input, sizeof(input));
	}	
	else if (function == P_settings)
	{
		status = readSettings();
	}
	else if (function == P_stats)
	{
		status = readStats();
	}
	else if (function == P_display)
	{
		status = setIntegerParam(P_display, value);
	}
	else if (function > LAST_NGEM_BASE_PARAM)
	{
	    status = setGEMParam(function, value);
	}
    if (status == asynSuccess)
    {
	    return asynPortDriver::writeInt32(pasynUser, value);
    }
    else
    {
        return status;
    }
}

template <typename T>
asynStatus nGEMDriver::setGEMParam(int param_id, T value)
{
	char input[256];
	std::ostringstream oss;
    for(map_t::const_iterator it=m_settings.begin(); it != m_settings.end(); ++it)
    {
	    if (it->second->param_id == param_id)
	    {
		    oss << "setting " << it->first << ":" << value;
	        return writeReadData(oss.str().c_str(), input, sizeof(input));
		}
	}
	return asynError;
}

asynStatus nGEMDriver::readInt32(asynUser *pasynUser, epicsInt32* value)
{
	int function = pasynUser->reason;
	if (function < FIRST_NGEM_PARAM)
	{
		return ADDriver::readInt32(pasynUser, value);
	}
	else
	{
		return asynPortDriver::readInt32(pasynUser, value);
	}
}

asynStatus nGEMDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	if (function < FIRST_NGEM_PARAM)
	{
		return ADDriver::writeFloat64(pasynUser, value);
	}
	else if (function > LAST_NGEM_BASE_PARAM)
	{
	    status = setGEMParam(function, value);
    }
    if (status == asynSuccess)
    {
		return asynPortDriver::writeFloat64(pasynUser, value);
    }
    else
    {
        return status;
    }
}

asynStatus nGEMDriver::readFloat64(asynUser *pasynUser, epicsFloat64* value)
{
	int function = pasynUser->reason;
	if (function < FIRST_NGEM_PARAM)
	{
		return ADDriver::readFloat64(pasynUser, value);
	}
	else
	{
		return asynPortDriver::readFloat64(pasynUser, value);
	}
}

asynStatus nGEMDriver::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
	std::string value_s(value, maxChars);
	char input[1024];
	asynStatus status = asynSuccess;
	int function = pasynUser->reason;
	if (function < FIRST_NGEM_PARAM)
	{
		return ADDriver::writeOctet(pasynUser, value, maxChars, nActual);
	}
	else if (function == P_command)
	{
		status = writeReadData(value_s.c_str(), input, sizeof(input));
		setStringParam(P_commandReply, input);
	}		
	else if (function > LAST_NGEM_BASE_PARAM)
	{
	    status = setGEMParam(function, value_s);
	}
    if (status == asynSuccess)
    {
	    return asynPortDriver::writeOctet(pasynUser, value, maxChars, nActual);
    }
    else
    {
        return status;
    }
}

asynStatus nGEMDriver::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason)
{
	int function = pasynUser->reason;
	if (function < FIRST_NGEM_PARAM)
	{
		return ADDriver::readOctet(pasynUser, value, maxChars, nActual, eomReason);
	}
	else
	{
		return asynPortDriver::readOctet(pasynUser, value, maxChars, nActual, eomReason);
	}
}

// called with asyn lock
void nGEMDriver::processData()
{
    static const char* functionName = "processData";
	static char buffer[1000000];
	static double value[1000000];
	int display;
	size_t nelements;
	int acquiring = 0;
    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    NDArray *pImage;
    double acquireTime, acquirePeriod, delay;
    epicsTimeStamp startTime, endTime;
    double elapsedTime;
		getIntegerParam(ADAcquire, &acquiring);
//		if (acquiring == 0)
//		{
//			m_old_acquiring = acquiring;
//			unlock();
//			continue;
//		}
		getIntegerParam(P_display, &display);
		switch (display)
		{
		case Display2DFXY:
			writeReadData("2dfxy b", buffer, sizeof(buffer));
			break;
		case Display2DLXY:
			writeReadData("2dlxy b", buffer, sizeof(buffer));
			break;
		case Display2DAVG:
			writeReadData("2davg b", buffer, sizeof(buffer));
			break;
		case Display2DRNG:
			writeReadData("2drng b", buffer, sizeof(buffer));
			break;
		default:
			break;
		}
		convertData(buffer, value, sizeof(value) / sizeof(double), nelements);
		applyDataMode(value, nelements);
		getDoubleParam(ADAcquirePeriod, &acquirePeriod);
//		if (m_old_acquiring == 0)
//		{
//			setIntegerParam(ADNumImagesCounter, 0);
//			m_old_acquiring = acquiring;
//		}
		setIntegerParam(ADStatus, ADStatusAcquire);
		epicsTimeGetCurrent(&startTime);
		getIntegerParam(ADImageMode, &imageMode);

		/* Get the exposure parameters */
		getDoubleParam(ADAcquireTime, &acquireTime);  // not really used

		setShutter(ADShutterOpen);
		callParamCallbacks();

		/* Update the image */
		status = computeImage(value, nelements);
		//        if (status) continue;

		// could sleep to make up to acquireTime

				/* Close the shutter */
		setShutter(ADShutterClosed);

		setIntegerParam(ADStatus, ADStatusReadout);
		/* Call the callbacks to update any changes */
		callParamCallbacks();

		pImage = this->pArrays[0];
		// setTimeStamp(epicsTS);   ??????

		/* Get the current parameters */
		getIntegerParam(NDArrayCounter, &imageCounter);
		getIntegerParam(ADNumImages, &numImages);
		getIntegerParam(ADNumImagesCounter, &numImagesCounter);
		getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
		imageCounter++;
		numImagesCounter++;
		setIntegerParam(NDArrayCounter, imageCounter);
		setIntegerParam(ADNumImagesCounter, numImagesCounter);

		/* Put the frame number and time stamp into the buffer */
		pImage->uniqueId = imageCounter;
		pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
		updateTimeStamp(&pImage->epicsTS);
		//getTimeStamp(&pImage->epicsTS);

		/* Get any attributes that have been defined for this driver */
		this->getAttributes(pImage->pAttributeList);

		if (arrayCallbacks) {
			/* Call the NDArray callback */
			/* Must release the lock here, or we can get into a deadlock, because we can
			 * block on the plugin lock, and the plugin can be calling us */
			this->unlock();
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
				"%s:%s: calling imageData callback\n", driverName, functionName);
			doCallbacksGenericPointer(pImage, NDArrayData, 0);
			this->lock();
		}

		/* Call the callbacks to update any changes */
		callParamCallbacks();
		/* sleep for the acquire period minus elapsed time. */
		epicsTimeGetCurrent(&endTime);
		elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
		delay = acquirePeriod - elapsedTime;
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: delay=%f\n",
			driverName, functionName, delay);
	
//        if (delay >= 0.0) {
 //           /* We set the status to waiting to indicate we are in the period delay */
//            setIntegerParam(ADStatus, ADStatusWaiting);
//            callParamCallbacks();
//            this->unlock();
//            epicsThreadSleep(delay);
//            this->lock();
//            setIntegerParam(ADStatus, ADStatusIdle);
//            callParamCallbacks();  
//        }
}

/** Computes the new image data */
int nGEMDriver::computeImage(double *value, size_t nelements)
{
    int status = asynSuccess;
    NDDataType_t dataType;
    int itemp;
    int binX, binY, minX, minY, sizeX, sizeY, reverseX, reverseY;
    int xDim=0, yDim=1, colorDim=-1;
    int maxSizeX, maxSizeY;
    int colorMode;
    int ndims=0;
    NDDimension_t dimsOut[3];
    size_t dims[3];
    NDArrayInfo_t arrayInfo;
    NDArray *pImage;
    const char* functionName = "computeImage";

    /* NOTE: The caller of this function must have taken the mutex */

    status |= getIntegerParam(ADBinX,         &binX);
    status |= getIntegerParam(ADBinY,         &binY);
    status |= getIntegerParam(ADMinX,         &minX);
    status |= getIntegerParam(ADMinY,         &minY);
    status |= getIntegerParam(ADSizeX,        &sizeX);
    status |= getIntegerParam(ADSizeY,        &sizeY);
    status |= getIntegerParam(ADReverseX,     &reverseX);
    status |= getIntegerParam(ADReverseY,     &reverseY);
    status |= getIntegerParam(ADMaxSizeX,     &maxSizeX);
    status |= getIntegerParam(ADMaxSizeY,     &maxSizeY);
    status |= getIntegerParam(NDColorMode,    &colorMode);
    status |= getIntegerParam(NDDataType,     &itemp); 
	dataType = (NDDataType_t)itemp;
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error getting parameters\n",
                    driverName, functionName);

    /* Make sure parameters are consistent, fix them if they are not */
    if (binX < 1) {
        binX = 1;
        status |= setIntegerParam(ADBinX, binX);
    }
    if (binY < 1) {
        binY = 1;
        status |= setIntegerParam(ADBinY, binY);
    }
    if (minX < 0) {
        minX = 0;
        status |= setIntegerParam(ADMinX, minX);
    }
    if (minY < 0) {
        minY = 0;
        status |= setIntegerParam(ADMinY, minY);
    }
    if (minX > maxSizeX-1) {
        minX = maxSizeX-1;
        status |= setIntegerParam(ADMinX, minX);
    }
    if (minY > maxSizeY-1) {
        minY = maxSizeY-1;
        status |= setIntegerParam(ADMinY, minY);
    }
    if (minX+sizeX > maxSizeX) {
        sizeX = maxSizeX-minX;
        status |= setIntegerParam(ADSizeX, sizeX);
    }
    if (minY+sizeY > maxSizeY) {
        sizeY = maxSizeY-minY;
        status |= setIntegerParam(ADSizeY, sizeY);
    }

    switch (colorMode) {
        case NDColorModeMono:
            ndims = 2;
            xDim = 0;
            yDim = 1;
            break;
        case NDColorModeRGB1:
            ndims = 3;
            colorDim = 0;
            xDim     = 1;
            yDim     = 2;
            break;
        case NDColorModeRGB2:
            ndims = 3;
            colorDim = 1;
            xDim     = 0;
            yDim     = 2;
            break;
        case NDColorModeRGB3:
            ndims = 3;
            colorDim = 2;
            xDim     = 0;
            yDim     = 1;
            break;
    }

// we could be more efficient
//    if (resetImage) {
    /* Free the previous raw buffer */
        if (m_pRaw) m_pRaw->release();
        /* Allocate the raw buffer we use to compute images. */
        dims[xDim] = maxSizeX;
        dims[yDim] = maxSizeY;
        if (ndims > 2) dims[colorDim] = 3;
        m_pRaw = this->pNDArrayPool->alloc(ndims, dims, dataType, 0, NULL);

        if (!m_pRaw) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: error allocating raw buffer\n",
                      driverName, functionName);
            return(status);
        }
//    }

    switch (dataType) {
        case NDInt8:
            status |= computeArray<epicsInt8>(value, nelements, maxSizeX, maxSizeY);
            break;
        case NDUInt8:
            status |= computeArray<epicsUInt8>(value, nelements, maxSizeX, maxSizeY);
            break;
        case NDInt16:
            status |= computeArray<epicsInt16>(value, nelements, maxSizeX, maxSizeY);
            break;
        case NDUInt16:
            status |= computeArray<epicsUInt16>(value, nelements, maxSizeX, maxSizeY);
            break;
        case NDInt32:
            status |= computeArray<epicsInt32>(value, nelements, maxSizeX, maxSizeY);
            break;
        case NDUInt32:
            status |= computeArray<epicsUInt32>(value, nelements, maxSizeX, maxSizeY);
            break;
        case NDFloat32:
            status |= computeArray<epicsFloat32>(value, nelements, maxSizeX, maxSizeY);
            break;
        case NDFloat64:
            status |= computeArray<epicsFloat64>(value, nelements, maxSizeX, maxSizeY);
            break;
    }

    /* Extract the region of interest with binning.
     * If the entire image is being used (no ROI or binning) that's OK because
     * convertImage detects that case and is very efficient */
    m_pRaw->initDimension(&dimsOut[xDim], sizeX);
    m_pRaw->initDimension(&dimsOut[yDim], sizeY);
    if (ndims > 2) m_pRaw->initDimension(&dimsOut[colorDim], 3);
    dimsOut[xDim].binning = binX;
    dimsOut[xDim].offset  = minX;
    dimsOut[xDim].reverse = reverseX;
    dimsOut[yDim].binning = binY;
    dimsOut[yDim].offset  = minY;
    dimsOut[yDim].reverse = reverseY;
    /* We save the most recent image buffer so it can be used in the read() function.
     * Now release it before getting a new version. */
    if (this->pArrays[0]) this->pArrays[0]->release();
    status = this->pNDArrayPool->convert(m_pRaw,
                                         &this->pArrays[0],
                                         dataType,
                                         dimsOut);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error allocating buffer in convert()\n",
                    driverName, functionName);
        return(status);
    }
    pImage = this->pArrays[0];
    pImage->getInfo(&arrayInfo);
    status = asynSuccess;
    status |= setIntegerParam(NDArraySize,  (int)arrayInfo.totalBytes);
    status |= setIntegerParam(NDArraySizeX, (int)pImage->dims[xDim].size);
    status |= setIntegerParam(NDArraySizeY, (int)pImage->dims[yDim].size);
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error setting parameters\n",
                    driverName, functionName);
    return(status);
}

// supplied array of x,y,t
template <typename epicsType> 
int nGEMDriver::computeArray(double* value, size_t nelements, int sizeX, int sizeY)
{
	epicsType *pMono = NULL, *pRed = NULL, *pGreen = NULL, *pBlue = NULL;
	int columnStep = 0, rowStep = 0, colorMode;
	int status = asynSuccess;
	double exposureTime;
	int i, j;

	status = getIntegerParam(NDColorMode, &colorMode);
	status = getDoubleParam(ADAcquireTime, &exposureTime);

	switch (colorMode) {
	case NDColorModeMono:
		pMono = (epicsType *)m_pRaw->pData;
		break;
	case NDColorModeRGB1:
		columnStep = 3;
		rowStep = 0;
		pRed = (epicsType *)m_pRaw->pData;
		pGreen = (epicsType *)m_pRaw->pData + 1;
		pBlue = (epicsType *)m_pRaw->pData + 2;
		break;
	case NDColorModeRGB2:
		columnStep = 1;
		rowStep = 2 * sizeX;
		pRed = (epicsType *)m_pRaw->pData;
		pGreen = (epicsType *)m_pRaw->pData + sizeX;
		pBlue = (epicsType *)m_pRaw->pData + 2 * sizeX;
		break;
	case NDColorModeRGB3:
		columnStep = 1;
		rowStep = 0;
		pRed = (epicsType *)m_pRaw->pData;
		pGreen = (epicsType *)m_pRaw->pData + sizeX * sizeY;
		pBlue = (epicsType *)m_pRaw->pData + 2 * sizeX*sizeY;
		break;
	}
	m_pRaw->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
	memset(m_pRaw->pData, 0, m_pRaw->dataSize);
	for (i = sizeY - 1; i >= 0; --i) {
		switch (colorMode) {
		case NDColorModeMono:
			for (j = 0; j < sizeX; j++) {
				*pMono = value[i*sizeX + j];
				++pMono;
			}
			break;
		case NDColorModeRGB1:
		case NDColorModeRGB2:
		case NDColorModeRGB3:
			for (j = 0; j < sizeX; j++) {
				*pRed = value[i*sizeX + j];
				*pGreen = value[i*sizeX + j];
				*pBlue = value[i*sizeX + j];
				pRed += columnStep;
				pGreen += columnStep;
				pBlue += columnStep;
			}
			pRed += rowStep;
			pGreen += rowStep;
			pBlue += rowStep;
			break;
		}
	}
	return(status);
}

/** Controls the shutter */
void nGEMDriver::setShutter(int open)
{
    int shutterMode;

    getIntegerParam(ADShutterMode, &shutterMode);
    if (shutterMode == ADShutterModeDetector) {
        /* Simulate a shutter by just changing the status readback */
        setIntegerParam(ADShutterStatus, open);
    } else {
        /* For no shutter or EPICS shutter call the base class method */
        ADDriver::setShutter(open);
    }
}


/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void nGEMDriver::report(FILE *fp, int details)
{
    fprintf(fp, "nGEM driver %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}
		
extern "C" {

/// EPICS iocsh callable function to call constructor of lvDCOMInterface().
/// \param[in] portName @copydoc initArg0
/// \param[in] configSection @copydoc initArg1
/// \param[in] configFile @copydoc initArg2
/// \param[in] host @copydoc initArg3
/// \param[in] options @copydoc initArg4
/// \param[in] progid @copydoc initArg5
/// \param[in] username @copydoc initArg6
/// \param[in] password @copydoc initArg7
int nGEMConfigure(const char *portName, const char *ipPortName)
{
	try
	{
		nGEMDriver* driver = new nGEMDriver(portName, ipPortName);
		if (driver == NULL)
		{
		    errlogSevPrintf(errlogMajor, "nGEMConfigure failed (NULL)\n");
			return(asynError);
		}
		return(asynSuccess);
	}
	catch(const std::exception& ex)
	{
		errlogSevPrintf(errlogMajor, "nGEMConfigure failed: %s\n", ex.what());
		return(asynError);
	}
}

// EPICS iocsh shell commands 

static const iocshArg initArg0 = { "portName", iocshArgString};			///< The name of the asyn driver port we will create
static const iocshArg initArg1 = { "ipPortName", iocshArgString};			    ///< options as per #lvDCOMOptions enum

static const iocshArg * const initArgs[] = { &initArg0, &initArg1 };

static const iocshFuncDef initFuncDef = {"nGEMConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    nGEMConfigure(args[0].sval, args[1].sval);
}

static void nGEMRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
    initHookRegister(myHookFunction);
}

epicsExportRegistrar(nGEMRegister);

}
