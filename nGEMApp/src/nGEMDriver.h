#ifndef NGEMDRIVER_H
#define NGEMDRIVER_H
 
#include "ADDriver.h"
#include "asynPortClient.h"

struct nGEMParam;

typedef std::map<std::string,nGEMParam*> map_t;

class nGEMDriver : public ADDriver 
{
public:
    nGEMDriver(const char *portName, const char* ipPortName);
 	static void pollerThreadC1(void* arg);
                
    // These are the methods that we override from asynPortDriver
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
	virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
	virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
	virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn);
	
    virtual void report(FILE *fp, int details);
    virtual void setShutter(int open);
	enum DiplayMode { Display2DFXY=0, Display2DLXY=1, Display2DAVG=2, Display2DRNG=3 }; ///< must match mbbi in EPICS DB

private:
    enum ParamGroup { GroupSettings=0, GroupStats=1 };
    map_t m_settings;
    map_t m_stats;
    int addParam(const char* name, ParamGroup group, asynParamType type);
	int daqStatus();
    void processData();
	void computeTOF();
	void copyData();
	template <typename T> asynStatus setGEMParam(int param_id, T value);
	
	int P_start; // int, FIRST_NGEM_PARAM
	int P_stop; // int
	int P_updatetof; // int
	int P_command; // string
	int P_commandReply; // string
	int P_1dtof; // float64array
	int P_1dsx; // float64array
	int P_1dsy; // float64array
	int P_2dfxy; // float64array
	int P_2dlxy; // float64array
	int P_2davg; // float64array
	int P_2drng; // float64array
	int P_display; // int
	int P_stats; // int
	int P_settings; // int
	
	// these are returned from addParam()
	int P_daqStatus; // string
	int P_runNo; // string
	int P_inst; // string
	int P_basepath; // string
	int P_tofmin; // double
	int P_tofmax; // double
	int P_tofwidth; // double

	// calculated
	int P_ntof; // integer
	int P_tof; // float64array
	int P_1dsxt; // float64array, x array for P_1dsx
	int P_1dsyt; // float64array, x array for P_1dsy
	int P_dir; // string

#define NUM_NGEM_PARAMS 100
#define FIRST_NGEM_PARAM P_start
	
//	asynUser* m_detPort;
	asynOctetClient* m_det;
	int m_old_acquiring;
    NDArray* m_pRaw;
	std::vector<double> m_tof;
	std::vector<double> m_tof_old;
	
	void pollerThread1();
	void setADAcquire(int acquire);
	int computeImage(double *value, size_t nelements);
	template <typename epicsType> int computeArray(double* value, size_t nelements, int sizeX, int sizeY);
    
    asynStatus writeData(const char* output, int timeout);
    asynStatus writeReadData(const char* output, char* input, size_t inputLen);
    asynStatus readPairs(const char* command, map_t& map);
	asynStatus readSettings();
	asynStatus readStats();
    asynStatus convertData(void* buffer, double* data, int nin, size_t& nout);
};

#define P_startString			"START"
#define P_stopString			"STOP"
#define P_updatetofString		"UPDATETOF"
#define P_commandString			"COMMAND"
#define P_commandReplyString	"COMMAND_REPLY"
#define P_1dtofString			"1DTOF"
#define P_1dsxString			"1DSX"
#define P_1dsyString			"1DSY"
#define P_2dfxyString			"2DFXY"
#define P_2dlxyString			"2DLXY"
#define P_2davgString			"2DAVG"
#define P_2drngString			"2DRNG"
#define P_displayString			"DISPLAY"
#define P_statsString			"STATS"
#define P_settingsString		"SETTINGS"
#define P_1dsxtString			"1DSXT"
#define P_1dsytString			"1DSYT"
#define P_ntofString			"NTOF"
#define P_tofString				"TOF"
#define P_dirString				"DIR"

struct nGEMParam
{
	nGEMDriver* driver;
	int param_id;
	asynParamType type;
    nGEMParam(nGEMDriver* d, int i, asynParamType t) : driver(d), param_id(i), type(t) {}
	void setValue(const std::string& value)
	{
		switch(type)
		{
			case asynParamOctet:
			    driver->setStringParam(param_id, value.c_str());
				break;

			case asynParamInt32:
			    driver->setIntegerParam(param_id, atoi(value.c_str()));
				break;

			case asynParamFloat64:
			    driver->setDoubleParam(param_id, atof(value.c_str()));
				break;
				
			default:
			    throw std::runtime_error("unknown");
			    break;
		}
	}
};

#endif /* NGEMDRIVER_H */
