/*
 * Enums.h
 *
 *  Created on: May 21, 2011
 *      Author: hh7
 */

#ifndef ENUMS_H_
#define ENUMS_H_

//#include <string.h>
//#include <boost/thread/mutex.hpp>
//#include <errno.h>
#include <boost/cstdint.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {
namespace android_teach {

#define DEBUG 1
#define NUMBER_OF_JOINTS 7

enum DEV_VSP_MSG_TYPE
{
	DEV_VSP_HELLO,
	DEV_VSP_DISCONNECT,
	DEV_VSP_READINGS,
	DEV_VSP_GET_CONFIGURATION,
	DEV_VSP_ERROR_CODE,
	DEC_VSP_TERMINATE,
	DEV_VSP_NONE,
	DEV_VSP_START,
	DEV_VSP_WAIT,
};

enum VSP_DEV_MSG_TYPE
{
	VSP_DEV_HELLO,
	VSP_DEV_DISCONNECT,
	VSP_DEV_CONFIRM,
	VSP_DEV_CONFIGURE,
	VSP_DEV_NONE,
	VSP_DEV_WAIT,
	VSP_DEV_ERROR_CODE,
	VSP_DEV_RESUME,
	VSP_DEV_WAIT_BEFORE_START
};

// komunikacja ECP i VSP

enum VSP_ECP_MSG_TYPE
{
	VSP_ECP_HELLO,
	VSP_ECP_READINGS,
    VSP_ECP_CONFIRM,
	VSP_ECP_DISCONNECT,
	VSP_TERMINATE,
	VSP_ECP_WAIT,
	VSP_ECP_ERROR_CODE,
	VSP_ECP_GET_CONFIGURATION,
	VSP_ECP_NONE,
	VSP_ECP_ABORT,
	VSP_ECP_WAIT_BEFORE_START
};

enum ECP_VSP_MSG_TYPE
{
	ECP_VSP_GET_READING,
	ECP_VSP_CONFIGURE,
	ECP_VSP_HELLO,
	ECP_VSP_DISCONNECT,
	ECP_VSP_ERROR_CODE,
	ECP_VSP_START,
	ECP_VSP_NONE,
	ECP_VSP_WAIT,
    ECP_VSP_RESUME
};

// komunikaty o stanie urzadzenia
class Readings
{
	public:
        // mode = 0 - zadawanie predkosci, mode = 1 - zadawaie pozycji, mode=2 node control
        int32_t mode;
        int32_t value;
        int32_t selectedAxis;
        double newJointValue[NUMBER_OF_JOINTS];


		Readings()
		{
            value = 0;
            selectedAxis = 0;
            mode = 0;
            for (int i = 0; i <= NUMBER_OF_JOINTS; ++i)
            {
                newJointValue[i] = 0;
            }
		}

};


class Configuration
{
public:
    int32_t numberOfSpeedIntervals;
	double jointValue[NUMBER_OF_JOINTS];
    double jointMaxValue[NUMBER_OF_JOINTS];
    double jointMinValue[NUMBER_OF_JOINTS];
    double positionStep[NUMBER_OF_JOINTS];


	Configuration()
	{
		for (int i = 0; i <= NUMBER_OF_JOINTS; ++i)
		{
		    jointValue[i] = 0;
		    jointMaxValue[i] = 0;
		    jointMinValue[i] = 0;
		    positionStep[i] = 0;
		    numberOfSpeedIntervals = 0;
		}
	}
};


class EcpStatus
{
public:
	int32_t limit;
	double jointValue[NUMBER_OF_JOINTS];


	EcpStatus()
	{
        for (int i = 0; i <= NUMBER_OF_JOINTS; ++i)
		{
		    jointValue[i] = 0;
		}
		limit = 0;
	}
};




class EcpVspMsg
{
public:
	ECP_VSP_MSG_TYPE msgType;

	EcpVspMsg()
	{
		msgType = ECP_VSP_NONE;
	}

	EcpVspMsg(ECP_VSP_MSG_TYPE type)
	{
		msgType = type;
	}
};

class EcpVspStateMsg : public EcpVspMsg
{
public:

	EcpStatus ecpStatus;

	EcpVspStateMsg(): EcpVspMsg(ECP_VSP_GET_READING)
	{
		msgType = ECP_VSP_NONE;
	}

	EcpVspStateMsg(ECP_VSP_MSG_TYPE type)
	{
		msgType = type;
	}
};

class EcpVspConfigMsg : public EcpVspMsg
{
public:
    Configuration config;


	EcpVspConfigMsg()
	: EcpVspMsg(ECP_VSP_CONFIGURE)
	{
		//TODO add configuration
	}

	EcpVspConfigMsg(ECP_VSP_MSG_TYPE type)
	{
		msgType = type;
	}
};

class EcpVspErrorMsg : public EcpVspMsg
{
public:
	int32_t errorCode;

	EcpVspErrorMsg()
	: EcpVspMsg(ECP_VSP_ERROR_CODE), errorCode(0)
	{
		//nothing to write
	}

	EcpVspErrorMsg(const int32_t errorCode)
	: EcpVspMsg(ECP_VSP_ERROR_CODE), errorCode(errorCode)
	{
		//nothing to write
	}
};



class VspEcpMsg
{
public:
	VSP_ECP_MSG_TYPE msgType;

	VspEcpMsg()
	{
		msgType = VSP_ECP_NONE;
	}

	VspEcpMsg(VSP_ECP_MSG_TYPE type)
	{
		msgType = type;
	}
};


class VspEcpReadingsMsg : public VspEcpMsg
{
public:
    Readings readings;

	VspEcpReadingsMsg() : VspEcpMsg(VSP_ECP_READINGS)
	{
		//nothing to write
	}
};

class VspEcpErrorMsg : public VspEcpMsg
{
public:
	int32_t errorCode;

	VspEcpErrorMsg()
	: VspEcpMsg(VSP_ECP_ERROR_CODE), errorCode(0)
	{
		// nothing to write
	}

	VspEcpErrorMsg(const int32_t errorCode)
	: VspEcpMsg(VSP_ECP_ERROR_CODE), errorCode(errorCode)
	{
		// nothing to write
	}
};














/*
class VspDevMsg
{
public:
	VSP_DEV_MSG_TYPE msgType;

	VspDevMsg()
	{
		msgType = VSP_DEV_NONE;
	}

	VspDevMsg(VSP_DEV_MSG_TYPE type)
	{
		msgType = type;
	}

};


class VspDevStateMsg : public VspDevMsg
{
public:
	EcpStatus ecpStatus;

	VspDevErrorMsg()
	: VspDevMsg(VSP_DEV_CONFIRM)
	{
		//nothing to write
	}

	VspDevErrorMsg(VSP_DEV_MSG_TYPE type)
	: VspDevMsg(type)
	{
		//nothing to write
	}
};


class VspDevErrorMsg : public VspDevMsg
{
public:
	int32_t errorCode;

	VspDevErrorMsg()
	: VspDevMsg(VSP_DEV_ERROR_CODE), errorCode(0)
	{
		//nothing to write
	}

	VspDevErrorMsg(const int32_t errorCode)
	: VspDevMsg(VSP_DEV_ERROR_CODE), errorCode(errorCode)
	{
		//nothing to write
	}
};


class DevVspMsg
{
public:
	DEV_VSP_MSG_TYPE msgType;

	DevVspMsg()
	{
		msgType = DEV_VSP_NONE;
	}

	DevVspMsg(DEV_VSP_MSG_TYPE type)
	{
		msgType = type;
	}
};

class DevVspErrorMsg : public DevVspMsg
{
public:
	int32_t errorCode;

	DevVspErrorMsg()
	: DevVspMsg(DEV_VSP_ERROR_CODE), errorCode(0)
	{
		//nothing to write
	}

	DevVspErrorMsg(const int32_t errorCode)
	: DevVspMsg(DEV_VSP_ERROR_CODE), errorCode(errorCode)
	{
		//nothing to write
	}
};


class DevVspReadingsMsg : public DevVspMsg
{
public:
    Readings readings;

	DevVspReadingsMsg() : DevVspMsg(DEV_VSP_READINGS)
	{
		//nothing to write
	}

};
*/


/*
union VspEcpMsgUnion
{
	VspEcpErrorMsg errorMsg;
	VspEcpMsg commonMsg;
	VspEcpReadingsMsg readingsMsg;
};

union EcpVspMsgUnion
{
	EcpVspErrorMsg errorMsg;
	EcpVspMsg commonMsg;
	EcpVspConfigureMsg configureMsg;
};


union DevVspMsgUnion
{
	DevVspMsg commonMsg;
	DevVspErrorMsg errorMsg;
	DevVspReadingsMsg readingsMsg;
};

union VspDevMsgUnion
{
	VspDevMsg commonMsg;
	VspDevErrorMsg errorMsg;
};
*/


//static boost::mutex mux;
//static boost::mutex syncMux;
//static Readings readings;
//static Configuration config;
//static EcpStatus ecpStatus;
//static bool devConnected = false;
//static bool ecpConnected = false;
//static bool wait = false;


} //android_teach
} //sensor
} //ecp_mp
} //mrrocpp

#endif /* ENUMS_H_ */
