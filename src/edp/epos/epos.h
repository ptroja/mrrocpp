/*! \file epos.h

 header file for libEPOS functions

 mh, july 2006

 */

#ifndef _EPOS_H
#define _EPOS_H

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <stdint.h>  /* int types with given size */
#include <math.h>

#include <boost/exception.hpp>
#include <boost/type_traits/is_same.hpp>

#include <string>
#include <exception>
#include <vector>

/* added oct06 for openTCPEPOS() */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//#include "eposlib.h" -> moved to epos.c

/* all EPOS data exchange is based on 16bit words, but other types are
 also used...*/
typedef uint32_t DWORD; ///< \brief 32bit type for EPOS data exchange
typedef uint16_t WORD; ///< \brief 16bit type for EPOS data exchange
#ifndef CPP
typedef char BYTE; ///< \brief 8bit type for EPOS data exchange
#endif

typedef int8_t INTEGER8;
typedef int16_t INTEGER16;
typedef int32_t INTEGER32;

typedef uint8_t UNSIGNED8;
typedef uint16_t UNSIGNED16;
typedef uint32_t UNSIGNED32;

/* EPOS will reset communication after 500ms of inactivity */

/*! \brief try NTRY times to read one byte from EPOS, the give up */
#define NTRY      5
/*! \brief sleep TRYSLEEP usec between read() from EPOS, if no data available */
#define TRYSLEEP  (unsigned int)1e5

/* all high-level methods throws this exception in case of error */

struct epos_error: virtual std::exception, virtual boost::exception { };

typedef boost::error_info<struct tag_reason,const char *> reason;
typedef boost::error_info<struct tag_errno_code,int> errno_code;
typedef boost::error_info<struct tag_errno_code,const char *> errno_call;

typedef std::vector<WORD> answer_t;

class epos
{
	private:
		/* Implement read functions defined in EPOS Communication Guide, 6.3.1 */
		/* [ one simplification: Node-ID is always 0] */

		const std::string device;
		struct termios options;
		DWORD E_error; ///< EPOS global error status
		int ep; ///< (-1) EPOS file descriptor
		char gMarker; ///< (0) for internal handling

		/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.1*/
		std::vector<WORD> ReadObject(WORD index, BYTE subindex);

		template <class T>
		T ReadObjectValue(WORD index, BYTE subindex) {
			answer_t answer = ReadObject(index, subindex);

			// check error code
			checkEPOSerror();

#ifdef DEBUG
			printf("ReadObjectValue(%0x04x, 0x02x)==> %d\n", val);
#endif

			if ((boost::is_same<T, uint8_t>::value) || (boost::is_same<T, int8_t>::value) ||
				(boost::is_same<T, uint16_t>::value) || (boost::is_same<T, int16_t>::value)) {
				T val = answer[3];
				return val;
			}
			else if ((boost::is_same<T, uint32_t>::value) || (boost::is_same<T, int32_t>::value)) {
				T val = (answer[3] | (answer[4] << 16));
				return val;
			} else {
				throw epos_error() << reason("Unsupported ReadObjectValue conversion");
			}
		}

		#if 0
		/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.2 */
		int InitiateSegmentedRead(WORD index, BYTE subindex );

		/*! \brief int SegmentRead(WORD **ptr) - read data segment of the object
		   initiated with 'InitiateSegmentedRead()'
		*/
		int SegmentRead(WORD **ptr);
		#endif

		/* 6.3.2:  write functions */

		/*! 6.3.2.1 WriteObject()

		   WORD *data is a pointer to a 2 WORDs array (== 4 BYTES)
		   holding data to transmit
		*/
		void WriteObject(WORD index, BYTE subindex, const WORD data[2]);

		void WriteObjectValue(WORD index, BYTE subindex, WORD data0, WORD data1 = 0x0000);

		/* helper functions below */

		/*! \brief  write a single BYTE to EPOS */
		void writeBYTE(BYTE c);

		/*! \brief  write a single WORD to EPOS */
		void writeWORD(WORD w);

		/*! \brief  read a single BYTE from EPOS, timeout implemented */
		BYTE readBYTE();

		/*! \brief  read a single WORD from EPOS, timeout implemented */
		WORD readWORD();

		/*! \brief  send command to EPOS, taking care of all neccessary 'ack' and
		   checksum tests*/
		void sendCom(WORD *frame);

		/*! \brief  int readAnswer(WORD **ptr) - read an answer frame from EPOS */
		answer_t readAnswer();

		/*! \brief check global variable E_error for EPOS error code */
		int checkEPOSerror();

		/*! \brief Checksum calculation;
		copied from EPOS Communication Guide, p.8
		 */
		WORD CalcFieldCRC(const WORD *pDataArray, WORD numberOfWords) const;

		/*! \brief exit(-1) if ptr == NULL */
		void checkPtr(const void* ptr) const;

		/*! \brief compare two 16bit bitmasks, return 1 (true) or 0 (false) */
		bool bitcmp(WORD a, WORD b) const;

	public:
		/*! create new EPOS object */
		epos(const std::string & _device);

		/*! delete EPOS object */
		~epos();

		/*! open the connection to EPOS */
		int openEPOS(tcflag_t br);

		/*! close the connection to EPOS */
		int closeEPOS();

		/*! check if the connection to EPOS is alive */
//		int checkEPOS();

		/*! \brief check EPOS status, return state according to firmware spec 8.1.1 */
		int checkEPOSstate();

		/*! \brief pretty-print EPOS state */
		int printEPOSstate();

		//States
		typedef enum _state {
			ST_DISABLED = 0,
			ST_ENABLED = 1,
			ST_QUICKSTOP = 2,
			ST_FAULT = 3
		} state_t;

		/*! \brief change EPOS state   ==> firmware spec 8.1.3 */
		void changeEPOSstate(state_t state);

		/*! \brief example from EPOS com. guide: ask EPOS for software version

		 firmware spec 14.1.33
		 ** returns software version as HEX **

		 */
		UNSIGNED16 readSWversion();

		/*! \brief ask for device name,
		 device name is placed in 'name' (string must be big enough, NO CHECKING!!)
		 */
		std::string readDeviceName();

		/*! \brief ask for RS232 timeout; firmware spec 14.1.35 */
		UNSIGNED16 readRS232timeout();

		/*! \brief read digital input polarity mask */
		UNSIGNED16 readDInputPolarity();

		/*! \brief set home switch polarity -- firmware spec 14.1.47 */
		void setHomePolarity(int pol);

		/*! \brief read Statusword; 14.1.58 */
		UNSIGNED16 readStatusWord();

		/*! \brief pretty-print Statusword */
		int printEPOSstatusword(WORD statusword);

		/*! \brief read EPOS control word (firmware spec 14.1.57) */
		UNSIGNED16 readControlword();

		/*! \brief pretty-print Controlword */
		void printEPOScontrolword(WORD controlword);

		//Operational mode
		typedef enum _operational_mode {
			OMD_PROFILE_POSITION_MODE = 1,		//! profile position mode
			OMD_PROFILE_VELOCITY_MODE = 3,		//! profile velocity mode
			OMD_HOMING_MODE = 6,				//! homing
			OMD_INTERPOLATED_POSITION_MODE = 7,
			OMD_POSITION_MODE = -1,				//! position mode
			OMD_VELOCITY_MODE = -2,				//! velocity mode
			OMD_CURRENT_MODE = -3,				//! current mode
			//OMD_DIAGNOSTIC_MODE = -4			//! diagnostic mode
			OMD_MASTER_ENCODER_MODE	= -5,
			OMD_STEP_DIRECTION_MODE	= -6
		} operational_mode_t;

		/*! \brief set EPOS mode of operation -- 14.1.59 */
		void setOpMode(operational_mode_t);

		/*! \brief read and returns  EPOS mode of operation -- 14.1.60
		 here, RETURN(0) MEANS ERROR!
		 '-1' is a valid OpMode, but 0 is not!
		 */
		INTEGER8 readOpMode();

		/*! \brief read demanded position; 14.1.61 */
		INTEGER32 readDemandPosition();

		/*! \brief read actual position; 14.1.62 */
		INTEGER32 readActualPosition();

		/*! \brief read position window; 14.1.64 */
		UNSIGNED32 readPositionWindow();

		/*! \brief write position window; 14.1.64 */
		void writePositionWindow(UNSIGNED32 value);

		/*! < by Martí Morta (mmorta@iri.upc.edu) > */

		/*! \brief read position window time; 14.1.67 */

		/*! \brief read position window time; 14.1.67 */
//		int writePositionWindowTime(unsigned int val);

//		int writePositionSoftwareLimits(long val, long val2);

		void writePositionProfileVelocity(UNSIGNED32 vel);
		void writePositionProfileAcceleration(UNSIGNED32 acc);
		void writePositionProfileDeceleration(UNSIGNED32 dec);
		void writePositionProfileQuickStopDeceleration(UNSIGNED32 qsdec);
		void writePositionProfileMaxVelocity(UNSIGNED32 maxvel);
		void writePositionProfileType(INTEGER16 type);

		UNSIGNED32 readPositionProfileVelocity();
		UNSIGNED32 readPositionProfileAcceleration();
		UNSIGNED32 readPositionProfileDeceleration();
		UNSIGNED32 readPositionProfileQuickStopDeceleration();
		UNSIGNED32 readPositionProfileMaxVelocity();
		INTEGER16 readPositionProfileType();

		//VelocityNotation
		typedef enum _velocity_notation {
		    VN_STANDARD                                          = 0,
		    VN_DECI                                              = -1,
		    VN_CENTI                                             = -2,
		    VN_MILLI                                             = -3
		} velocity_notation_t;

		velocity_notation_t readVelocityNotationIndex();
		void writeVelocityNotationIndex(velocity_notation_t val);

	    //SensorType
		typedef enum _sensor_type {
			ST_UNKNOWN                                           = 0,
			ST_INC_ENCODER_3CHANNEL                              = 1,
			ST_INC_ENCODER_2CHANNEL                              = 2,
			ST_HALL_SENSORS                                      = 3,
			ST_SSI_ABS_ENCODER_BINARY                            = 4,
			ST_SSI_ABS_ENCODER_GREY                              = 5
		} sensor_type_t;

		UNSIGNED32 readSensorPulses();
		sensor_type_t readSensorType();
		UNSIGNED16 readSensorPolarity();

		void writeSensorType(sensor_type_t val);
		void writeSensorPulses(UNSIGNED32 val);
		void writeSensorPolarity(UNSIGNED16 val);

		UNSIGNED16 readRS232Baudrate();
		void writeRS232Baudrate(UNSIGNED16 val);

		INTEGER16 readP();
		INTEGER16 readI();
		INTEGER16 readD();
		UNSIGNED16 readVFF();
		UNSIGNED16 readAFF();
		void writeP(INTEGER16 val);
		void writeI(INTEGER16 val);
		void writeD(INTEGER16 val);
		void writeVFF(UNSIGNED16 val);
		void writeAFF(UNSIGNED16 val);

		INTEGER16 readPcurrent();
		INTEGER16 readIcurrent();
		void writePcurrent(INTEGER16 val);
		void writeIcurrent(INTEGER16 val);

		void saveParameters();

		INTEGER32 readHomePosition();
		void writeHomePosition(INTEGER32 val);

		UNSIGNED16 readMotorContinousCurrentLimit();
		void writeMotorContinousCurrentLimit(UNSIGNED16 cur);

		UNSIGNED16 readMotorOutputCurrentLimit();
		void writeMotorOutputCurrentLimit(UNSIGNED16 cur);

		UNSIGNED8 readMotorPolePair();
		void writeMotorPolePair(UNSIGNED8 cur);

		UNSIGNED32 readMotorMaxSpeedCurrent();
		void writeMotorMaxSpeedCurrent(UNSIGNED32 cur);

		UNSIGNED16 readMotorThermalConstant();
		void writeMotorThermalConstant(UNSIGNED16 cur);

		/*! < by Martí Morta /> */

		/*! \brief read actual position; 14.1.67 */
		INTEGER32 readDemandVelocity();

		/*! \brief read actual position; 14.1.68 */
		INTEGER32 readActualVelocity();

		/*! \brief read actual current; 14.1.69 */
		INTEGER16 readActualCurrent();

		/*! \brief read target position; 14.1.70 */
		INTEGER32 readTargetPosition();

		//! Homing method
	    typedef enum _homing_method {
	    	HM_ACTUAL_POSITION                                 = 35,
	    	HM_NEGATIVE_LIMIT_SWITCH                           = 17,
	    	HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX                 = 1,
			HM_POSITIVE_LIMIT_SWITCH                           = 18,
			HM_POSITIVE_LIMIT_SWITCH_AND_INDEX                 = 2,
			HM_HOME_SWITCH_POSITIVE_SPEED                      = 23,
			HM_HOME_SWITCH_POSITIVE_SPEED_AND_INDEX            = 7,
			HM_HOME_SWITCH_NEGATIVE_SPEED                      = 27,
			HM_HOME_SWITCH_NEGATIVE_SPEED_AND_INDEX            = 11,
			HM_CURRENT_THRESHOLD_POSITIVE_SPEED                = -3,
			HM_CURRENT_THRESHOLD_POSITIVE_SPEED_AND_INDEX      = -1,
			HM_CURRENT_THRESHOLD_NEGATIVE_SPEED                = -4,
			HM_CURRENT_THRESHOLD_NEGATIVE_SPEED_AND_INDEX      = -2,
			HM_INDEX_POSITIVE_SPEED                            = 34,
			HM_INDEX_NEGATIVE_SPEED                            = 33
	    } homing_method_t;

		/*! \brief does a homing move. Give homing mode (see firmware 9.3) and start position */
		int doHoming(homing_method_t method, INTEGER32 start);

		/*! \brief set OpMode to ProfilePosition and make relative movement */
		void moveRelative(INTEGER32 steps);

		/*! \brief set OpMode to ProfilePosition and make absolute movement */
		void moveAbsolute(INTEGER32 steps);

		/*! \brief reads position, velocity and current and displays them in an
		 endless loop. Returns after target position has been reached */
		void monitorStatus();

		/*! \brief as monitorStatus(), but also waits for Homing Attained' signal */
		void monitorHomingStatus();

		/*! \brief waits for positoning to finish, argument is timeout in
		 seconds. give timeout==0 to disable timeout */
		int waitForTarget(unsigned int t);
};

#endif
