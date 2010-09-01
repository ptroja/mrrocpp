/*! \file epos.h

 \brief libEPOS - a library to control an EPOS; definition

 \b libEPOS is a GNU/Linux C library to control an EPOS motor control
 unit by maxon motor. Since maxon does not offer linux software for
 their products, I wrote this library from scratch.

 It based on the following maxon motor documents:
 - EPOS Positioning Controller - Firmware specification (Edition April 2006)
 - EPOS Positioning Controller - Communication Guide (Edition January 2005)
 - EPOS Positioning Controller - Application Note "Device Programming" (Edition February 2006)


 The only fully implemented and tested "Operation Mode" is "Profile
 Position Mode", but adding support for other OpModes should be fairly
 easy, since the main work was implementing the data exchange with
 EPOS.

 I have only checked the library to work with an EPOS 24/1 (firmware
 v2024). Since I have no access to other hardware, I have no chance to
 check other EPOS versions. But there is no hint at all that it should
 NOT work with other EPOS variants.

 \date July 2006
 \author Marcus Hauser, LSW Heidelberg
 \author Martí Morta <mmorta@iri.upc.edu>
 \author Piotr Trojanek <piotr.trojanek@gmail.com>, Warsaw University of Technology

 \defgroup libEPOS

 @{
 */

#ifndef _EPOS_H
#define _EPOS_H

#include <cstdio>   /* Standard input/output definitions */
#include <cstring>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <cerrno>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <cstdlib>
#include <stdint.h>  /* int types with given size */
#include <cmath>

#include <boost/exception.hpp>
#include <boost/type_traits/is_same.hpp>

#include <string>
#include <exception>
#include <vector>

#include <ftdi.hpp>

/* added oct06 for openTCPEPOS() */
/*
 #include <sys/types.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>
 */

namespace mrrocpp {
namespace edp {
namespace epos {

/* all EPOS data exchange is based on 16bit words, but other types are
 also used...*/
typedef uint32_t DWORD; ///< \brief 32bit type for EPOS data exchange
typedef uint16_t WORD; ///< \brief 16bit type for EPOS data exchange
#ifndef CPP
typedef char BYTE; ///< \brief 8bit type for EPOS data exchange
#endif

//! signed 8-bit integer
typedef int8_t INTEGER8;

//! signed 16-bit integer
typedef int16_t INTEGER16;

//! signed 32-bit integer
typedef int32_t INTEGER32;

//! unsigned 8-bit integer
typedef uint8_t UNSIGNED8;

//! unsigned 16-bit integer
typedef uint16_t UNSIGNED16;

//! unsigned 32-bit integer
typedef uint32_t UNSIGNED32;

/* EPOS will reset communication after 500ms of inactivity */

/*! \brief try NTRY times to read one byte from EPOS, the give up */
#define NTRY      5

/*! \brief wait TRYSLEEP usec between read() from EPOS, if no data available */
#define TRYSLEEP  (unsigned int)1e5

//! all high-level methods throws this exception in case of error
struct epos_error : virtual public std::exception, virtual public boost::exception
{
	~epos_error() throw ()
	{
	}
	;
};

//! reason of an exception
typedef boost::error_info <struct tag_reason, std::string> reason;

//! errno code of a failed system call
typedef boost::error_info <struct tag_errno_code, int> errno_code;

//! failed system call
typedef boost::error_info <struct tag_errno_code, std::string> errno_call;

//! \brief interface to EPOS MAXON controller
class epos
{
private:
	/* Implement read functions defined in EPOS Communication Guide, 6.3.1 */
	/* [ one simplification: Node-ID is always 0] */

	//! device name of EPOS port
	const std::string device;

	//! serial port settings
	struct termios options;

	//! USB FTDI context
	struct ftdi_context ftdic;

	bool device_opened;

	//! EPOS global error status
	DWORD E_error;

	//! EPOS file descriptor
	int ep;

	//! for internal progress character handling
	char gMarker;

	/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.1
	 *
	 * @param ans answer buffer
	 * @param lenght of answer buffer
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @return answer array from the controller
	 */
	unsigned int ReadObject(WORD *ans, unsigned int ans_len, WORD index, BYTE subindex, uint8_t nodeId = 1);

	/*! \brief Read Object Value from EPOS memory, firmware definition 6.3.1.1
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @return object value
	 */
	template <class T>
	T ReadObjectValue(WORD index, BYTE subindex, uint8_t nodeId = 0)
	{
		WORD answer[8];
		ReadObject(answer, 8, index, subindex, nodeId);

		// check error code
		checkEPOSerror();

#ifdef DEBUG
		printf("ReadObjectValue(%0x04x, 0x02x)==> %d\n", val);
#endif

		if ((boost::is_same <T, uint8_t>::value) || (boost::is_same <T, int8_t>::value)
				|| (boost::is_same <T, uint16_t>::value) || (boost::is_same <T, int16_t>::value)) {
			T val = (T) answer[3];
			return val;
		} else if ((boost::is_same <T, uint32_t>::value) || (boost::is_same <T, int32_t>::value)) {
			T val = (T) (answer[3] | (answer[4] << 16));
			return val;
		} else {
			throw epos_error() << reason("Unsupported ReadObjectValue conversion");
		}
	}

#if 0
	/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.2
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 */
	int InitiateSegmentedRead(WORD index, BYTE subindex );

	/*! \brief read data segment of the object initiated with 'InitiateSegmentedRead()'
	 *
	 * @param ptr pointer to data to be filled
	 */
	int SegmentRead(WORD **ptr);
#endif

	/*! \brief write obect to EPOS
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param data pointer to a 2 WORDs array (== 4 BYTES) holding data to transmit
	 */
	void WriteObject(WORD index, BYTE subindex, const WORD data[2], uint8_t nodeId = 1);

	/*! \brief write object value to EPOS
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param data0 first WORD of the object
	 * @param data1 second WORD of the object
	 */
	void WriteObjectValue(WORD index, BYTE subindex, uint32_t data);

	/* helper functions below */

	/*! \brief write a single BYTE to EPOS
	 *
	 * @param c BYTE to write
	 */
	void writeBYTE(BYTE c);

	/*! \brief  write a single WORD to EPOS
	 *
	 * @param w WORD to write
	 */
	void writeWORD(WORD w);

	/*! \brief  read a single BYTE from EPOS, timeout implemented
	 *
	 * @return readed data BYTE
	 */
	BYTE readBYTE();

	/*! \brief  read a single WORD from EPOS, timeout implemented
	 *
	 * @return readed data BYTE
	 */
	WORD readWORD();

	/*! \brief  send command to EPOS, taking care of all neccessary 'ack' and checksum tests
	 *
	 * @param frame array of WORDs to write
	 */
	void sendCommand(WORD *frame);

	/*! \brief  read an answer frame from EPOS
	 *
	 * @return answer array from the controller
	 */
	unsigned int readAnswer(WORD *ans, unsigned int ans_len);

	/*! \brief check global variable E_error for EPOS error code */
	int checkEPOSerror();

	/*! \brief Checksum calculation
	 *
	 * Copied from EPOS Communication Guide, p.8
	 *
	 * @param pDataArray pointer to data for checksum calculcation
	 * @param numberOfWords lenght of the data
	 */
	WORD CalcFieldCRC(const WORD *pDataArray, WORD numberOfWords) const;

	/*! \brief compare two 16bit bitmasks
	 *
	 * @return result of comparision */
	bool bitcmp(WORD a, WORD b) const;

	//! USB device indentifiers
	const int vendor, product, index;

public:
	/*! \brief create new EPOS object
	 *
	 * @param _device device string describing the device on which the EPOS is connected to, e.g. "/dev/ttyS0"
	 */
	epos(const std::string & _device);

	/*! \brief create new USB EPOS object
	 *
	 * @param vendor USB device vendor ID
	 * @param product USB device vendor ID
	 * @param index USB device vendor ID
	 */
	epos(int _vendor = 0x0403, int _product = 0xa8b0, unsigned int index = 0);

	/*! \brief delete EPOS object */
	~epos();

	/*! \brief establish serial connection to EPOS
	 */
	void openEPOS(speed_t speed);

	/*! \brief establish USB connection to EPOS2
	 */
	void openEPOS();

	/*! \brief close the connection to EPOS
	 */
	void closeEPOS();

	/*! \brief check if the connection to EPOS is alive */
	//		int checkEPOS();

	/*! \brief check EPOS status
	 *
	 * @return state according to firmware spec */
	int checkEPOSstate();

	/*! \brief pretty-print EPOS state
	 *
	 * @retval 0 status is OK
	 * @retval -1 status is unknown
	 */
	int printEPOSstate();

	//! \brief States of the EPOS controller
	typedef enum _state
	{
		ST_DISABLED = 0, ST_ENABLED = 1, ST_QUICKSTOP = 2, ST_FAULT = 3
	} state_t;

	/*! \brief change EPOS state   ==> firmware spec 8.1.3 */
	void changeEPOSstate(state_t state);

	/*! \brief ask EPOS for software version */
	UNSIGNED16 readSWversion();

	/*! \brief read manufactor device name string firmware */
	std::string readDeviceName();

	/*! \brief ask for RS232 timeout; firmware spec 14.1.35 */
	UNSIGNED16 readRS232timeout();

	/*! \brief read digital input polarity mask */
	UNSIGNED16 readDInputPolarity();

	/*! \brief set home switch polarity -- firmware spec 14.1.47 */
	void setHomePolarity(int pol);

	/*! \brief read Statusword; 14.1.58 */
	UNSIGNED16 readStatusWord();

	/*! \brief pretty-print statusword to stdout
	 *
	 * \param statusword WORD variable holding the statusword
	 */
	void printEPOSstatusword(WORD statusword);

	/*! \brief read EPOS control word (firmware spec 14.1.57) */
	UNSIGNED16 readControlword();

	/*! \brief pretty-print controlword */
	void printEPOScontrolword(WORD controlword);

	//! \brief EPOS Operational mode
	typedef enum _operational_mode
	{
		OMD_PROFILE_POSITION_MODE = 1, //! profile position mode
		OMD_PROFILE_VELOCITY_MODE = 3, //! profile velocity mode
		OMD_HOMING_MODE = 6, //! homing
		OMD_INTERPOLATED_POSITION_MODE = 7,
		OMD_POSITION_MODE = -1, //! position mode
		OMD_VELOCITY_MODE = -2, //! velocity mode
		OMD_CURRENT_MODE = -3, //! current mode
		//OMD_DIAGNOSTIC_MODE = -4			//! diagnostic mode
		OMD_MASTER_ENCODER_MODE = -5,
		OMD_STEP_DIRECTION_MODE = -6
	} operational_mode_t;

	/*! \brief set EPOS mode of operation -- 14.1.59 */
	void setOpMode(operational_mode_t);

	/*! \brief read and returns  EPOS mode of operation -- 14.1.60
	 *
	 * @return 0 MEANS ERROR; '-1' is a valid OpMode, but 0 is not!
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

	/*! \brief read position window time; 14.1.67 */
	//		int writePositionWindowTime(unsigned int val);

	//		int writePositionSoftwareLimits(long val, long val2);

	//! write position profile velocity
	void writePositionProfileVelocity(UNSIGNED32 vel);

	//! write position profile acceleration
	void writePositionProfileAcceleration(UNSIGNED32 acc);

	//! write position profile deceleration
	void writePositionProfileDeceleration(UNSIGNED32 dec);

	//! write position profile quict stop deceleration
	void writePositionProfileQuickStopDeceleration(UNSIGNED32 qsdec);

	//! write position profile max velocity
	void writePositionProfileMaxVelocity(UNSIGNED32 maxvel);

	/*! \brief write position profile type
	 *
	 * @param type 0: linear ramp (trapezoidal profile), 1: sin^2 ramp (sinusoidal profile)
	 */
	void writePositionProfileType(INTEGER16 type);

	//! \brief read position profile velocity
	UNSIGNED32 readPositionProfileVelocity();

	//! \brief read position profile acceleration
	UNSIGNED32 readPositionProfileAcceleration();

	//! \brief read position profile deceleration
	UNSIGNED32 readPositionProfileDeceleration();

	//! \brief read position profile quick stop decelration
	UNSIGNED32 readPositionProfileQuickStopDeceleration();

	//! \brief read position profile max velocity
	UNSIGNED32 readPositionProfileMaxVelocity();

	//! \brief read position profile type
	INTEGER16 readPositionProfileType();

	//! \brief velocity notation
	typedef enum _velocity_notation
	{
		VN_STANDARD = 0, VN_DECI = -1, VN_CENTI = -2, VN_MILLI = -3
	} velocity_notation_t;

	//! \brief read velocity notation
	velocity_notation_t readVelocityNotationIndex();

	//! \brief read velocity notation index
	void writeVelocityNotationIndex(velocity_notation_t val);

	//! sensor type
	typedef enum _sensor_type
	{
		ST_UNKNOWN = 0,
		ST_INC_ENCODER_3CHANNEL = 1,
		ST_INC_ENCODER_2CHANNEL = 2,
		ST_HALL_SENSORS = 3,
		ST_SSI_ABS_ENCODER_BINARY = 4,
		ST_SSI_ABS_ENCODER_GREY = 5
	} sensor_type_t;

	//! \brief read sensor pulses
	UNSIGNED32 readSensorPulses();

	//! \brief read sensor type
	sensor_type_t readSensorType();

	//! \brief read sensor polarity
	UNSIGNED16 readSensorPolarity();

	//! \brief write sensor type
	void writeSensorType(sensor_type_t val);

	//! \brief write sensor pulses
	void writeSensorPulses(UNSIGNED32 val);

	//! \brief write sensor polarity
	void writeSensorPolarity(UNSIGNED16 val);

	//! \brief read RS232 baudrate
	UNSIGNED16 readRS232Baudrate();

	//! \brief write RS232 baudrate
	void writeRS232Baudrate(UNSIGNED16 val);

	//! \brief read P value of the PID regularor
	INTEGER16 readP();

	//! \brief read I value of the PID regularor
	INTEGER16 readI();

	//! \brief read V value of the PID regularor
	INTEGER16 readD();

	//! \brief read Velocity Feed Forward value of the PID regularor
	UNSIGNED16 readVFF();

	//! \brief read Acceleration Feed Forward value of the PID regulator
	UNSIGNED16 readAFF();

	//! \brief write P value of the PID regularor
	void writeP(INTEGER16 val);

	//! \brief write I value of the PID regularor
	void writeI(INTEGER16 val);

	//! \brief write D value of the PID regularor
	void writeD(INTEGER16 val);

	//! \brief write Velocity Feed Forward value of the PID regularor
	void writeVFF(UNSIGNED16 val);

	//! \brief write Acceleration Feed Forward value of the PID regularor
	void writeAFF(UNSIGNED16 val);

	//! \brief read P value of the PI current regularor
	INTEGER16 readPcurrent();

	//! \brief read I value of the PI current regularor
	INTEGER16 readIcurrent();

	//! \brief write P value of the PI current regularor
	void writePcurrent(INTEGER16 val);

	//! \brief write I value of the PI current regularor
	void writeIcurrent(INTEGER16 val);

	//! \brief save actual parameters in non-volatile memory
	void saveParameters();

	//! \brief read home position
	INTEGER32 readHomePosition();

	//! \brief write home position
	void writeHomePosition(INTEGER32 val);

	//! \brief read motor continous current limit
	UNSIGNED16 readMotorContinousCurrentLimit();

	//! \brief write motor continous current limit
	void writeMotorContinousCurrentLimit(UNSIGNED16 cur);

	//! \brief read motor output current limit
	UNSIGNED16 readMotorOutputCurrentLimit();

	//! \brief write motor output current limit
	void writeMotorOutputCurrentLimit(UNSIGNED16 cur);

	//! \brief read motor pole pair number
	UNSIGNED8 readMotorPolePair();

	//! \brief write motor pole pair number
	void writeMotorPolePair(UNSIGNED8 cur);

	//! \brief read motor max speed current
	UNSIGNED32 readMotorMaxSpeedCurrent();

	//! \brief write motor max speed current
	void writeMotorMaxSpeedCurrent(UNSIGNED32 val);

	//! \brief read motor thermal constant
	UNSIGNED16 readMotorThermalConstant();

	//! \brief write motor thermal constant
	void writeMotorThermalConstant(UNSIGNED16 val);

	/*! \brief read actual position; 14.1.67 */
	INTEGER32 readDemandVelocity();

	/*! \brief read actual position; 14.1.68 */
	INTEGER32 readActualVelocity();

	/*! \brief read actual current; 14.1.69 */
	INTEGER16 readActualCurrent();

	/*! \brief read target position; 14.1.70 */
	INTEGER32 readTargetPosition();

	//! Homing method
	typedef enum _homing_method
	{
		HM_ACTUAL_POSITION = 35,
		HM_NEGATIVE_LIMIT_SWITCH = 17,
		HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX = 1,
		HM_POSITIVE_LIMIT_SWITCH = 18,
		HM_POSITIVE_LIMIT_SWITCH_AND_INDEX = 2,
		HM_HOME_SWITCH_POSITIVE_SPEED = 23,
		HM_HOME_SWITCH_POSITIVE_SPEED_AND_INDEX = 7,
		HM_HOME_SWITCH_NEGATIVE_SPEED = 27,
		HM_HOME_SWITCH_NEGATIVE_SPEED_AND_INDEX = 11,
		HM_CURRENT_THRESHOLD_POSITIVE_SPEED = -3,
		HM_CURRENT_THRESHOLD_POSITIVE_SPEED_AND_INDEX = -1,
		HM_CURRENT_THRESHOLD_NEGATIVE_SPEED = -4,
		HM_CURRENT_THRESHOLD_NEGATIVE_SPEED_AND_INDEX = -2,
		HM_INDEX_POSITIVE_SPEED = 34,
		HM_INDEX_NEGATIVE_SPEED = 33
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

}
} /* namespace edp */
} /* namespace mrrocpp */

//! @}

#endif
