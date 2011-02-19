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
#include <cstdlib>
#include <stdint.h>  /* int types with given size */
#include <cmath>

#include <boost/type_traits/is_same.hpp>

#include <string>
#include <exception>
#include <vector>

// Include for BYTE/WORD/DWORD typedefs
#include "epos_access.h"

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

/*!
 * Data types used for object dictionary (Firmware Specification reference)
 */

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

//! \brief interface to EPOS MAXON controller
class epos
{
private:
	/* Implement read functions defined in EPOS Communication Guide, 6.3.1 */
	/* [ one simplification: Node-ID is always 0] */

	/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.1
	 *
	 * @param ans answer buffer
	 * @param length of answer buffer
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @return answer array from the controller
	 */
	unsigned int ReadObject(WORD *ans, unsigned int ans_len, WORD index, BYTE subindex);

	/*! \brief Read Object Value from EPOS memory, firmware definition 6.3.1.1
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @return object value
	 */
	template <class T>
	T ReadObjectValue(WORD index, BYTE subindex)
	{
		WORD answer[8];
		ReadObject(answer, 8, index, subindex);

		// check error code
		checkEPOSerror(device.E_error);

#ifdef DEBUG
		T val;
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
	void WriteObject(WORD index, BYTE subindex, const WORD data[2]);

	/*! \brief write object value to EPOS
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param data0 first WORD of the object
	 * @param data1 second WORD of the object
	 */
	void WriteObjectValue(WORD index, BYTE subindex, uint32_t data);

	/*! \brief compare two 16bit bitmasks
	 *
	 * @return result of comparison */
	static bool bitcmp(WORD a, WORD b);

	//! Object to access the device
	epos_access & device;

	//! ID of the EPOS device on the CAN bus
	const uint8_t nodeId;

public:
	/*! \brief create new USB EPOS object
	 *
	 * @param _device object to access the device
	 * @param _nodeId ID of the EPOS device on the CAN bus
	 */
	epos(epos_access & _device, uint8_t _nodeId);

	/*! \brief check global variable E_error for EPOS error code */
	static void checkEPOSerror(DWORD E_error);

	/*! \brief check if the connection to EPOS is alive */
	//		int checkEPOS();

	/*! \brief check EPOS status
	 * @return state according to firmware spec */
	int checkEPOSstate();

	//! Find EPOS state corresponding to given status word
	static int status2state(WORD w);

	//! Utility routine to pretty print device state
	static const char * stateDescription(int state);

	/*! \brief pretty-print EPOS state
	 *
	 * @retval 0 status is OK
	 * @retval -1 status is unknown
	 */
	int printEPOSstate();

	/*! pretty-print EPOS Error Register */
	static void printErrorRegister(UNSIGNED8 reg);

	//! Seconds per minute -- used in motion profile calculations,
	//! since EPOS velocity is in [rpm] and acceleration is in [rpm/s].
	static const unsigned SECONDS_PER_MINUTE;

	//! \brief States of the EPOS controller
	typedef enum _state
	{
		SHUTDOWN, SWITCH_ON, SWITCH_ON_AND_ENABLE, DISABLE_VOLTAGE,
		QUICKSTOP, DISABLE_OPERATION, ENABLE_OPERATION, FAULT_RESET
	} state_t;

	//! Reset the device by issuing a shutdown command followed by power-on and halt
	void reset();

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
	static void printEPOSstatusword(WORD statusword);

	/*! \brief read EPOS control word (firmware spec 14.1.57) */
	UNSIGNED16 readControlword();

	//! write EPOS control word
	void writeControlword(UNSIGNED16 val);

	/*! \brief pretty-print controlword */
	void printEPOScontrolword(WORD controlword);

	//! \brief start motion with absolute demanded position
	void startAbsoluteMotion();

	//! \brief start motion with relative demanded position
	void startRelativeMotion();

	//! \brief EPOS Operational mode
	typedef enum _operational_mode
	{
		OMD_INTERPOLATED_POSITION_MODE = 7,
		OMD_HOMING_MODE = 6, //! homing
		OMD_PROFILE_VELOCITY_MODE = 3, //! profile velocity mode
		OMD_PROFILE_POSITION_MODE = 1, //! profile position mode
		OMD_POSITION_MODE = -1, //! position mode
		OMD_VELOCITY_MODE = -2, //! velocity mode
		OMD_CURRENT_MODE = -3, //! current mode
		//OMD_DIAGNOSTIC_MODE = -4			//! diagnostic mode
		OMD_MASTER_ENCODER_MODE = -5,
		OMD_STEP_DIRECTION_MODE = -6
	} operational_mode_t;

	/*! \brief set EPOS mode of operation -- 14.1.59 */
	void setOperationMode(operational_mode_t);

	/*! \brief read and returns  EPOS mode of operation -- 14.1.60
	 *
	 * @return 0 MEANS ERROR; '-1' is a valid OpMode, but 0 is not!
	 */
	operational_mode_t readActualOperationMode();

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

	//! write velocity normally attained at the end of the acceleration ramp during a profiled move
	void writeProfileVelocity(UNSIGNED32 vel);

	//! write acceleration ramp during a movement
	void writeProfileAcceleration(UNSIGNED32 acc);

	//! write deceleration ramp during a movement
	void writeProfileDeceleration(UNSIGNED32 dec);

	//! write deceleration ramp during a Quickstop
	void writeQuickStopDeceleration(UNSIGNED32 qsdec);

	//! write maximal allowed speed
	void writeMaxProfileVelocity(UNSIGNED32 maxvel);

	//! write maximal allowed acceleration
	void writeMaxAcceleration(UNSIGNED32 maxvel);

	/*! \brief write position profile type
	 *
	 * @param type 0: linear ramp (trapezoidal profile), 1: sin^2 ramp (sinusoidal profile)
	 */
	void writePositionProfileType(INTEGER16 type);

	//! \brief read velocity normally attained at the end of the acceleration ramp during a profiled move
	UNSIGNED32 readProfileVelocity();

	//! \brief read acceleration ramp during a movement
	UNSIGNED32 readProfileAcceleration();

	//! \brief read deceleration ramp during a movement
	UNSIGNED32 readProfileDeceleration();

	//! \brief read deceleration ramp during a Quickstop
	UNSIGNED32 readQuickStopDeceleration();

	//! \brief read maximal allowed speed
	UNSIGNED32 readMaxProfileVelocity();

	//! \brief read maximal allowed acceleration
	UNSIGNED32 readMaxAcceleration();

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

	//! \brief read P value of the PID regulator
	INTEGER16 readP();

	//! \brief read I value of the PID regulator
	INTEGER16 readI();

	//! \brief read V value of the PID regulator
	INTEGER16 readD();

	//! \brief read Velocity Feed Forward value of the PID regulator
	UNSIGNED16 readVFF();

	//! \brief read Acceleration Feed Forward value of the PID regulator
	UNSIGNED16 readAFF();

	//! \brief write P value of the PID regulator
	void writeP(INTEGER16 val);

	//! \brief write I value of the PID regulator
	void writeI(INTEGER16 val);

	//! \brief write D value of the PID regulator
	void writeD(INTEGER16 val);

	//! \brief write Velocity Feed Forward value of the PID regulator
	void writeVFF(UNSIGNED16 val);

	//! \brief write Acceleration Feed Forward value of the PID regulator
	void writeAFF(UNSIGNED16 val);

	//! \brief read P value of the PI current regulator
	INTEGER16 readPcurrent();

	//! \brief read I value of the PI current regulator
	INTEGER16 readIcurrent();

	//! \brief write P value of the PI current regulator
	void writePcurrent(INTEGER16 val);

	//! \brief write I value of the PI current regulator
	void writeIcurrent(INTEGER16 val);

	//! \brief save actual parameters in non-volatile memory
	void saveParameters();

	//! \brief read home position
	INTEGER32 readHomePosition();

	//! \brief write home position
	void writeHomePosition(INTEGER32 val);

	//! \brief read motor continuous current limit
	UNSIGNED16 readMotorContinousCurrentLimit();

	//! \brief write motor continuous current limit
	void writeMotorContinousCurrentLimit(UNSIGNED16 cur);

	//! \brief read motor output current limit
	UNSIGNED16 readMotorOutputCurrentLimit();

	//! \brief write motor output current limit
	void writeMotorOutputCurrentLimit(UNSIGNED16 cur);

	//! \brief read motor pole pair number
	UNSIGNED8 readMotorPolePairNumber();

	//! \brief write motor pole pair number
	void writeMotorPolePairNumber(UNSIGNED8 cur);

	//! \brief read motor max speed current
	UNSIGNED32 readMotorMaxSpeed();

	//! \brief write motor max speed current
	void writeMotorMaxSpeed(UNSIGNED32 val);

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

	/*! \brief read target position; 14.1.70 */
	void writeTargetPosition(INTEGER32 val);

	/*! read Maximal Following Error */
	UNSIGNED32 readMaxFollowingError();

	/*! write Maximal Following Error */
	void writeMaxFollowingError(UNSIGNED32 val);

	/*! read Home Offset */
	INTEGER32 readHomeOffset();

	/*! write Home Offset */
	void writeHomeOffset(INTEGER32 val);

	/*! read Speed for Switch Search */
	UNSIGNED32 readSpeedForSwitchSearch();

	/*! write Speed for Switch Search */
	void writeSpeedForSwitchSearch(UNSIGNED32 val);

	/*! read Speed for Zero Search */
	UNSIGNED32 readSpeedForZeroSearch();

	/*! write Speed for Zero Search */
	void writeSpeedForZeroSearch(UNSIGNED32 val);

	/*! read Homing Acceleration */
	UNSIGNED32 readHomingAcceleration();

	/*! write Homing Acceleration  */
	void writeHomingAcceleration(UNSIGNED32 val);

	/*! read Current Threshold for Homing Mode */
	UNSIGNED16 readCurrentThresholdForHomingMode();

	/*! write Current Threshold for Homing Mode  */
	void writeCurrentThresholdForHomingMode(UNSIGNED16 val);

	/*! read Error register */
	UNSIGNED8 readErrorRegister();

	/*! read number of Errors is Error History register */
	UNSIGNED8 readNumberOfErrors();

	/*! read Error History at index */
	UNSIGNED32 readErrorHistory(unsigned int num);

	/*! clear Error register */
	void clearNumberOfErrors();

	/*! Store all device parameters in a non-volatile memory */
	void Store();

	/*! All device parameters will be restored with default values */
	void Restore();

	/*! \brief Read the Minimal Position Limit
	 * If the desired or the actual position is lower then the negative position
	 * limit a software position limit Error will be launched.
	 */
	INTEGER32 readMinimalPositionLimit();

	/*! Write the Minimal Position Limit */
	void writeMinimalPositionLimit(INTEGER32 val);

    /*! Read the Maximal Position Limit */
	INTEGER32 readMaximalPositionLimit();

	/*! Write the Maximal Position Limit */
	void writeMaximalPositionLimit(INTEGER32 val);

	// Gear Configuration

	//! read Gear Ratio Numerator
	UNSIGNED32 readGearRatioNumerator();

	//! write Gear Ratio Numerator
	void writeGearRatioNumerator(UNSIGNED32 val);

	//! read Gear Ratio Denominator
	UNSIGNED16 readGearRatioDenominator();

	//! write Gear Ratio Denominator
	void writeGearRatioDenominator(UNSIGNED16 val);

	//! read Gear Maximal Speed
	UNSIGNED32 readGearMaximalSpeed();

	//! write Gear Maximal Speed
	void writeGearMaximalSpeed(UNSIGNED32 val);

	/*!
	 * Interpolated Profile Motion mode commands
	 */

	/*! Provides the actual free buffer size and is given in interpolation data records */
	UNSIGNED32 readActualBufferSize();

	/*! Clear a buffer and reenable access to it */
	void clearPvtBuffer();

	static const char * ErrorCodeMessage(UNSIGNED32 code);

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

	//! Is the actual position referenced to home position
	bool isReferenced();

	//! Is the movement target reached
	bool isTargetReached();

	//! Start homing according to preset parameters
	void startHoming();

	//! Check if both homing target is reached and homing is attained
	bool isHomingFinished();

	/*! read Homing Method */
	homing_method_t readHomingMethod();

	/*! write Homing Method */
	void writeHomingMethod(homing_method_t method);

	/*! \brief does a homing move. Give homing mode (see firmware 9.3) and start position */
	int doHoming(homing_method_t method, INTEGER32 offset = 0);

	/*! \brief set OpMode to ProfilePosition and make relative movement */
	void moveRelative(INTEGER32 steps);

	/*! \brief set OpMode to ProfilePosition and make absolute movement */
	void moveAbsolute(INTEGER32 steps);

	/*! \brief reads position, velocity and current and displays them in an
	 endless loop. Returns after target position has been reached */
	void monitorStatus();

	/*! \brief as monitorStatus(), but also waits for Homing Attained' signal */
	void monitorHomingStatus();

	/*! \brief waits for positioning to finish, argument is timeout in
	 seconds. give timeout==0 to disable timeout */
	int waitForTarget(unsigned int t);

private:
	//! Cached for parameters values
	//! @note have to be at the bottom because some typedefs are defined above
	operational_mode_t OpMode;
	INTEGER16 PositionProfileType;
	UNSIGNED32 ProfileVelocity;
	UNSIGNED32 ProfileAcceleration;
	UNSIGNED32 ProfileDeceleration;
};

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */

//! @}

#endif
