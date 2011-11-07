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

 * \defgroup libEPOS Library for low-level access to the EPOS motion controller
 * @{
 */

#ifndef _EPOS_H
#define _EPOS_H

#include <cstdio>   /* Standard input/output definitions */
#include <cstdlib>
#include <stdint.h>  /* int types with given size */

#include <string>
#include <exception>
#include <vector>

// Include for BYTE/WORD/DWORD typedefs
#include "robot/canopen/gateway.h"

/* added oct06 for openTCPEPOS() */
/*
 #include <sys/types.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>
 */

namespace mrrocpp {
namespace edp {
namespace maxon {

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
	/* Implement read functions defined in EPOS Communication Guide */
	/* [ one simplification: Node-ID is always 0] */

	/*! \brief read Object Value from EPOS memory
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @return object value
	 */
	template <class T>
	T ReadObjectValue(canopen::WORD index, canopen::BYTE subindex)
	{
		return device.ReadObjectValue<T>(nodeId, index, subindex);
	}

	/*! \brief write object value to EPOS
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param data object data
	 */
	template<class T>
	void WriteObjectValue(canopen::WORD index, canopen::BYTE subindex, T data)
	{
		device.WriteObject(nodeId, index, subindex, (uint32_t) data);
	}

	/*! \brief Initiate Write Object to EPOS memory (for 5 bytes and more)
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 */
	void InitiateSegmentedWrite(canopen::WORD index, canopen::BYTE subindex, canopen::DWORD ObjectLength);

	/*! \brief write data segment of the object initiated with 'InitiateSegmentedWrite()'
	 *
	 * @param ptr pointer to data to be filled
	 * @param len length of the data to write
	 */
	void SegmentedWrite(canopen::BYTE * ptr, std::size_t len);

	/*! \brief compare two 16bit bitmasks
	 *
	 * @return result of comparison */
	static bool bitcmp(canopen::WORD a, canopen::WORD b);

	//! Object to access the device
	canopen::gateway & device;

	//! ID of the EPOS device on the CAN bus
	const uint8_t nodeId;

	//! remote operation enable bit
	bool remote;

public:
	/*! \brief create new EPOS object
	 *
	 * @param _device object to access the device
	 * @param _nodeId ID of the EPOS device on the CAN bus
	 */
	epos(canopen::gateway & _device, uint8_t _nodeId);

	//! Actual state of the device
	typedef enum _actual_state_t {
		UNKNOWN = -1,
		START = 0,
		NOT_READY_TO_SWITCH_ON = 1,
		SWITCH_ON_DISABLED = 2,
		READY_TO_SWITCH_ON = 3,
		SWITCHED_ON = 4,
		REFRESH = 5,
		MEASURE_INIT = 6,
		OPERATION_ENABLE = 7,
		QUICK_STOP_ACTIVE = 8,
		FAULT_REACTION_ACTIVE_DISABLED = 9,
		FAULT_REACTION_ACTIVE_ENABLED = 10,
		FAULT = 11
	} actual_state_t;

	//! \ingroup libEPOS
	//! \defgroup epos_configuration Configuration
	//! @{

	//! \brief check EPOS status
	//! @return state according to firmware spec
	actual_state_t getState();

	//! \brief Find EPOS state corresponding to given status word
	static actual_state_t status2state(canopen::WORD w);

	//! \brief Check if the remote operation is enabled
	//! @param status status word
	static bool isRemoteOperationEnabled(canopen::WORD status);

	//! \brief Utility routine to pretty print device state
	static const char * stateDescription(int state);

	//! \brief Change the state of the remote (CAN) operation, required for the PDO requests
	void setRemoteOperation(bool enable);

	/*! \brief Prints pretty-formatted controller state.
	 *
	 * @retval 0 status is OK
	 * @retval -1 status is unknown
	 */
	int printState();

	//! \brief Pretty-formatted print EPOS error register.
	static void printErrorRegister(UNSIGNED8 reg);

	//! \brief Seconds per minute -- used in motion profile calculations,
	//! since EPOS velocity is in [rpm] and acceleration is in [rpm/s].
	static const unsigned SECONDS_PER_MINUTE;

	//! \brief Commands that can be send to EPOS controller in order to change its state.
	typedef enum _desired_state
	{
		SHUTDOWN, SWITCH_ON, SWITCH_ON_AND_ENABLE, DISABLE_VOLTAGE,
		QUICKSTOP, DISABLE_OPERATION, ENABLE_OPERATION, FAULT_RESET
	} desired_state_t;

	//! \brief Reset the device by issuing a shutdown command followed by power-on and halt
	void reset();

	/*! \brief change EPOS state */
	void setState(desired_state_t state);

	/*! \brief read CAN Node ID */
	UNSIGNED8 getNodeID();

	/*! \brief ask EPOS for software version */
	UNSIGNED16 getSWversion();

	/*! \brief read manufactor device name string firmware */
	std::string getDeviceName();

	/*! \brief ask for RS232 timeout */
	UNSIGNED16 getRS232timeout();

	/*! \brief read digital input polarity mask */
	UNSIGNED16 getDInputPolarity();

	/*! \brief read digital input*/
	UNSIGNED16 getDInput();

	/*! \brief set home switch polarity */
	void setHomePolarity(int pol);

	/*! \brief read Statusword */
	UNSIGNED16 getStatusWord();

	/*! \brief Pretty-formatted print of status word to stdout.
	 *
	 * \param statusword WORD variable holding the statusword
	 */
	static void printStatusWord(canopen::WORD statusword);

	/*! \brief read EPOS control word */
	UNSIGNED16 getControlword();

	//! write EPOS control word
	void setControlword(UNSIGNED16 val);

	//! \brief Pretty-formatted print of control word to stdout.
	void printControlWord(canopen::WORD controlword);

	//! \brief start motion with absolute demanded position
	void startAbsoluteMotion();

	//! \brief start motion with relative demanded position
	void startRelativeMotion();

	//! \brief EPOS Operational mode
	typedef enum _operational_mode
	{
		OMD_INTERPOLATED_POSITION_MODE = 7,	//! interpolated position mode
		OMD_HOMING_MODE = 6,				//! homing
		OMD_PROFILE_VELOCITY_MODE = 3,		//! profile velocity mode
		OMD_PROFILE_POSITION_MODE = 1,		//! profile position mode
		OMD_POSITION_MODE = -1,				//! position mode
		OMD_VELOCITY_MODE = -2,				//! velocity mode
		OMD_CURRENT_MODE = -3,				//! current mode
		//OMD_DIAGNOSTIC_MODE = -4			//! diagnostic mode
		OMD_MASTER_ENCODER_MODE = -5,		//! master encoder mode
		OMD_STEP_DIRECTION_MODE = -6		//! step/direction mode
	} operational_mode_t;

	/*! \brief set EPOS mode of operation */
	void setOperationMode(operational_mode_t);

	/*! \brief read and returns  EPOS mode of operation
	 *
	 * @return 0 MEANS ERROR; '-1' is a valid OpMode, but 0 is not!
	 */
	operational_mode_t getActualOperationMode();

	/*! \brief read demanded position */
	INTEGER32 getDemandPosition();

	/*! \brief read actual position */
	INTEGER32 getActualPosition();

	/*! \brief read position window */
	UNSIGNED32 getPositionWindow();

	/*! \brief write position window */
	void setPositionWindow(UNSIGNED32 value);

	/*! \brief read position window time */
	//		int writePositionWindowTime(unsigned int val);

	//		int writePositionSoftwareLimits(long val, long val2);

	//! write velocity normally attained at the end of the acceleration ramp during a profiled move
	void setProfileVelocity(UNSIGNED32 vel);

	//! write acceleration ramp during a movement
	void setProfileAcceleration(UNSIGNED32 acc);

	//! write deceleration ramp during a movement
	void setProfileDeceleration(UNSIGNED32 dec);

	//! write deceleration ramp during a Quickstop
	void setQuickStopDeceleration(UNSIGNED32 qsdec);

	//! write maximal allowed speed
	void setMaxProfileVelocity(UNSIGNED32 maxvel);

	//! write maximal allowed acceleration
	void setMaxAcceleration(UNSIGNED32 maxvel);

	/*! \brief write position profile type
	 *
	 * @param type 0: linear ramp (trapezoidal profile), 1: sin^2 ramp (sinusoidal profile)
	 */
	void setPositionProfileType(INTEGER16 type);

	//! \brief read velocity normally attained at the end of the acceleration ramp during a profiled motion
	UNSIGNED32 getProfileVelocity();

	//! \brief read acceleration ramp during a movement
	UNSIGNED32 getProfileAcceleration();

	//! \brief read deceleration ramp during a movement
	UNSIGNED32 getProfileDeceleration();

	//! \brief read deceleration ramp during a Quickstop
	UNSIGNED32 getQuickStopDeceleration();

	//! \brief read maximal allowed speed
	UNSIGNED32 getMaxProfileVelocity();

	//! \brief read maximal allowed acceleration
	UNSIGNED32 getMaxAcceleration();

	//! \brief read position profile type
	INTEGER16 getPositionProfileType();

	//! \brief velocity notation
	typedef enum _velocity_notation
	{
		VN_STANDARD = 0, VN_DECI = -1, VN_CENTI = -2, VN_MILLI = -3
	} velocity_notation_t;

	//! \brief read velocity notation
	velocity_notation_t getVelocityNotationIndex();

	//! \brief read velocity notation index
	void setVelocityNotationIndex(velocity_notation_t val);

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
	UNSIGNED32 getSensorPulses();

	//! \brief read sensor type
	sensor_type_t getSensorType();

	//! \brief read sensor polarity
	UNSIGNED16 getSensorPolarity();

	//! \brief write sensor type
	void setSensorType(sensor_type_t val);

	//! \brief write sensor pulses
	void setSensorPulses(UNSIGNED32 val);

	//! \brief write sensor polarity
	void setSensorPolarity(UNSIGNED16 val);

	//! \brief read RS232 baudrate
	UNSIGNED16 getRS232Baudrate();

	//! \brief write RS232 baudrate
	void setRS232Baudrate(UNSIGNED16 val);

	//! @}

	//! \ingroup libEPOS
	//! \defgroup epos_pid Regulator tuning
	//! @{

	//! \brief read P value of the PID regulator
	INTEGER16 getP();

	//! \brief read I value of the PID regulator
	INTEGER16 getI();

	//! \brief read V value of the PID regulator
	INTEGER16 getD();

	//! \brief read Velocity Feed Forward value of the PID regulator
	UNSIGNED16 getVFF();

	//! \brief read Acceleration Feed Forward value of the PID regulator
	UNSIGNED16 getAFF();

	//! \brief write P value of the PID regulator
	void setP(INTEGER16 val);

	//! \brief write I value of the PID regulator
	void setI(INTEGER16 val);

	//! \brief write D value of the PID regulator
	void setD(INTEGER16 val);

	//! \brief write Velocity Feed Forward value of the PID regulator
	void setVFF(UNSIGNED16 val);

	//! \brief write Acceleration Feed Forward value of the PID regulator
	void setAFF(UNSIGNED16 val);

	//! \brief read P value of the PI current regulator
	INTEGER16 getPcurrent();

	//! \brief read I value of the PI current regulator
	INTEGER16 getIcurrent();

	//! \brief write P value of the PI current regulator
	void setPcurrent(INTEGER16 val);

	//! \brief write I value of the PI current regulator
	void setIcurrent(INTEGER16 val);

	//! @}

	//! \brief save actual parameters in non-volatile memory
	void saveParameters();

	//! \brief read home position
	INTEGER32 getHomePosition();

	//! \brief write home position
	void setHomePosition(INTEGER32 val);

	//! \brief read motor continuous current limit
	UNSIGNED16 setMotorContinousCurrentLimit();

	//! \brief write motor continuous current limit
	void getMotorContinousCurrentLimit(UNSIGNED16 cur);

	//! \brief read motor output current limit
	UNSIGNED16 getMotorOutputCurrentLimit();

	//! \brief write motor output current limit
	void setMotorOutputCurrentLimit(UNSIGNED16 cur);

	//! \brief read motor pole pair number
	UNSIGNED8 getMotorPolePairNumber();

	//! \brief write motor pole pair number
	void setMotorPolePairNumber(UNSIGNED8 cur);

	//! \brief read motor max speed current
	UNSIGNED32 getMotorMaxSpeed();

	//! \brief write motor max speed current
	void setMotorMaxSpeed(UNSIGNED32 val);

	//! \brief read motor thermal constant
	UNSIGNED16 getMotorThermalConstant();

	//! \brief write motor thermal constant
	void setMotorThermalConstant(UNSIGNED16 val);

	/*! \brief read actual position */
	INTEGER32 setDemandVelocity();

	/*! \brief read actual position */
	INTEGER32 getActualVelocity();

	/*! \brief read actual current */
	INTEGER16 getActualCurrent();

	/*! \brief read target position */
	INTEGER32 getTargetPosition();

	/*! \brief read target position */
	void setTargetPosition(INTEGER32 val);

	/*! \brief read Maximal Following Error */
	UNSIGNED32 getMaxFollowingError();

	/*! \brief write Maximal Following Error */
	void setMaxFollowingError(UNSIGNED32 val);

	/*! \brief read Home Offset */
	INTEGER32 getHomeOffset();

	/*! \brief write Home Offset */
	void setHomeOffset(INTEGER32 val);

	/*! \brief read Speed for Switch Search */
	UNSIGNED32 getSpeedForSwitchSearch();

	/*! \brief write Speed for Switch Search */
	void setSpeedForSwitchSearch(UNSIGNED32 val);

	/*! \brief read Speed for Zero Search */
	UNSIGNED32 getSpeedForZeroSearch();

	/*! \brief write Speed for Zero Search */
	void setSpeedForZeroSearch(UNSIGNED32 val);

	/*! \brief read Homing Acceleration */
	UNSIGNED32 getHomingAcceleration();

	/*! \brief write Homing Acceleration  */
	void setHomingAcceleration(UNSIGNED32 val);

	/*! \brief read Current Threshold for Homing Mode */
	UNSIGNED16 getCurrentThresholdForHomingMode();

	/*! \brief write Current Threshold for Homing Mode  */
	void setCurrentThresholdForHomingMode(UNSIGNED16 val);

	/*! \brief read Error register */
	UNSIGNED8 getErrorRegister();

	/*! \brief read number of Errors is Error History register */
	UNSIGNED8 getNumberOfErrors();

	/*! \brief read Error History at index */
	UNSIGNED32 getErrorHistory(unsigned int num);

	/*! \brief clear Error register */
	void clearNumberOfErrors();

	/*! \brief Store all device parameters in a non-volatile memory */
	void Store();

	/*! \brief All device parameters will be restored with default values */
	void Restore();

	//! \ingroup libEPOS
	//! \defgroup epos_limits Position Limits
	//! @{

	/*! \brief read the Minimal Position Limit
	 * If the desired or the actual position is lower then the negative position
	 * limit a software position limit Error will be launched.
	 */
	INTEGER32 getMinimalPositionLimit();

	/*! \brief write the Minimal Position Limit */
	void setMinimalPositionLimit(INTEGER32 val);

    /*! \brief read the Maximal Position Limit */
	INTEGER32 getMaximalPositionLimit();

	/*! \brief write the Maximal Position Limit */
	void setMaximalPositionLimit(INTEGER32 val);

	//! @}

	//! \ingroup libEPOS
	//! \defgroup epos_gear Gear Configuration
	//! @{

	//! \brief read Gear Ratio Numerator
	UNSIGNED32 getGearRatioNumerator();

	//! \brief write Gear Ratio Numerator
	void setGearRatioNumerator(UNSIGNED32 val);

	//! \brief read Gear Ratio Denominator
	UNSIGNED16 getGearRatioDenominator();

	//! \brief write Gear Ratio Denominator
	void setGearRatioDenominator(UNSIGNED16 val);

	//! \brief read Gear Maximal Speed
	UNSIGNED32 getGearMaximalSpeed();

	//! \brief write Gear Maximal Speed
	void setGearMaximalSpeed(UNSIGNED32 val);

	//! @}

	//! \ingroup libEPOS
	//! \defgroup epos_ipm Interpolated Profile Motion
	//! @{

	/*! \brief Provides the actual free buffer size and is given in interpolation data records */
	UNSIGNED32 getActualBufferSize();

	/*! \brief clear a buffer and reenable access to it */
	void clearPvtBuffer();

	//! \brief write Interpolation Sub Mode Selection
	void setInterpolationSubModeSelection(INTEGER16 val);

	//! \brief read Interpolation Sub Mode Selection
	INTEGER16 getInterpolationSubModeSelection();

	//! \brief write Interpolation Time Period Value
	void setInterpolationTimePeriod(UNSIGNED8 val);

	//! \brief read Interpolation Time Period Value
	UNSIGNED8 getInterpolationTimePeriod();

	//! \brief write Interpolation Time Index
	void setInterpolationTimeIndex(INTEGER8 val);

	//! \brief read Interpolation Time Period Index
	INTEGER8 getInterpolationTimeIndex();

	//! \brief write Interpolation data record
	void setInterpolationDataRecord(INTEGER32 position, INTEGER32 velocity, UNSIGNED8 time);

	//! \brief read Interpolation buffer status
	UNSIGNED16 getInterpolationBufferStatus();

	//! \brief check Interpolation Buffer warning
	static bool checkInterpolationBufferWarning(UNSIGNED16 status);

	//! \brief check Interpolation Buffer underflow warning
	static bool checkInterpolationBufferUnderflowWarning(UNSIGNED16 status);

	//! \brief check Interpolation Buffer error
	static bool checkInterpolationBufferError(UNSIGNED16 status);

	//! \brief Print status message about the Interpolation Buffer
	static void printInterpolationBufferStatus(UNSIGNED16 status);

	//! \brief read Interpolation buffer underflow warning
	UNSIGNED16 getInterpolationBufferUnderflowWarning();

	//! write Interpolation buffer underflow warning
	void setInterpolationBufferUnderflowWarning(UNSIGNED16 val);

	//! \brief read Interpolation buffer overflow warning
	UNSIGNED16 getInterpolationBufferOverflowWarning();

	//! \brief write Interpolation buffer overflow warning
	void setInterpolationBufferOverflowWarning(UNSIGNED16 val);

	//! \brief start Interpolated Position Mode motion
	void startInterpolatedPositionMotion();

	//! @}

	//! \brief Get message of the error code
	/*static*/ const char * ErrorCodeMessage(UNSIGNED32 code);

	//! \ingroup libEPOS
	//! \defgroup epos_homing Homing
	//! @{

	//! \brief Homing method
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

	//! \brief rIs the actual position referenced to home position?
	bool isReferenced();

	//! \brief rIs the movement target reached?
	bool isTargetReached();

	//! \brief rStart homing according to preset parameters
	void startHoming();

	//! \brief rCheck if both homing target is reached and homing is attained
	bool isHomingFinished();

	/*! \brief read Homing Method */
	homing_method_t getHomingMethod();

	/*! \brief write Homing Method */
	void setHomingMethod(homing_method_t method);

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

	//! \brief Read analog input 1.
	INTEGER16 getAnalogInput1();

	//! @}

private:
	//! Cached for parameters values
	//! @note have to be at the bottom because some typedefs are defined above
	operational_mode_t OpMode;
	INTEGER16 PositionProfileType;
	UNSIGNED32 ProfileVelocity;
	UNSIGNED32 ProfileAcceleration;
	UNSIGNED32 ProfileDeceleration;
};

} /* namespace maxon */
} /* namespace edp */
} /* namespace mrrocpp */

//! @}

#endif
