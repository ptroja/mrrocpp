/*! \file festo.h

 \brief libfesto - a library to control an FESTO CO2 field bus devices; definition

 \date Septrmber 2011
 \author Piotr Trojanek <piotr.trojanek@gmail.com>, Warsaw University of Technology

 * \defgroup libfesto Library for control of the FESTO field bus devices
 * @{
 */

#ifndef _CPV_H
#define _CPV_H

#include <stdint.h>  /* int types with given size */

#include <string>
#include <exception>

#include "robot/canopen/gateway.h"

namespace mrrocpp {
namespace edp {
namespace festo {

/*!
 * Data types used for object dictionary ('Field bus protocol: CANopen' manual)
 */

////! signed 8-bit integer
//typedef int8_t INTEGER8;
//
////! signed 16-bit integer
//typedef int16_t INTEGER16;
//
////! signed 32-bit integer
//typedef int32_t INTEGER32;

//! unsigned 8-bit integer
typedef uint8_t U8;

//! unsigned 16-bit integer
typedef uint16_t U16;

//! unsigned 32-bit integer
typedef uint32_t U32;

//! \brief interface to FESTO device
class cpv
{
private:
	/* Implement read functions defined in EPOS Communication Guide */

	/*! \brief read Object Value
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

	//! Object to access the device
	canopen::gateway & device;

	//! ID of the device on the CAN bus
	const uint8_t nodeId;

	//! remote operation enable bit
	bool remote;

public:
	/*! \brief create new controller object
	 *
	 * @param _device object to access the device
	 * @param _nodeId ID of the device on the CAN bus
	 */
	cpv(canopen::gateway & _device, uint8_t _nodeId);

	U32 readDeviceType();

	U8 readErrorRegister();

	U32 readManufacturerStatusRegister();

	U8 readNumberOfCurrentFaults();

	/*! \brief read most recent fault table
	 *
	 * @param field field id (0..10)
	 */
	U32 readMostRecentFault(uint8_t field);

	std::string readDeviceName();

	std::string readHardwareVersion();

	std::string readSoftwareVersion();

	/* Hardbeat protocol configuration not implemented */

	U32 readVendorID();

	U32 readProductCode();

	U32 readRevisionNumber();

	U32 readSerialNumber();

	U8 readNumberOfCPModulesConnected();

	/*! \brief read connected CP module type recent fault table
	 *
	 * @param module module id (1..3)
	 */
	U16 readModuleType(uint8_t module);

	/* Digital CP inputs support not implemented */

	/*! \brief read number of 8-output groups
	 *
	 * @return Number of 8-output groups (2 or 4)
	 */
	U8 readNumberOf8OutputGroups();

	/*! \brief read outputs status
	 *
	 * @param group outputs group id (1..4)
	 * @return outputs status
	 */
	U8 readOutputs(uint8_t group);

	/*! \brief read connected CP module type recent fault table
	 *
	 * @param group outputs group id (1..4)
	 */
	void writeOutputs(uint8_t group, uint8_t value);

	U8 readNumberOf8OutputGroupsErrorMode();

	U8 readOutputsErrorMode(uint8_t group);

	void writeOutputsErrorMode(uint8_t group, uint8_t value);

	U8 readNumberOf8OutputGroupsErrorValue();

	U8 readOutputsErrorValue(uint8_t group);

	void writeOutputsErrorValue(uint8_t group, uint8_t value);

#if 0

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
	actual_state_t checkEPOSstate();

	//! \brief Find EPOS state corresponding to given status word
	static actual_state_t status2state(canopen::WORD w);

	//! \brief Check if the remote operation is enabled
	//! @param status status word
	static bool isRemoteOperationEnabled(canopen::WORD status);

	//! \brief Utility routine to pretty print device state
	static const char * stateDescription(int state);

	//! \brief Change the state of the remote (CAN) operation, required for the PDO requests
	void setRemoteOperation(bool enable);

	/*! \brief pretty-print EPOS state
	 *
	 * @retval 0 status is OK
	 * @retval -1 status is unknown
	 */
	int printEPOSstate();

	/*! \brief pretty-print EPOS Error Register */
	static void printErrorRegister(UNSIGNED8 reg);

	//! \brief Seconds per minute -- used in motion profile calculations,
	//! since EPOS velocity is in [rpm] and acceleration is in [rpm/s].
	static const unsigned SECONDS_PER_MINUTE;

	//! \brief States of the EPOS controller
	typedef enum _state
	{
		SHUTDOWN, SWITCH_ON, SWITCH_ON_AND_ENABLE, DISABLE_VOLTAGE,
		QUICKSTOP, DISABLE_OPERATION, ENABLE_OPERATION, FAULT_RESET
	} state_t;

	//! \brief Reset the device by issuing a shutdown command followed by power-on and halt
	void reset();

	/*! \brief change EPOS state */
	void changeEPOSstate(state_t state);

	/*! \brief read CAN Node ID */
	UNSIGNED8 readNodeID();

	/*! \brief ask EPOS for software version */
	UNSIGNED16 readSWversion();

	/*! \brief read manufactor device name string firmware */
	std::string readDeviceName();

	/*! \brief ask for RS232 timeout */
	UNSIGNED16 readRS232timeout();

	/*! \brief read digital input polarity mask */
	UNSIGNED16 readDInputPolarity();

	/*! \brief set home switch polarity */
	void setHomePolarity(int pol);

	/*! \brief read Statusword */
	UNSIGNED16 readStatusWord();

	/*! \brief pretty-print statusword to stdout
	 *
	 * \param statusword WORD variable holding the statusword
	 */
	static void printEPOSstatusword(canopen::WORD statusword);

	/*! \brief read EPOS control word */
	UNSIGNED16 readControlword();

	//! write EPOS control word
	void writeControlword(UNSIGNED16 val);

	/*! \brief pretty-print controlword */
	void printEPOScontrolword(canopen::WORD controlword);

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
	operational_mode_t readActualOperationMode();

	/*! \brief read demanded position */
	INTEGER32 readDemandPosition();

	/*! \brief read actual position */
	INTEGER32 readActualPosition();

	/*! \brief read position window */
	UNSIGNED32 readPositionWindow();

	/*! \brief write position window */
	void writePositionWindow(UNSIGNED32 value);

	/*! \brief read position window time */
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

	//! \brief read velocity normally attained at the end of the acceleration ramp during a profiled motion
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

	//! @}

	//! \ingroup libEPOS
	//! \defgroup epos_pid Regulator tuning
	//! @{

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

	//! @}

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

	/*! \brief read actual position */
	INTEGER32 readDemandVelocity();

	/*! \brief read actual position */
	INTEGER32 readActualVelocity();

	/*! \brief read actual current */
	INTEGER16 readActualCurrent();

	/*! \brief read target position */
	INTEGER32 readTargetPosition();

	/*! \brief read target position */
	void writeTargetPosition(INTEGER32 val);

	/*! \brief read Maximal Following Error */
	UNSIGNED32 readMaxFollowingError();

	/*! \brief write Maximal Following Error */
	void writeMaxFollowingError(UNSIGNED32 val);

	/*! \brief read Home Offset */
	INTEGER32 readHomeOffset();

	/*! \brief write Home Offset */
	void writeHomeOffset(INTEGER32 val);

	/*! \brief read Speed for Switch Search */
	UNSIGNED32 readSpeedForSwitchSearch();

	/*! \brief write Speed for Switch Search */
	void writeSpeedForSwitchSearch(UNSIGNED32 val);

	/*! \brief read Speed for Zero Search */
	UNSIGNED32 readSpeedForZeroSearch();

	/*! \brief write Speed for Zero Search */
	void writeSpeedForZeroSearch(UNSIGNED32 val);

	/*! \brief read Homing Acceleration */
	UNSIGNED32 readHomingAcceleration();

	/*! \brief write Homing Acceleration  */
	void writeHomingAcceleration(UNSIGNED32 val);

	/*! \brief read Current Threshold for Homing Mode */
	UNSIGNED16 readCurrentThresholdForHomingMode();

	/*! \brief write Current Threshold for Homing Mode  */
	void writeCurrentThresholdForHomingMode(UNSIGNED16 val);

	/*! \brief read Error register */
	UNSIGNED8 readErrorRegister();

	/*! \brief read number of Errors is Error History register */
	UNSIGNED8 readNumberOfErrors();

	/*! \brief read Error History at index */
	UNSIGNED32 readErrorHistory(unsigned int num);

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
	INTEGER32 readMinimalPositionLimit();

	/*! \brief write the Minimal Position Limit */
	void writeMinimalPositionLimit(INTEGER32 val);

    /*! \brief read the Maximal Position Limit */
	INTEGER32 readMaximalPositionLimit();

	/*! \brief write the Maximal Position Limit */
	void writeMaximalPositionLimit(INTEGER32 val);

	//! @}

	//! \ingroup libEPOS
	//! \defgroup epos_gear Gear Configuration
	//! @{

	//! \brief read Gear Ratio Numerator
	UNSIGNED32 readGearRatioNumerator();

	//! \brief write Gear Ratio Numerator
	void writeGearRatioNumerator(UNSIGNED32 val);

	//! \brief read Gear Ratio Denominator
	UNSIGNED16 readGearRatioDenominator();

	//! \brief write Gear Ratio Denominator
	void writeGearRatioDenominator(UNSIGNED16 val);

	//! \brief read Gear Maximal Speed
	UNSIGNED32 readGearMaximalSpeed();

	//! \brief write Gear Maximal Speed
	void writeGearMaximalSpeed(UNSIGNED32 val);

	//! @}

	//! \ingroup libEPOS
	//! \defgroup epos_ipm Interpolated Profile Motion
	//! @{

	/*! \brief Provides the actual free buffer size and is given in interpolation data records */
	UNSIGNED32 readActualBufferSize();

	/*! \brief clear a buffer and reenable access to it */
	void clearPvtBuffer();

	//! \brief write Interpolation Sub Mode Selection
	void writeInterpolationSubModeSelection(INTEGER16 val);

	//! \brief read Interpolation Sub Mode Selection
	INTEGER16 readInterpolationSubModeSelection();

	//! \brief write Interpolation Time Period Value
	void writeInterpolationTimePeriod(UNSIGNED8 val);

	//! \brief read Interpolation Time Period Value
	UNSIGNED8 readInterpolationTimePeriod();

	//! \brief write Interpolation Time Index
	void writeInterpolationTimeIndex(INTEGER8 val);

	//! \brief read Interpolation Time Period Index
	INTEGER8 readInterpolationTimeIndex();

	//! \brief write Interpolation data record
	void writeInterpolationDataRecord(INTEGER32 position, INTEGER32 velocity, UNSIGNED8 time);

	//! \brief read Interpolation buffer status
	UNSIGNED16 readInterpolationBufferStatus();

	//! \brief check Interpolation Buffer warning
	static bool checkInterpolationBufferWarning(UNSIGNED16 status);

	//! \brief check Interpolation Buffer underflow warning
	static bool checkInterpolationBufferUnderflowWarning(UNSIGNED16 status);

	//! \brief check Interpolation Buffer error
	static bool checkInterpolationBufferError(UNSIGNED16 status);

	//! \brief Print status message about the Interpolation Buffer
	static void printInterpolationBufferStatus(UNSIGNED16 status);

	//! \brief read Interpolation buffer underflow warning
	UNSIGNED16 readInterpolationBufferUnderflowWarning();

	//! write Interpolation buffer underflow warning
	void writeInterpolationBufferUnderflowWarning(UNSIGNED16 val);

	//! \brief read Interpolation buffer overflow warning
	UNSIGNED16 readInterpolationBufferOverflowWarning();

	//! \brief write Interpolation buffer overflow warning
	void writeInterpolationBufferOverflowWarning(UNSIGNED16 val);

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
	homing_method_t readHomingMethod();

	/*! \brief write Homing Method */
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

	//! @}

private:
	//! Cached for parameters values
	//! @note have to be at the bottom because some typedefs are defined above
	operational_mode_t OpMode;
	INTEGER16 PositionProfileType;
	UNSIGNED32 ProfileVelocity;
	UNSIGNED32 ProfileAcceleration;
	UNSIGNED32 ProfileDeceleration;
#endif
};

} /* namespace festo */
} /* namespace edp */
} /* namespace mrrocpp */

//! @}

#endif
