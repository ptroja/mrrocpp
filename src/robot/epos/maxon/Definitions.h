/*************************************************************************************************************************************
**                  maxon motor ag, CH-6072 Sachseln
**************************************************************************************************************************************
**
** FILE:            Definitions.h
**
** Summary:         Exported Functions for Linux 32-Bit DLL
**
** Date:            15.12.2010
** Dev. Platform:   Eclipse 3.6 (Helios)
** Target:          Linux (Ubuntu 10.10)
** Written by:      maxon motor ag, CH-6072 Sachseln
**
** Changes:         1.00    (15.12.10): Initial Version
*************************************************************************************************************************************/

#ifndef _H_LINUX_EPOSCMD_
#define _H_LINUX_EPOSCMD_

//Communication
    int CreateCommunication();
    int DeleteCommunication();

// INITIALISATION FUNCTIONS
    #define Initialisation_DllExport            extern "C"
    #define HelpFunctions_DllExport             extern "C"
// CONFIGURATION FUNCTIONS
    #define Configuration_DllExport             extern "C"
// OPERATION FUNCTIONS
    #define Status_DllExport                    extern "C"
    #define StateMachine_DllExport              extern "C"
    #define ErrorHandling_DllExport             extern "C"
    #define MotionInfo_DllExport                extern "C"
    #define ProfilePositionMode_DllExport       extern "C"
    #define ProfileVelocityMode_DllExport       extern "C"
    #define HomingMode_DllExport                extern "C"
    #define InterpolatedPositionMode_DllExport  extern "C"
    #define PositionMode_DllExport              extern "C"
    #define VelocityMode_DllExport              extern "C"
    #define CurrentMode_DllExport               extern "C"
    #define MasterEncoderMode_DllExport         extern "C"
    #define StepDirectionMode_DllExport         extern "C"
    #define InputsOutputs_DllExport             extern "C"
// LOW LAYER FUNCTIONS
    #define CanLayer_DllExport                  extern "C"

/*************************************************************************************************************************************
* INITIALISATION FUNCTIONS
*************************************************************************************************************************************/

//Communication
    Initialisation_DllExport void*  VCS_OpenDevice(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, unsigned long* pErrorCode);
    Initialisation_DllExport int  VCS_SetProtocolStackSettings(void* KeyHandle, unsigned long Baudrate, unsigned long Timeout, unsigned long* pErrorCode);
    Initialisation_DllExport int  VCS_GetProtocolStackSettings(void* KeyHandle, unsigned long* pBaudrate, unsigned long* pTimeout, unsigned long* pErrorCode);
    Initialisation_DllExport int  VCS_CloseDevice(void* KeyHandle, unsigned long* pErrorCode);
    Initialisation_DllExport int  VCS_CloseAllDevices(unsigned long* pErrorCode);

//Info
    HelpFunctions_DllExport int  VCS_GetVersion(void* KeyHandle, unsigned short NodeId, unsigned short* pHardwareVersion, unsigned short* pSoftwareVersion, unsigned short* pApplicationNumber, unsigned short* pApplicationVersion, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetErrorInfo(unsigned long ErrorCodeValue, char* pErrorInfo, unsigned short MaxStrSize);

//Advanced Functions
    HelpFunctions_DllExport int  VCS_GetDeviceNameSelection(int StartOfSelection, char* pDeviceNameSel, unsigned short MaxStrSize, int* pEndOfSelection, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetProtocolStackNameSelection(char* DeviceName, int StartOfSelection, char* pProtocolStackNameSel, unsigned short MaxStrSize, int* pEndOfSelection, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetInterfaceNameSelection(char* DeviceName, char* ProtocolStackName, int StartOfSelection, char* pInterfaceNameSel, unsigned short MaxStrSize, int* pEndOfSelection, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetPortNameSelection(char* DeviceName, char* ProtocolStackName, char* InterfaceName, int StartOfSelection, char* pPortSel, unsigned short MaxStrSize, int* pEndOfSelection, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetBaudrateSelection(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, int StartOfSelection, unsigned long* pBaudrateSel, int* pEndOfSelection, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetKeyHandle(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, void** pKeyHandle, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetDeviceName(void* KeyHandle, char* pDeviceName, unsigned short MaxStrSize, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetProtocolStackName(void* KeyHandle, char* pProtocolStackName, unsigned short MaxStrSize, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetInterfaceName(void* KeyHandle, char* pInterfaceName, unsigned short MaxStrSize, unsigned long* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetPortName(void* KeyHandle, char* pPortName, unsigned short MaxStrSize, unsigned long* pErrorCode);

/*************************************************************************************************************************************
* CONFIGURATION FUNCTIONS
*************************************************************************************************************************************/

//General
    Configuration_DllExport int  VCS_SetObject(void* KeyHandle, unsigned short NodeId, unsigned short ObjectIndex, unsigned char ObjectSubIndex, void* pData, unsigned long NbOfBytesToWrite, unsigned long* pNbOfBytesWritten, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetObject(void* KeyHandle, unsigned short NodeId, unsigned short ObjectIndex, unsigned char ObjectSubIndex, void* pData, unsigned long NbOfBytesToRead, unsigned long* pNbOfBytesRead, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_Restore(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_Store(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

//Advanced Functions
    //Motor
    Configuration_DllExport int  VCS_SetMotorType(void* KeyHandle, unsigned short NodeId, unsigned short MotorType, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_SetDcMotorParameter(void* KeyHandle, unsigned short NodeId, unsigned short NominalCurrent, unsigned short MaxOutputCurrent, unsigned short ThermalTimeConstant, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_SetEcMotorParameter(void* KeyHandle, unsigned short NodeId, unsigned short NominalCurrent, unsigned short MaxOutputCurrent, unsigned short ThermalTimeConstant, unsigned char NbOfPolePairs, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetMotorType(void* KeyHandle, unsigned short NodeId, unsigned short* pMotorType, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetDcMotorParameter(void* KeyHandle, unsigned short NodeId, unsigned short* pNominalCurrent, unsigned short* pMaxOutputCurrent, unsigned short* pThermalTimeConstant, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetEcMotorParameter(void* KeyHandle, unsigned short NodeId, unsigned short* pNominalCurrent, unsigned short* pMaxOutputCurrent, unsigned short* pThermalTimeConstant, unsigned char* pNbOfPolePairs, unsigned long* pErrorCode);

    //Sensor
    Configuration_DllExport int  VCS_SetSensorType(void* KeyHandle, unsigned short NodeId, unsigned short SensorType, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_SetIncEncoderParameter(void* KeyHandle, unsigned short NodeId, unsigned long EncoderResolution, int InvertedPolarity, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_SetHallSensorParameter(void* KeyHandle, unsigned short NodeId, int InvertedPolarity, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_SetSsiAbsEncoderParameter(void* KeyHandle, unsigned short NodeId, unsigned short DataRate, unsigned short NbOfMultiTurnDataBits, unsigned short NbOfSingleTurnDataBits, int InvertedPolarity, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetSensorType(void* KeyHandle, unsigned short NodeId, unsigned short* pSensorType, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetIncEncoderParameter(void* KeyHandle, unsigned short NodeId, unsigned long* pEncoderResolution, int* pInvertedPolarity, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetHallSensorParameter(void* KeyHandle, unsigned short NodeId, int* pInvertedPolarity, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetSsiAbsEncoderParameter(void* KeyHandle, unsigned short NodeId, unsigned short* pDataRate, unsigned short* pNbOfMultiTurnDataBits, unsigned short* pNbOfSingleTurnDataBits, int* pInvertedPolarity, unsigned long* pErrorCode);

    //Safety
    Configuration_DllExport int  VCS_SetMaxFollowingError(void* KeyHandle, unsigned short NodeId, unsigned long MaxFollowingError, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetMaxFollowingError(void* KeyHandle, unsigned short NodeId, unsigned long* pMaxFollowingError, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_SetMaxProfileVelocity(void* KeyHandle, unsigned short NodeId, unsigned long MaxProfileVelocity, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetMaxProfileVelocity(void* KeyHandle, unsigned short NodeId, unsigned long* pMaxProfileVelocity, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_SetMaxAcceleration(void* KeyHandle, unsigned short NodeId, unsigned long MaxAcceleration, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetMaxAcceleration(void* KeyHandle, unsigned short NodeId, unsigned long* pMaxAcceleration, unsigned long* pErrorCode);

    //Position Regulator
    Configuration_DllExport int  VCS_SetPositionRegulatorGain(void* KeyHandle, unsigned short NodeId, unsigned short P, unsigned short I, unsigned short D, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_SetPositionRegulatorFeedForward(void* KeyHandle, unsigned short NodeId, unsigned short VelocityFeedForward, unsigned short AccelerationFeedForward, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetPositionRegulatorGain(void* KeyHandle, unsigned short NodeId, unsigned short* pP, unsigned short* pI, unsigned short* pD, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetPositionRegulatorFeedForward(void* KeyHandle, unsigned short NodeId, unsigned short* pVelocityFeedForward, unsigned short* pAccelerationFeedForward, unsigned long* pErrorCode);

    //Velocity Regulator
    Configuration_DllExport int  VCS_SetVelocityRegulatorGain(void* KeyHandle, unsigned short NodeId, unsigned short P, unsigned short I, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetVelocityRegulatorGain(void* KeyHandle, unsigned short NodeId, unsigned short* pP, unsigned short* pI, unsigned long* pErrorCode);

    //Current Regulator
    Configuration_DllExport int  VCS_SetCurrentRegulatorGain(void* KeyHandle, unsigned short NodeId, unsigned short P, unsigned short I, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetCurrentRegulatorGain(void* KeyHandle, unsigned short NodeId, unsigned short* pP, unsigned short* pI, unsigned long* pErrorCode);

    //Inputs/Outputs
    Configuration_DllExport int  VCS_DigitalInputConfiguration(void* KeyHandle, unsigned short NodeId, unsigned short DigitalInputNb, unsigned short Configuration, int Mask, int Polarity, int ExecutionMask, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_DigitalOutputConfiguration(void* KeyHandle, unsigned short NodeId, unsigned short DigitalOutputNb, unsigned short Configuration, int State, int Mask, int Polarity, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_AnalogInputConfiguration(void* KeyHandle, unsigned short NodeId, unsigned short AnalogInputNb, unsigned short Configuration, int ExecutionMask, unsigned long* pErrorCode);

    //Units
    Configuration_DllExport int  VCS_SetVelocityUnits(void* KeyHandle, unsigned short NodeId, unsigned char VelDimension, char VelNotation, unsigned long* pErrorCode);
    Configuration_DllExport int  VCS_GetVelocityUnits(void* KeyHandle, unsigned short NodeId, unsigned char* pVelDimension, char* pVelNotation, unsigned long* pErrorCode);

/*************************************************************************************************************************************
* OPERATION FUNCTIONS
*************************************************************************************************************************************/

//OperationMode
    Status_DllExport int  VCS_SetOperationMode(void* KeyHandle, unsigned short NodeId, char OperationMode, unsigned long* pErrorCode);
    Status_DllExport int  VCS_GetOperationMode(void* KeyHandle, unsigned short NodeId, char* pOperationMode, unsigned long* pErrorCode);

//StateMachine
    StateMachine_DllExport int  VCS_ResetDevice(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_SetState(void* KeyHandle, unsigned short NodeId, unsigned short State, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_SetEnableState(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_SetDisableState(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_SetQuickStopState(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_ClearFault(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_GetState(void* KeyHandle, unsigned short NodeId, unsigned short* pState, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_GetEnableState(void* KeyHandle, unsigned short NodeId, int* pIsEnabled, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_GetDisableState(void* KeyHandle, unsigned short NodeId, int* pIsDisabled, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_GetQuickStopState(void* KeyHandle, unsigned short NodeId, int* pIsQuickStopped, unsigned long* pErrorCode);
    StateMachine_DllExport int  VCS_GetFaultState(void* KeyHandle, unsigned short NodeId, int* pIsInFault, unsigned long* pErrorCode);

//ErrorHandling
    ErrorHandling_DllExport int  VCS_GetNbOfDeviceError(void* KeyHandle, unsigned short NodeId, unsigned char *pNbDeviceError, unsigned long *pErrorCode);
    ErrorHandling_DllExport int  VCS_GetDeviceErrorCode(void* KeyHandle, unsigned short NodeId, unsigned char DeviceErrorNumber, unsigned long *pDeviceErrorCode, unsigned long *pErrorCode);

//Motion Info
    MotionInfo_DllExport int  VCS_GetMovementState(void* KeyHandle, unsigned short NodeId, int* pTargetReached, unsigned long* pErrorCode);
    MotionInfo_DllExport int  VCS_GetPositionIs(void* KeyHandle, unsigned short NodeId, long* pPositionIs, unsigned long* pErrorCode);
    MotionInfo_DllExport int  VCS_GetVelocityIs(void* KeyHandle, unsigned short NodeId, long* pVelocityIs, unsigned long* pErrorCode);
    MotionInfo_DllExport int  VCS_GetCurrentIs(void* KeyHandle, unsigned short NodeId, short* pCurrentIs, unsigned long* pErrorCode);
    MotionInfo_DllExport int  VCS_WaitForTargetReached(void* KeyHandle, unsigned short NodeId, unsigned long Timeout, unsigned long* pErrorCode);

//Profile Position Mode
    ProfilePositionMode_DllExport int  VCS_ActivateProfilePositionMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_SetPositionProfile(void* KeyHandle, unsigned short NodeId, unsigned long ProfileVelocity, unsigned long ProfileAcceleration, unsigned long ProfileDeceleration, unsigned long* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_GetPositionProfile(void* KeyHandle, unsigned short NodeId, unsigned long* pProfileVelocity, unsigned long* pProfileAcceleration, unsigned long* pProfileDeceleration, unsigned long* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_MoveToPosition(void* KeyHandle, unsigned short NodeId, long TargetPosition, int Absolute, int Immediately, unsigned long* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_GetTargetPosition(void* KeyHandle, unsigned short NodeId, long* pTargetPosition, unsigned long* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_HaltPositionMovement(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

    //Advanced Functions
    ProfilePositionMode_DllExport int  VCS_EnablePositionWindow(void* KeyHandle, unsigned short NodeId, unsigned long PositionWindow, unsigned short PositionWindowTime, unsigned long* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_DisablePositionWindow(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

//Profile Velocity Mode
    ProfileVelocityMode_DllExport int  VCS_ActivateProfileVelocityMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_SetVelocityProfile(void* KeyHandle, unsigned short NodeId, unsigned long ProfileAcceleration, unsigned long ProfileDeceleration, unsigned long* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_GetVelocityProfile(void* KeyHandle, unsigned short NodeId, unsigned long* pProfileAcceleration, unsigned long* pProfileDeceleration, unsigned long* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_MoveWithVelocity(void* KeyHandle, unsigned short NodeId, long TargetVelocity, unsigned long* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_GetTargetVelocity(void* KeyHandle, unsigned short NodeId, long* pTargetVelocity, unsigned long* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_HaltVelocityMovement(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

    //Advanced Functions
    ProfileVelocityMode_DllExport int  VCS_EnableVelocityWindow(void* KeyHandle, unsigned short NodeId, unsigned long VelocityWindow, unsigned short VelocityWindowTime, unsigned long* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_DisableVelocityWindow(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

//Homing Mode
    HomingMode_DllExport int  VCS_ActivateHomingMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    HomingMode_DllExport int  VCS_SetHomingParameter(void* KeyHandle, unsigned short NodeId, unsigned long HomingAcceleration, unsigned long SpeedSwitch, unsigned long SpeedIndex, long HomeOffset, unsigned short CurrentTreshold, long HomePosition, unsigned long* pErrorCode);
    HomingMode_DllExport int  VCS_GetHomingParameter(void* KeyHandle, unsigned short NodeId, unsigned long* pHomingAcceleration, unsigned long* pSpeedSwitch, unsigned long* pSpeedIndex, long* pHomeOffset, unsigned short* pCurrentTreshold, long* pHomePosition, unsigned long* pErrorCode);
    HomingMode_DllExport int  VCS_FindHome(void* KeyHandle, unsigned short NodeId, char HomingMethod, unsigned long* pErrorCode);
    HomingMode_DllExport int  VCS_StopHoming(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    HomingMode_DllExport int  VCS_DefinePosition(void* KeyHandle, unsigned short NodeId, long HomePosition, unsigned long* pErrorCode);

//Interpolated Position Mode
    InterpolatedPositionMode_DllExport int  VCS_ActivateInterpolatedPositionMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_SetIpmBufferParameter(void* KeyHandle, unsigned short NodeId, unsigned short UnderflowWarningLimit, unsigned short OverflowWarningLimit, unsigned long* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_GetIpmBufferParameter(void* KeyHandle, unsigned short NodeId, unsigned short* pUnderflowWarningLimit, unsigned short* pOverflowWarningLimit, unsigned long* pMaxBufferSize, unsigned long* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_ClearIpmBuffer(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_GetFreeIpmBufferSize(void* KeyHandle, unsigned short NodeId, unsigned long* pBufferSize, unsigned long* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_AddPvtValueToIpmBuffer(void* KeyHandle, unsigned short NodeId, long Position, long Velocity, unsigned char Time, unsigned long* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_StartIpmTrajectory(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_StopIpmTrajectory(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_GetIpmStatus(void* KeyHandle, unsigned short NodeId, int* pTrajectoryRunning, int* pIsUnderflowWarning, int* pIsOverflowWarning, int* pIsVelocityWarning, int* pIsAccelerationWarning, int* pIsUnderflowError, int* pIsOverflowError, int* pIsVelocityError, int* pIsAccelerationError, unsigned long* pErrorCode);

//Position Mode
    PositionMode_DllExport int  VCS_ActivatePositionMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    PositionMode_DllExport int  VCS_SetPositionMust(void* KeyHandle, unsigned short NodeId, long PositionMust, unsigned long* pErrorCode);
    PositionMode_DllExport int  VCS_GetPositionMust(void* KeyHandle, unsigned short NodeId, long* pPositionMust, unsigned long* pErrorCode);

    //Advanced Functions
    PositionMode_DllExport int  VCS_ActivateAnalogPositionSetpoint(void* KeyHandle, unsigned short NodeId, unsigned short AnalogInputNumber, float Scaling, long Offset, unsigned long* pErrorCode);
    PositionMode_DllExport int  VCS_DeactivateAnalogPositionSetpoint(void* KeyHandle, unsigned short NodeId, unsigned short AnalogInputNumber, unsigned long* pErrorCode);
    PositionMode_DllExport int  VCS_EnableAnalogPositionSetpoint(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    PositionMode_DllExport int  VCS_DisableAnalogPositionSetpoint(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

//Velocity Mode
    VelocityMode_DllExport int  VCS_ActivateVelocityMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    VelocityMode_DllExport int  VCS_SetVelocityMust(void* KeyHandle, unsigned short NodeId, long VelocityMust, unsigned long* pErrorCode);
    VelocityMode_DllExport int  VCS_GetVelocityMust(void* KeyHandle, unsigned short NodeId, long* pVelocityMust, unsigned long* pErrorCode);

    //Advanced Functions
    VelocityMode_DllExport int  VCS_ActivateAnalogVelocitySetpoint(void* KeyHandle, unsigned short NodeId, unsigned short AnalogInputNumber, float Scaling, long Offset, unsigned long* pErrorCode);
    VelocityMode_DllExport int  VCS_DeactivateAnalogVelocitySetpoint(void* KeyHandle, unsigned short NodeId, unsigned short AnalogInputNumber, unsigned long* pErrorCode);
    VelocityMode_DllExport int  VCS_EnableAnalogVelocitySetpoint(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    VelocityMode_DllExport int  VCS_DisableAnalogVelocitySetpoint(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

//Current Mode
    CurrentMode_DllExport int  VCS_ActivateCurrentMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    CurrentMode_DllExport int  VCS_SetCurrentMust(void* KeyHandle, unsigned short NodeId, short CurrentMust, unsigned long* pErrorCode);
    CurrentMode_DllExport int  VCS_GetCurrentMust(void* KeyHandle, unsigned short NodeId, short* pCurrentMust, unsigned long* pErrorCode);

    //Advanced Functions
    CurrentMode_DllExport int  VCS_ActivateAnalogCurrentSetpoint(void* KeyHandle, unsigned short NodeId, unsigned short AnalogInputNumber, float Scaling, short Offset, unsigned long* pErrorCode);
    CurrentMode_DllExport int  VCS_DeactivateAnalogCurrentSetpoint(void* KeyHandle, unsigned short NodeId, unsigned short AnalogInputNumber, unsigned long* pErrorCode);
    CurrentMode_DllExport int  VCS_EnableAnalogCurrentSetpoint(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    CurrentMode_DllExport int  VCS_DisableAnalogCurrentSetpoint(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

//MasterEncoder Mode
    MasterEncoderMode_DllExport int  VCS_ActivateMasterEncoderMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    MasterEncoderMode_DllExport int  VCS_SetMasterEncoderParameter(void* KeyHandle, unsigned short NodeId, unsigned short ScalingNumerator, unsigned short ScalingDenominator, unsigned char Polarity, unsigned long MaxVelocity, unsigned long MaxAcceleration, unsigned long* pErrorCode);
    MasterEncoderMode_DllExport int  VCS_GetMasterEncoderParameter(void* KeyHandle, unsigned short NodeId, unsigned short* pScalingNumerator, unsigned short* pScalingDenominator, unsigned char* pPolarity, unsigned long* pMaxVelocity, unsigned long* pMaxAcceleration, unsigned long* pErrorCode);

//StepDirection Mode
    StepDirectionMode_DllExport int  VCS_ActivateStepDirectionMode(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    StepDirectionMode_DllExport int  VCS_SetStepDirectionParameter(void* KeyHandle, unsigned short NodeId, unsigned short ScalingNumerator, unsigned short ScalingDenominator, unsigned char Polarity, unsigned long MaxVelocity, unsigned long MaxAcceleration, unsigned long* pErrorCode);
    StepDirectionMode_DllExport int  VCS_GetStepDirectionParameter(void* KeyHandle, unsigned short NodeId, unsigned short* pScalingNumerator, unsigned short* pScalingDenominator, unsigned char* pPolarity, unsigned long* pMaxVelocity, unsigned long* pMaxAcceleration, unsigned long* pErrorCode);

//Inputs Outputs
    //General
    InputsOutputs_DllExport int  VCS_GetAllDigitalInputs(void* KeyHandle, unsigned short NodeId, unsigned short* pInputs, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_GetAllDigitalOutputs(void* KeyHandle, unsigned short NodeId, unsigned short* pOutputs, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_SetAllDigitalOutputs(void* KeyHandle, unsigned short NodeId, unsigned short Outputs, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_GetAnalogInput(void* KeyHandle, unsigned short NodeId, unsigned short InputNumber, unsigned short* pAnalogValue, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_SetAnalogOutput(void* KeyHandle, unsigned short NodeId, unsigned short OutputNumber, unsigned short AnalogValue, unsigned long* pErrorCode);

    //Position Compare
    InputsOutputs_DllExport int  VCS_SetPositionCompareParameter(void* KeyHandle, unsigned short NodeId, unsigned char OperationalMode, unsigned char IntervalMode, unsigned char DirectionDependency, unsigned short IntervalWidth, unsigned short IntervalRepetitions, unsigned short PulseWidth, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_GetPositionCompareParameter(void* KeyHandle, unsigned short NodeId, unsigned char* pOperationalMode, unsigned char* pIntervalMode, unsigned char* pDirectionDependency, unsigned short* pIntervalWidth, unsigned short* pIntervalRepetitions, unsigned short* pPulseWidth, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_ActivatePositionCompare(void* KeyHandle, unsigned short NodeId, unsigned short DigitalOutputNumber, int Polarity, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_DeactivatePositionCompare(void* KeyHandle, unsigned short NodeId, unsigned short DigitalOutputNumber, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_EnablePositionCompare(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_DisablePositionCompare(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_SetPositionCompareReferencePosition(void* KeyHandle, unsigned short NodeId, long ReferencePosition, unsigned long* pErrorCode);

    //Position Marker
    InputsOutputs_DllExport int  VCS_SetPositionMarkerParameter(void* KeyHandle, unsigned short NodeId, unsigned char PositionMarkerEdgeType, unsigned char PositionMarkerMode, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_GetPositionMarkerParameter(void* KeyHandle, unsigned short NodeId, unsigned char* pPositionMarkerEdgeType, unsigned char* pPositionMarkerMode, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_ActivatePositionMarker(void* KeyHandle, unsigned short NodeId, unsigned short DigitalInputNumber, int Polarity, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_DeactivatePositionMarker(void* KeyHandle, unsigned short NodeId, unsigned short DigitalInputNumber, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_ReadPositionMarkerCounter(void* KeyHandle, unsigned short NodeId, unsigned short* pCount, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_ReadPositionMarkerCapturedPosition(void* KeyHandle, unsigned short NodeId, unsigned short CounterIndex, long* pCapturedPosition, unsigned long* pErrorCode);
    InputsOutputs_DllExport int  VCS_ResetPositionMarkerCounter(void* KeyHandle, unsigned short NodeId, unsigned long* pErrorCode);

/*************************************************************************************************************************************
* LOW LAYER FUNCTIONS
*************************************************************************************************************************************/

//CanLayer Functions
    CanLayer_DllExport int  VCS_SendCANFrame(void* KeyHandle, unsigned short CobID, unsigned short Length, void* pData, unsigned long* pErrorCode);
    CanLayer_DllExport int  VCS_ReadCANFrame(void* KeyHandle, unsigned short CobID, unsigned short Length, void* pData, unsigned long Timeout, unsigned long* pErrorCode);
    CanLayer_DllExport int  VCS_RequestCANFrame(void* KeyHandle, unsigned short CobID, unsigned short Length, void* pData, unsigned long* pErrorCode);
    CanLayer_DllExport int  VCS_SendNMTService(void* KeyHandle, unsigned short NodeId, unsigned short CommandSpecifier, unsigned long* pErrorCode);

/*************************************************************************************************************************************
* TYPE DEFINITIONS
*************************************************************************************************************************************/
//Communication
	//Dialog Mode
	const int DM_PROGRESS_DLG					= 0;
	const int DM_PROGRESS_AND_CONFIRM_DLG		= 1;
	const int DM_CONFIRM_DLG					= 2;
	const int DM_NO_DLG							= 3;

//Configuration
    //MotorType
    const unsigned short MT_DC_MOTOR                      = 1;
    const unsigned short MT_EC_SINUS_COMMUTATED_MOTOR     = 10;
    const unsigned short MT_EC_BLOCK_COMMUTATED_MOTOR     = 11;

    //SensorType
    const unsigned short ST_UNKNOWN                       = 0;
    const unsigned short ST_INC_ENCODER_3CHANNEL          = 1;
    const unsigned short ST_INC_ENCODER_2CHANNEL          = 2;
    const unsigned short ST_HALL_SENSORS                  = 3;
    const unsigned short ST_SSI_ABS_ENCODER_BINARY        = 4;
    const unsigned short ST_SSI_ABS_ENCODER_GREY          = 5;

//In- and outputs
    //Digital input configuration
    const unsigned short DIC_NEGATIVE_LIMIT_SWITCH        = 0;
    const unsigned short DIC_POSITIVE_LIMIT_SWITCH        = 1;
    const unsigned short DIC_HOME_SWITCH                  = 2;
    const unsigned short DIC_POSITION_MARKER              = 3;
    const unsigned short DIC_DRIVE_ENABLE                 = 4;
    const unsigned short DIC_QUICK_STOP                   = 5;
    const unsigned short DIC_GENERAL_PURPOSE_J            = 6;
    const unsigned short DIC_GENERAL_PURPOSE_I            = 7;
    const unsigned short DIC_GENERAL_PURPOSE_H            = 8;
    const unsigned short DIC_GENERAL_PURPOSE_G            = 9;
    const unsigned short DIC_GENERAL_PURPOSE_F            = 10;
    const unsigned short DIC_GENERAL_PURPOSE_E            = 11;
    const unsigned short DIC_GENERAL_PURPOSE_D            = 12;
    const unsigned short DIC_GENERAL_PURPOSE_C            = 13;
    const unsigned short DIC_GENERAL_PURPOSE_B            = 14;
    const unsigned short DIC_GENERAL_PURPOSE_A            = 15;

    //Digital output configuration
    const unsigned short DOC_READY_FAULT                  = 0;
    const unsigned short DOC_POSITION_COMPARE             = 1;
    const unsigned short DOC_GENERAL_PURPOSE_H            = 8;
    const unsigned short DOC_GENERAL_PURPOSE_G            = 9;
    const unsigned short DOC_GENERAL_PURPOSE_F            = 10;
    const unsigned short DOC_GENERAL_PURPOSE_E            = 11;
    const unsigned short DOC_GENERAL_PURPOSE_D            = 12;
    const unsigned short DOC_GENERAL_PURPOSE_C            = 13;
    const unsigned short DOC_GENERAL_PURPOSE_B            = 14;
    const unsigned short DOC_GENERAL_PURPOSE_A            = 15;

    //Analog input configuration
    const unsigned short AIC_ANALOG_CURRENT_SETPOINT      = 0;
    const unsigned short AIC_ANALOG_VELOCITY_SETPOINT     = 1;
    const unsigned short AIC_ANALOG_POSITION_SETPOINT     = 2;

//Units
    //VelocityDimension
    const unsigned char VD_RPM                               = 0xA4;

    //VelocityNotation
    const char VN_STANDARD                          = 0;
    const char VN_DECI                              = -1;
    const char VN_CENTI                             = -2;
    const char VN_MILLI                             = -3;

//Operational mode 
    const char OMD_PROFILE_POSITION_MODE          = 1;
    const char OMD_PROFILE_VELOCITY_MODE          = 3;
    const char OMD_HOMING_MODE                    = 6;
    const char OMD_INTERPOLATED_POSITION_MODE     = 7;
    const char OMD_POSITION_MODE                  = -1;
    const char OMD_VELOCITY_MODE                  = -2;
    const char OMD_CURRENT_MODE                   = -3;
    const char OMD_MASTER_ENCODER_MODE            = -5;
    const char OMD_STEP_DIRECTION_MODE            = -6;

//States
    const unsigned short ST_DISABLED                         = 0;
    const unsigned short ST_ENABLED                          = 1;
    const unsigned short ST_QUICKSTOP                        = 2;
    const unsigned short ST_FAULT                            = 3;

//Homing mode
    //Homing method
    const char HM_ACTUAL_POSITION                               = 35;
    const char HM_NEGATIVE_LIMIT_SWITCH                         = 17;
    const char HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX               = 1;
    const char HM_POSITIVE_LIMIT_SWITCH                         = 18;
    const char HM_POSITIVE_LIMIT_SWITCH_AND_INDEX               = 2;
    const char HM_HOME_SWITCH_POSITIVE_SPEED                    = 23;
    const char HM_HOME_SWITCH_POSITIVE_SPEED_AND_INDEX          = 7;
    const char HM_HOME_SWITCH_NEGATIVE_SPEED                    = 27;
    const char HM_HOME_SWITCH_NEGATIVE_SPEED_AND_INDEX          = 11;
    const char HM_CURRENT_THRESHOLD_POSITIVE_SPEED              = -3;
    const char HM_CURRENT_THRESHOLD_POSITIVE_SPEED_AND_INDEX    = -1;
    const char HM_CURRENT_THRESHOLD_NEGATIVE_SPEED              = -4;
    const char HM_CURRENT_THRESHOLD_NEGATIVE_SPEED_AND_INDEX    = -2;
    const char HM_INDEX_POSITIVE_SPEED                          = 34;
    const char HM_INDEX_NEGATIVE_SPEED                          = 33;

//Input Output PositionMarker
    //PositionMarkerEdgeType
    const unsigned char PET_BOTH_EDGES                   = 0;
    const unsigned char PET_RISING_EDGE                  = 1;
    const unsigned char PET_FALLING_EDGE                 = 2;

    //PositionMarkerMode
    const unsigned char PM_CONTINUOUS                    = 0;
    const unsigned char PM_SINGLE                        = 1;
    const unsigned char PM_MULTIPLE                      = 2;

//Input Output PositionCompare
    //PositionCompareOperationalMode
    const unsigned char PCO_SINGLE_POSITION_MODE         = 0;
    const unsigned char PCO_POSITION_SEQUENCE_MODE       = 1;

    //PositionCompareIntervalMode
    const unsigned char PCI_NEGATIVE_DIR_TO_REFPOS       = 0;
    const unsigned char PCI_POSITIVE_DIR_TO_REFPOS       = 1;
    const unsigned char PCI_BOTH_DIR_TO_REFPOS           = 2;

    //PositionCompareDirectionDependency
    const unsigned char PCD_MOTOR_DIRECTION_NEGATIVE     = 0;
    const unsigned char PCD_MOTOR_DIRECTION_POSITIVE     = 1;
    const unsigned char PCD_MOTOR_DIRECTION_BOTH         = 2;

//Data recorder
    //Trigger type
    const unsigned short DR_MOVEMENT_START_TRIGGER        = 1;    //bit 1
    const unsigned short DR_ERROR_TRIGGER                 = 2;    //bit 2
    const unsigned short DR_DIGITAL_INPUT_TRIGGER         = 4;    //bit 3
    const unsigned short DR_MOVEMENT_END_TRIGGER          = 8;    //bit 4

//CanLayer Functions
    const unsigned short NCS_START_REMOTE_NODE            = 1;
    const unsigned short NCS_STOP_REMOTE_NODE             = 2;
    const unsigned short NCS_ENTER_PRE_OPERATIONAL        = 128;
    const unsigned short NCS_RESET_NODE                   = 129;
    const unsigned short NCS_RESET_COMMUNICATION          = 130;

#endif //_H_LINUX_EPOSCMD_


