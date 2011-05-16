#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>
//#include "ixcan.h"
#include "pisocan.h"


/* defined ERROR ID */
#define DNM_NOERROR 0
#define DNM_OPEN_ERROR 1
#define DNM_CLOSE_ERROR 2
#define DNM_PORT_ALREADY_ONLINE 3
#define DNM_MACID_ERROR 4
//#define DNM_BAUDRATE_ERROR 5
#define DNM_PORT_INIT_ERROR 5
#define DNM_PORT_CONFIG_ERROR 6
#define DNM_ENABLERXIRQ_ERROR 7
#define DNM_SENDMSG_ERROR 8
#define DNM_ONLINE_ERROR 9
#define DNM_DUPLICATE_MACID_TIMEOUT 10
#define DNM_ADDTHREAD_ERROR 11
#define DNM_RESUMETHREAD_ERROR 12
#define DNM_MASTER_STATE_ERROR 13
#define DNM_OFFLINE_ERROR 14
#define DNM_CHANGE_MACID_ERROR 15
#define DNM_CHANGE_BAUDRATE_ERROR 16
#define DNM_DEVICE_IN_SCANLIST 17
#define DNM_DEVICE_NOT_IN_SCANLIST 18
#define DNM_EXPLICITMSG_NOT_CONFIG 19
#define DNM_CONFIG_EXPLICITMSG_ERROR 20
#define DNM_POLL_NOT_CONFIG 21
#define DNM_CONFIG_POLL_ERROR 22
#define DNM_WRITE_POLL_OUTPUT_DATALENGTH_ERROR 23
#define DNM_BITSTROBE_NOT_CONFIG 24
#define DNM_CONFIG_BITSTROBE_ERROR 25
#define DNM_COS_CYCLIC_NOT_CONFIG 26
#define DNM_CONFIG_COS_CYCLIC_ERROR 27
#define DNM_STROBE_REQUEST_THREAD_NOT_EXIST 28
#define DNM_SUSPEND_STROBE_REQUEST_THREAD_ERROR 29
#define DNM_THREAD_CANCEL_ERROR 30
#define DNM_SET_ATTRIBUTE_DATA_IS_EMPTY 31
#define DNM_DEVICE_IS_UNCONNECTED 32
#define DNM_DEVICE_IS_CONNECTING 33
#define DNM_SET_ATTRIBUTE_RESPONSE_TIMEOUT 34
#define DNM_SET_ATTRIBUTE_FRAGMENT_ACK_TIMEOUT 35
#define DNM_SET_ATTRIBUTE_RESPONSE_ERROR 36
#define DNM_SET_ATTRIBUTE_ACK_ERROR 37
#define DNM_WAIT_BUFFER_RESPONSE_TIMEOUT 38
#define DNM_GET_ATTRIBUTE_RESPONSE_TIMEOUT 39
#define DNM_GET_ATTRIBUTE_RESPONSE_ERROR 40
#define DNM_CREATE_EXPLICITMSG_THREAD_ERROR 41
#define DNM_DEVICE_ERROR 42
#define DNM_COS_CYCLIC_ALREADY_CONFIG 43
#define DNM_COS_CYCLIC_CONNECT_DUPLICATE 44
#define DNM_SET_POLL_INPUT_LEN_ERROR 45
#define DNM_SET_POLL_OUTPUT_LEN_ERROR 46
#define DNM_SET_BITSTROBE_INPUT_LEN_ERROR 47
#define DNM_WRITE_COS_OUTPUT_LEN_ERROR 48
#define DNM_WRITE_CYCLIC_OUTPUT_LEN_ERROR 49
#define DNM_SET_COS_INPUT_LEN_ERROR 50
#define DNM_SET_CYCLIC_INPUT_LEN_ERROR 51
#define DNM_SET_COS_OUTPUT_LEN_ERROR 52
#define DNM_SET_CYCLIC_OUTPUT_LEN_ERROR 53
#define DNM_GET_OWNERSHIP_ERROR 54
#define DNM_MASTER_NOT_GET_OWNERSHIP 55
#define DNM_BITSTROBE_NO_RESPONSE 56
#define DNM_BITSTROBE_RESPONSE_DATA_ERROR 57
#define DNM_POLL_NO_RESPONSE 58
#define DNM_POLL_RESPONSE_DATA_ERROR 59
#define DNM_POLL_FRAGMENT_ERROR 60
#define DNM_COS_CYCLIC_NO_RESPONSE 61
#define DNM_COS_CYCLIC_RESPONSE_DATA_ERROR 62
#define DNM_COS_CYCLIC_FRAGMENT_ERROR 63
#define DNM_NORESPONSE 64

/* define driver and dll version */


typedef struct dnmchannel
{
        BYTE BoardNo;
        BYTE Port;
	BYTE DesMACID;
} dnmchannel_t;

typedef struct dnmconfig
{
        DWORD Acr;
        DWORD Amr;
        DWORD Baudrate;
} dnmconfig_t;

char * DNM_GetDriverVersion(void);

char * DNM_GetLibraryVersion(void);

DWORD DNM_Open(BYTE BoardNo, BYTE Port);
DWORD DNM_Close(BYTE BoardNo, BYTE Port);
DWORD DNM_InitPort(BYTE BoardNo, BYTE Port);
DWORD DNM_Online(BYTE BoardNo, BYTE Port, DWORD Macid);
DWORD DNM_Offline(BYTE BoardNo, BYTE Port);
DWORD DNM_ChangeMACID(BYTE BoardNo, BYTE Port, DWORD Macid);
//DWORD DNM_ChangeBaudrate(BYTE BoardNo, BYTE Port, DWORD Baudrate);
DWORD DNM_AddDevice(BYTE BoardNo, BYTE Port, DWORD DesMacID);
DWORD DNM_RemoveDevice(BYTE BoardNo, BYTE Port, DWORD DesMacID);

DWORD DNM_ConfigExplicitMsg(BYTE BoardNo , BYTE Port , DWORD DesMacID , WORD watchdog_timeout_action);

DWORD DNM_SetAttribute(BYTE BoardNo , BYTE Port , DWORD DesMacID , WORD ClassID , BYTE InstanceID , BYTE AttributeID , BYTE *SetValue , DWORD Length);

DWORD DNM_GetAttribute(BYTE BoardNo, BYTE Port, DWORD DesMacID, WORD ClassID, BYTE InstanceID, BYTE AttributeID, BYTE *GetValue, DWORD *Length);


DWORD DNM_CheckExplicitMsgConnectionStatus(BYTE BoardNo , BYTE Port , DWORD DesMacID);

DWORD DNM_ConfigPoll(BYTE BoardNo , BYTE Port , DWORD DesMacID , WORD produced_connection_size,WORD consumed_connection_size,WORD expected_packet_rate,WORD watchdog_timeout_action);

DWORD DNM_ReadPollInputData(BYTE BoardNo, BYTE Port, DWORD DesMacID ,BYTE *DataBuf ,DWORD *DataLength);

DWORD DNM_WritePollOutputData(BYTE BoardNo, BYTE Port, DWORD DesMacID, BYTE *DataBuf, DWORD DataLength);

DWORD DNM_CheckPollConnectionStatus(BYTE BoardNo , BYTE Port , DWORD DesMacID);

DWORD DNM_ConfigBitStrobe(BYTE BoardNo, BYTE Port, DWORD DesMacID, WORD produced_connection_size, WORD expected_packet_rate, WORD watchdog_timeout_action);

DWORD DNM_ReadBitStrobe(BYTE BoardNo, BYTE Port, DWORD DesMacID, BYTE *DataBuf, DWORD *DataLength);

DWORD DNM_CheckBitStrobeConnectionStatus(BYTE BoardNo , BYTE Port , DWORD DesMacID);

DWORD DNM_ConfigCOS(BYTE BoardNo, BYTE Port, DWORD DesMacID, WORD produced_connection_size, WORD consumed_connection_size, BYTE isnonack, WORD expected_packet_rate, WORD watchdog_timeout_action);

DWORD DNM_ConfigCyclic(BYTE BoardNo, BYTE Port, DWORD DesMacID, WORD produced_connection_size, WORD consumed_connection_size, BYTE isnonack, WORD expected_packet_rate, WORD watchdog_timeout_action);

DWORD DNM_ReadCOSInputData(BYTE BoardNo, BYTE Port, DWORD DesMacID, BYTE *DataBuf, DWORD *DataLength);

DWORD DNM_WriteCOSOutputData(BYTE BoardNo, BYTE Port, DWORD DesMacID, BYTE *DataBuf, DWORD DataLength);

DWORD DNM_ReadCyclicInputData(BYTE BoardNo, BYTE Port, DWORD DesMacID, BYTE *DataBuf, DWORD *DataLength);

DWORD DNM_WriteCyclicOutputData(BYTE BoardNo, BYTE Port, DWORD DesMacID, BYTE *DataBuf, DWORD DataLength);

DWORD DNM_CheckCOSConnectionStatus(BYTE BoardNo, BYTE Port, DWORD DesMacID);

DWORD DNM_CheckCyclicConnectionStatus(BYTE BoardNo, BYTE Port, DWORD DesMacID);

DWORD DNM_StartDevice(BYTE BoardNo, BYTE Port, DWORD DesMacID);

DWORD DNM_StopDevice(BYTE BoardNo, BYTE Port, DWORD DesMacID);

DWORD DNM_StartCommunicate(BYTE BoardNo, BYTE Port);

DWORD DNM_StopCommunicate(BYTE BoardNo, BYTE Port);

DWORD DNM_GetOwnership(BYTE BoardNo, BYTE Port);

DWORD DNM_CheckFaultNode(BYTE BoardNo, BYTE Port);

DWORD DNM_GetAllFaultNode(BYTE BoardNo, BYTE Port, WORD *VendorID, DWORD *SerialNo, BYTE *NodeCount);

DWORD DNM_ChangeFaultMACID(BYTE BoardNo, BYTE Port, WORD VendorID, DWORD SerialNo, BYTE NewMacID);

DWORD DNM_GetAllDeviceMACID(BYTE BoardNo, BYTE Port, BYTE *AllMacID, BYTE *DeviceCount);

/*
DWORD DNM_GetExplicitMsgNoResponse(BYTE BoardNo, BYTE Port, BYTE DesMacID);

DWORD DNM_GetExplicitMsgErrorResponse(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetBitStrobeNoResponse(BYTE BoardNo, BYTE Port, BYTE DesMacID);

DWORD DNM_GetCOS_CyclicWithPollNoResponse(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetCOS_CyclicWithPollErrorResponse(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetCOS_CyclicWithPollFragmentError(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetCOS_CyclicWithPollSendError(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetCOS_CyclicWithoutPollSendError(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetCOS_CyclicWithoutPollACKNoResponse(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetPollSendError(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetPollNoResponse(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetPollErrorResponse(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_GetPollFragmentError(BYTE BoardNo,BYTE Port,BYTE DesMacID);
*/
DWORD DNM_GetDeviceStatus(BYTE BoardNo,BYTE Port,BYTE DesMacID);

DWORD DNM_SetPollFrequency(WORD Frequency);

DWORD DNM_SetBitStrobeFrequency(WORD Frequency);

