#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include "pisocan.h"
//#include "ixcan.h"

/* defined ERROR ID */
#define CPM_NOERROR 0
#define CPM_OPEN_ERROR 1
#define CPM_CLOSE_ERROR 2
#define CPM_PORT_INIT_ERROR 3
#define CPM_MASTER_INIT_ERROR 4
//#define CPM_CONFIG_ERROR 4
//#define CPM_ENABLERXIRQ_ERROR 5
#define CPM_NODEID_EXIST 5
#define CPM_NODEID_NOEXIST 6
#define CPM_NODEID_OVERRANGE 7
#define CPM_SLAVESTATE_ERROR 8
#define CPM_COBID_EXIST 9
#define CPM_NODEADD_ERROR 10
#define CPM_NODEREMOVE_ERROR 11
#define CPM_SENDMSG_ERROR 12
#define CPM_TIMEOUT 13
#define CPM_ADDTHREAD_ERROR 14
#define CPM_RESUMETHREAD_ERROR 15
#define CPM_SUSPENDTHREAD_ERROR 16
#define CPM_READ_SEGMENT 17
#define CPM_WRITE_SEGMENT 18
#define CPM_READ_BLOCK 19
#define CPM_WRITE_BLOCK 20
#define CPM_DATASIZE_RANGE_ERROR 21
#define CPM_READDATA_ERROR 22
#define CPM_WRITEDATA_ERROR 23
#define CPM_ENDBLOCK 24
#define CPM_DATA_RESEND 25
#define CPM_EMCYFIFO_EMPTY 26
#define CPM_PDOINSTALL_ERROR 27
#define CPM_COBID_NOTRXPDO 28
#define CPM_COBID_NOTTXPDO 29
#define CPM_ALLPDO_USING 30
#define CPM_MAPPING_ENABLE_ERROR 31
#define CPM_SETPDO_ERROR 32
#define CPM_PDO_NOEXIST 33
#define CPM_PDOREMOVE_ERROR 34
#define CPM_NORESPONSE 35

/* define driver and dll version */

typedef struct cpmchannel
{
        BYTE BoardNo;
        BYTE Port;
} cpmchannel_t;

typedef struct cpmconfig
{
        DWORD Acr;
        DWORD Amr;
        DWORD Baudrate;
} cpmconfig_t;

int CPM_Open(BYTE BoardNo, BYTE Port);

WORD CPM_Close(BYTE BoardNo, BYTE Port);

char * CPM_GetDriverVersion(void);

char * CPM_GetLibraryVersion(void);

WORD CPM_InitPort(BYTE BoardNo, BYTE Port, cpmchannel_t *Handle);

WORD CPM_Config(cpmchannel_t *Handle, struct can_filter *rfilter, WORD filter_size);

WORD CPM_InitMaster(cpmchannel_t *Handle, struct can_filter *rfilter, WORD filter_size, WORD SDOTimeOut);

WORD CPM_ChangeSDOTimeOut(cpmchannel_t *Handle,WORD SDOTimeOut);

WORD CPM_ShutdownMaster(cpmchannel_t *Handle);

WORD CPM_AddNode(cpmchannel_t *Handle, BYTE NodeID);

WORD CPM_RemoveNode(cpmchannel_t *Handle, BYTE NodeID);

WORD CPM_SDOReadData(cpmchannel_t *Handle, BYTE NodeID, WORD Index, BYTE SubIndex, canmsg_t *RxData, DWORD *RSize, BYTE Block);

WORD CPM_SDOReadSegment(cpmchannel_t *Handle, BYTE NodeID, canmsg_t *RxData);

WORD CPM_SDOWriteData(cpmchannel_t *Handle, BYTE NodeID, WORD Index, BYTE SubIndex, DWORD DataSize, canmsg_t *RxData, BYTE *Data, BYTE Block);

WORD CPM_SDOWriteSegment(cpmchannel_t *Handle, BYTE NodeID, canmsg_t *RxData, BYTE *Data);

WORD CPM_SDOReadBlock(cpmchannel_t *Handle, BYTE NodeID, BYTE *RxData);

WORD CPM_SDOWriteBlock(cpmchannel_t *Handle,BYTE NodeID,BYTE *Ackseq, BYTE *Data);

WORD CPM_SDOAbortTransmission(cpmchannel_t *Handle, BYTE NodeID);

WORD CPM_NMTGetState(cpmchannel_t *Handle, BYTE NodeID, BYTE *State);

WORD CPM_NMTChangeState(cpmchannel_t *Handle, BYTE NodeID, BYTE State);

WORD CPM_NMTGuarding(cpmchannel_t *Handle, BYTE NodeID, WORD GuardTime,BYTE LifeTimeFactor,DWORD GuardCycle);

WORD CPM_ChaneSYNCID(cpmchannel_t *Handle, BYTE NodeID, WORD CobID);

WORD CPM_SendSYNC(cpmchannel_t *Handle, DWORD Cobid, DWORD CycleTimer);

WORD CPM_ChangeEMCYID(cpmchannel_t *Handle, BYTE NodeID, WORD CobID);

WORD CPM_ReadEMCY(cpmchannel_t *Handle,canmsg_t *RData);

WORD CPM_InstallPDO(cpmchannel_t *Handle,BYTE NodeID,BYTE RxTxType,DWORD CobId,BYTE TransmitType,WORD Inhibitime,WORD EventTimer,BYTE EnableChannel);

WORD CPM_MappingPDO(cpmchannel_t *Handle,BYTE NodeID,BYTE RxTxType,DWORD CobId,BYTE Channel,BYTE *MappingData);

WORD CPM_RemovePDO(cpmchannel_t *Handle, BYTE NodeID, DWORD CobId);

WORD CPM_WritePDO(cpmchannel_t *Handle, DWORD CobId, BYTE *Data, BYTE Offset, BYTE DataLen);

WORD CPM_RemotePDO(cpmchannel_t *Handle,DWORD CobId,canmsg_t *RData);

WORD CPM_ResponsePDO(cpmchannel_t *Handle, canmsg_t *RData);

WORD CPM_ResPDOCount(cpmchannel_t *Handle);

WORD CPM_WriteDO(cpmchannel_t *Handle, BYTE NodeID, BYTE Channel, BYTE Value);

WORD CPM_WriteAO(cpmchannel_t *Handle, BYTE NodeID, BYTE Channel, WORD Value);

WORD CPM_ReadDI(cpmchannel_t *Handle, BYTE NodeID, BYTE Channel, BYTE *Value);

WORD CPM_ReadAI(cpmchannel_t *Handle, BYTE NodeID, BYTE Channel,WORD *Value);

WORD CPM_ReadManufacturerName(cpmchannel_t *Handle, BYTE NodeID, BYTE Slave_dev[]);
