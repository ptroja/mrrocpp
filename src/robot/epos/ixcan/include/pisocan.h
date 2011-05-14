
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <libgen.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include "ixcan.h"

/* unmark following line to enable debug message */
//#define DBG

/* debug flags */
#ifdef DBG
#define dbg(fmt, args...) printf(fmt, ##args)
#else
#define dbg(fmt, args...)
#endif

#define PF_CAN 29

/* defined ERROR ID */
#define SOCKETCAN_NOERROR 0
#define SOCKETCAN_OPEN_ERROR 1
#define SOCKETCAN_BIND_ERROR 2
#define SOCKETCAN_CLOSE_ERROR 3
#define SOCKETCAN_SEND_FRAME_ERROR 4
#define SOCKETCAN_RECEIVE_FRAME_ERROR 5

/* define driver and dll version */
#define SOCKETCAN_DRIVER_VERSION "0.2.2" 
#define SOCKETCAN_LIBRARY_VERSION "0.1.0" 
#define CAN_MODULE_NAME
#define LINE_SIZE 128
#define MAX_BOARD_NUMBER 4
#define MAX_CARD_SUPPORT 4
#define MAX_CAN400_PORT 4
#define MAX_CAN200_PORT 2

/* CAN_ERR_FLAG, CAN_EFF_FLAG, CAN_SFF_MASK, CAN_EFF_MASK defined in linux/include/can.h */
#define MSG_STD         0               /**< standard message format */
#define MSG_RTR         CAN_RTR_FLAG    /**< RTR Message */
#define MSG_EXT         CAN_EFF_FLAG    /**< Extended message format */
#define CANID_MASK      CAN_EFF_MASK
//#define MSG_STD_MASK    CAN_SFF_MASK
////#define MSG_EXT_MASK    CAN_EFF_MASK

typedef unsigned short WORD;
typedef unsigned int DWORD;
typedef void (*HANDLE)(int);
typedef unsigned char BYTE;
typedef int BOOL;

typedef struct can_frame canmsg_t;

/* implement the function in pisocan.c */

WORD SocketCAN_Open(char *canport, int *skt);
WORD SocketCAN_Close(int skt);
char * SocketCAN_GetDriverVersion(void);
char * SocketCAN_GetLibraryVersion(void);
WORD SocketCAN_SetCANFilter(int skt, struct can_filter *rfilter, int filter_size);
WORD SocketCAN_SendMsg(int skt, struct can_frame *frame);  
WORD SocketCAN_SendMsgWithResend(int skt, struct can_frame *frame, DWORD resend_count);
WORD SocketCAN_ReceiveMsg(int skt, struct can_frame *frame);  
WORD SocketCAN_ReceiveMsgNoWait(int skt, struct can_frame *frame);

