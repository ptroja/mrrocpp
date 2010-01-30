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

/* added oct06 for openTCPEPOS() */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>



//#include "eposlib.h" -> moved to epos.c

/* all EPOS data exchange is based on 16bit words, but other types are
   also used...*/
typedef uint32_t DWORD ; ///< \brief 32bit type for EPOS data exchange
typedef uint16_t WORD ; ///< \brief 16bit type for EPOS data exchange
#ifndef CPP
typedef char BYTE ; ///< \brief 8bit type for EPOS data exchange
#endif

typedef struct epos_s {
  char *dev;
  struct termios options;
  int sp;           ///< serial port file descriptor
  DWORD E_error;    ///< EPOS global error status
  int ep;           ///< (-1) EPOS file descriptor
  char gMarker;     ///< (0) for internal handling
} epos_t;

/* EPOS will reset communication after 500ms of inactivity */

/*! \brief try NTRY times to read one byte from EPOS, the give up */
#define NTRY      5
/*! \brief sleep TRYSLEEP usec between read() from EPOS, if no data available */
#define TRYSLEEP  (unsigned int)1e5 




/* all high-level functions return <0 in case of error */

/*! create new EPOS object */
epos_t *newEPOS(const char *device);
/*! delete EPOS object */
int deleteEPOS(epos_t *epos);


/*! open the connection to EPOS */
int openEPOS(epos_t *epos, tcflag_t br);
/*! close the connection to EPOS */
int closeEPOS(epos_t *epos);
/*! check if the connection to EPOS is alive */
int checkEPOS(epos_t *epos);


/*! \brief check global variable E_error for EPOS error code */
int checkEPOSerror(epos_t *epos);

/*! \brief check EPOS status, return state according to firmware spec 8.1.1 */
int checkEPOSstate(epos_t *epos);
/*! \brief pretty-print EPOS state */
int printEPOSstate(epos_t *epos);
/*! \brief change EPOS state   ==> firmware spec 8.1.3 */
int changeEPOSstate(epos_t *epos, int state);



/*! \brief example from EPOS com. guide: ask EPOS for software version 

firmware spec 14.1.33
** returns software version as HEX **

*/
int readSWversion(epos_t *epos);


/*! \brief ask for device name,  
   device name is placed in 'name' (string must be big enough, NO CHECKING!!)
*/
int readDeviceName(epos_t *epos, char *name);


/*! \brief ask for RS232 timeout; firmware spec 14.1.35 */
int readRS232timeout(epos_t *epos);

/*! \brief read digital input polarity mask */
int readDInputPolarity(epos_t *epos, WORD* w);

/*! \brief set home switch polarity -- firmware spec 14.1.47 */
int setHomePolarity(epos_t *epos, int pol);



/*! \brief read Statusword; 14.1.58 */
int readStatusword(epos_t *epos, WORD *eposStatus);
/*! \brief pretty-print Statusword */
int printEPOSstatusword(WORD statusword);



/*! \brief read EPOS control word (firmware spec 14.1.57) */
int readControlword(epos_t *epos, WORD *w);
/*! \brief pretty-print Controlword */
int printEPOScontrolword(WORD controlword);


/*! \brief set EPOS mode of operation -- 14.1.59 */
int setOpMode(epos_t *epos, int OpMode);

/*! \brief read and returns  EPOS mode of operation -- 14.1.60 
here, RETURN(0) MEANS ERROR! 
'-1' is a valid OpMode, but 0 is not!
*/
int readOpMode(epos_t *epos);


/*! \brief read actual position; 14.1.61 */
int readDemandPosition(epos_t *epos, long *val);
/*! \brief read actual position; 14.1.62 */
int readActualPosition(epos_t *epos, long *val);

/*! \brief read position window; 14.1.64 */
int readPositionWindow(epos_t *epos, unsigned long int *value);
/*! \brief write position window; 14.1.64 */
int writePositionWindow(epos_t *epos, unsigned long int value);



/*! < by Martí Morta (mmorta@iri.upc.edu) > */


/*! \brief read position window time; 14.1.67 */
int readPositionWindowTime(epos_t *epos, unsigned int *time);
/*! \brief read position window time; 14.1.67 */
int writePositionWindowTime(epos_t *epos, unsigned int val);

int readPositionSoftwareLimits(epos_t *epos, long *val,long *val2);
int writePositionSoftwareLimits(epos_t *epos, long val,long val2);

int writePositionProfileVelocity(epos_t *epos, unsigned long int vel);
int writePositionProfileAcceleration(epos_t *epos, unsigned long int acc);
int writePositionProfileDeceleration(epos_t *epos, unsigned long int dec);
int writePositionProfileQuickStopDeceleration(epos_t *epos, unsigned long int qsdec);
int writePositionProfileMaxVelocity(epos_t *epos, unsigned long int maxvel);
int writePositionProfileType(epos_t *epos, int type);

int readPositionProfileVelocity(epos_t *epos, unsigned long int *val);
int readPositionProfileAcceleration(epos_t *epos, unsigned long int *val);
int readPositionProfileDeceleration(epos_t *epos, unsigned long int *val);
int readPositionProfileQuickStopDeceleration(epos_t *epos, unsigned long int *val);
int readPositionProfileMaxVelocity(epos_t *epos, unsigned long int *val);
int readPositionProfileType(epos_t *epos, int *val);

int readVelocityNotationIndex(epos_t *epos, int *index);
int writeVelocityNotationIndex(epos_t *epos, unsigned int val);

int readSensorPulses(epos_t *epos, unsigned int *pulse);
int readSensorType(epos_t *epos, unsigned int *type);
int readSensorPolarity(epos_t *epos, unsigned int *polaritat);
int writeSensorType(epos_t *epos, unsigned int val);
int writeSensorPulses(epos_t *epos, unsigned int val);
int writeSensorPolarity(epos_t *epos, unsigned int val);

int readRS232Baudrate(epos_t *epos, unsigned int *type);
int writeRS232Baudrate(epos_t *epos, unsigned int val);

int readP(epos_t *epos, int *val);
int readI(epos_t *epos, int *val);
int readD(epos_t *epos, int *val);
int readVFF(epos_t *epos, unsigned int *val);
int readAFF(epos_t *epos, unsigned int *val);
int writeP(epos_t *epos, int val);
int writeI(epos_t *epos, int val);
int writeD(epos_t *epos, int val);
int writeVFF(epos_t *epos, unsigned int val);
int writeAFF(epos_t *epos, unsigned int val);

int readPcurrent(epos_t *epos, int *val);
int readIcurrent(epos_t *epos, int *val);
int writePcurrent(epos_t *epos, int val);
int writeIcurrent(epos_t *epos, int val);
int saveParameters(epos_t *epos);

int readHomePosition(epos_t *epos, long int *val);
int writeHomePosition(epos_t *epos, long int val);

int readMotorContinousCurrentLimit(epos_t *epos, unsigned int *cur);
int writeMotorContinousCurrentLimit(epos_t *epos, unsigned int cur);
int readMotorOutputCurrentLimit(epos_t *epos, unsigned int *cur);
int writeMotorOutputCurrentLimit(epos_t *epos, unsigned int cur);
int readMotorPolePair(epos_t *epos, unsigned int *cur);
int writeMotorPolePair(epos_t *epos, unsigned int cur);
int readMotorMaxSpeedCurrent(epos_t *epos, unsigned int *cur);
int writeMotorMaxSpeedCurrent(epos_t *epos, unsigned int cur);
int readMotorThermalConstant(epos_t *epos, unsigned int *cur);
int writeMotorThermalConstant(epos_t *epos, unsigned int cur);

/*! < by Martí Morta /> */


/*! \brief read actual position; 14.1.67 */
int readDemandVelocity(epos_t *epos, long *val);
/*! \brief read actual position; 14.1.68 */
int readActualVelocity(epos_t *epos, long *val);

/*! \brief read actual current; 14.1.69 */
int readActualCurrent(epos_t *epos, short *val);

/*! \brief read target position; 14.1.70 */
int readTargetPosition(epos_t *epos, long *val);



/*! \brief does a homing move. Give homing mode (see firmware 9.3) and start
   position */
int doHoming(epos_t *epos, int method, long int start);


/*! \brief set OpMode to ProfilePosition and make relative movement */
int moveRelative(epos_t *epos, long int steps);
/*! \brief set OpMode to ProfilePosition and make absolute movement */
int moveAbsolute(epos_t *epos, long int steps);

/*! \brief reads position, velocity and current and displays them in an
   endless loop. Returns after target position has been reached */
int monitorStatus(epos_t *epos);
/*! \brief as monitorStatus(), but also waits for Homing Attained' signal */
int monitorHomingStatus(epos_t *epos);

/*! \brief waits for positoning to finish, argument is timeout in
   seconds. give timeout==0 to disable timeout */
int waitForTarget(epos_t *epos, unsigned int t);


#endif
