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

#include <string>

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

/* EPOS will reset communication after 500ms of inactivity */

/*! \brief try NTRY times to read one byte from EPOS, the give up */
#define NTRY      5
/*! \brief sleep TRYSLEEP usec between read() from EPOS, if no data available */
#define TRYSLEEP  (unsigned int)1e5

/* all high-level functions return <0 in case of error */

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
		int ReadObject(WORD index, BYTE subindex, WORD **answer );

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
		int WriteObject(WORD index, BYTE subindex, const WORD data[2]);

		/* helper functions below */

		/*! \brief  write a single BYTE to EPOS */
		int writeBYTE(const BYTE *c);

		/*! \brief  write a single WORD to EPOS */
		int writeWORD(const WORD *w);

		/*! \brief  read a single BYTE from EPOS, timeout implemented */
		int readBYTE(BYTE *c);

		/*! \brief  read a single WORD from EPOS, timeout implemented */
		int readWORD(WORD *w);

		/*! \brief  send command to EPOS, taking care of all neccessary 'ack' and
		   checksum tests*/
		int sendCom(WORD *frame);

		/*! \brief  int readAnswer(WORD **ptr) - read an answer frame from EPOS */
		int readAnswer(WORD **ptr);

		/*! \brief Checksum calculation;
		copied from EPOS Communication Guide, p.8
		 */
		WORD CalcFieldCRC(const WORD *pDataArray, WORD numberOfWords);

		/*! \brief exit(-1) if ptr == NULL */
		void checkPtr(const void* ptr);

		/*! \brief compare two 16bit bitmasks, return 1 (true) or 0 (false) */
		int bitcmp(WORD a, WORD b);

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
		int checkEPOS();

		/*! \brief check global variable E_error for EPOS error code */
		int checkEPOSerror();

		/*! \brief check EPOS status, return state according to firmware spec 8.1.1 */
		int checkEPOSstate();
		/*! \brief pretty-print EPOS state */
		int printEPOSstate();
		/*! \brief change EPOS state   ==> firmware spec 8.1.3 */
		int changeEPOSstate(int state);

		/*! \brief example from EPOS com. guide: ask EPOS for software version

		 firmware spec 14.1.33
		 ** returns software version as HEX **

		 */
		int readSWversion();

		/*! \brief ask for device name,
		 device name is placed in 'name' (string must be big enough, NO CHECKING!!)
		 */
		int readDeviceName(char *name);

		/*! \brief ask for RS232 timeout; firmware spec 14.1.35 */
		int readRS232timeout();

		/*! \brief read digital input polarity mask */
		int readDInputPolarity(WORD* w);

		/*! \brief set home switch polarity -- firmware spec 14.1.47 */
		int setHomePolarity(int pol);

		/*! \brief read Statusword; 14.1.58 */
		int readStatusword(WORD *eposStatus);
		/*! \brief pretty-print Statusword */
		int printEPOSstatusword(WORD statusword);

		/*! \brief read EPOS control word (firmware spec 14.1.57) */
		int readControlword(WORD *w);
		/*! \brief pretty-print Controlword */
		int printEPOScontrolword(WORD controlword);

		/*! \brief set EPOS mode of operation -- 14.1.59 */
		int setOpMode(int OpMode);

		/*! \brief read and returns  EPOS mode of operation -- 14.1.60
		 here, RETURN(0) MEANS ERROR!
		 '-1' is a valid OpMode, but 0 is not!
		 */
		int readOpMode();

		/*! \brief read actual position; 14.1.61 */
		int readDemandPosition(long *val);
		/*! \brief read actual position; 14.1.62 */
		int readActualPosition(long *val);

		/*! \brief read position window; 14.1.64 */
		int readPositionWindow(unsigned long int *value);
		/*! \brief write position window; 14.1.64 */
		int writePositionWindow(unsigned long int value);

		/*! < by Martí Morta (mmorta@iri.upc.edu) > */

		/*! \brief read position window time; 14.1.67 */
		int readPositionWindowTime(unsigned int *time);
		/*! \brief read position window time; 14.1.67 */
		int writePositionWindowTime(unsigned int val);

		int readPositionSoftwareLimits(long *val, long *val2);
		int writePositionSoftwareLimits(long val, long val2);

		int writePositionProfileVelocity(unsigned long int vel);
		int writePositionProfileAcceleration(unsigned long int acc);
		int writePositionProfileDeceleration(unsigned long int dec);
		int writePositionProfileQuickStopDeceleration(unsigned long int qsdec);
		int writePositionProfileMaxVelocity(unsigned long int maxvel);
		int writePositionProfileType(int type);

		int readPositionProfileVelocity(unsigned long int *val);
		int readPositionProfileAcceleration(unsigned long int *val);
		int readPositionProfileDeceleration(unsigned long int *val);
		int readPositionProfileQuickStopDeceleration(unsigned long int *val);
		int readPositionProfileMaxVelocity(unsigned long int *val);
		int readPositionProfileType(int *val);

		int readVelocityNotationIndex(int *index);
		int writeVelocityNotationIndex(unsigned int val);

		int readSensorPulses(unsigned int *pulse);
		int readSensorType(unsigned int *type);
		int readSensorPolarity(unsigned int *polaritat);
		int writeSensorType(unsigned int val);
		int writeSensorPulses(unsigned int val);
		int writeSensorPolarity(unsigned int val);

		int readRS232Baudrate(unsigned int *type);
		int writeRS232Baudrate(unsigned int val);

		int readP(int *val);
		int readI(int *val);
		int readD(int *val);
		int readVFF(unsigned int *val);
		int readAFF(unsigned int *val);
		int writeP(int val);
		int writeI(int val);
		int writeD(int val);
		int writeVFF(unsigned int val);
		int writeAFF(unsigned int val);

		int readPcurrent(int *val);
		int readIcurrent(int *val);
		int writePcurrent(int val);
		int writeIcurrent(int val);
		int saveParameters();

		int readHomePosition(long int *val);
		int writeHomePosition(long int val);

		int readMotorContinousCurrentLimit(unsigned int *cur);
		int writeMotorContinousCurrentLimit(unsigned int cur);
		int readMotorOutputCurrentLimit(unsigned int *cur);
		int writeMotorOutputCurrentLimit(unsigned int cur);
		int readMotorPolePair(unsigned int *cur);
		int writeMotorPolePair(unsigned int cur);
		int readMotorMaxSpeedCurrent(unsigned int *cur);
		int writeMotorMaxSpeedCurrent(unsigned int cur);
		int readMotorThermalConstant(unsigned int *cur);
		int writeMotorThermalConstant(unsigned int cur);

		/*! < by Martí Morta /> */

		/*! \brief read actual position; 14.1.67 */
		int readDemandVelocity(long *val);
		/*! \brief read actual position; 14.1.68 */
		int readActualVelocity(long *val);

		/*! \brief read actual current; 14.1.69 */
		int readActualCurrent(short *val);

		/*! \brief read target position; 14.1.70 */
		int readTargetPosition(long *val);

		/*! \brief does a homing move. Give homing mode (see firmware 9.3) and start
		 position */
		int doHoming(int method, long int start);

		/*! \brief set OpMode to ProfilePosition and make relative movement */
		int moveRelative(long int steps);
		/*! \brief set OpMode to ProfilePosition and make absolute movement */
		int moveAbsolute(long int steps);

		/*! \brief reads position, velocity and current and displays them in an
		 endless loop. Returns after target position has been reached */
		int monitorStatus();
		/*! \brief as monitorStatus(), but also waits for Homing Attained' signal */
		int monitorHomingStatus();

		/*! \brief waits for positoning to finish, argument is timeout in
		 seconds. give timeout==0 to disable timeout */
		int waitForTarget(unsigned int t);


};

#endif
