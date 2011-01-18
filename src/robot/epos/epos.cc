/*! \file epos.cc

 \brief libEPOS - a library to control an EPOS; implementation

 \addtogroup libEPOS Library to control an EPOS motor control unit

 \@{
 */

#include <cstdio>   /* Standard input/output definitions */
#include <cstring>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <cerrno>   /* Error number definitions */
#include <cstdlib>
#include <stdint.h>  /* int types with given size */
#include <cmath>
#include <sys/select.h>

#include "epos.h"

namespace mrrocpp {
namespace edp {
namespace epos {

/* ********************************************* */
/*    definitions used only internal in c   */
/* ********************************************* */

/*! starting point for (slow!) homing movement. If the zero point is
 not off too far, this will speed things up enormously!
 */
#define E_STARTPOS_HOMING -200000

/* EPOS error codes (Communication Guide, 6.4)  */

/* CANopen defined error codes */
#define E_NOERR         0x00000000   ///< Error code: no error
#define E_ONOTEX        0x06020000   ///< Error code: object does not exist
#define E_SUBINEX       0x06090011   ///< Error code: subindex does not exist
#define E_OUTMEM        0x05040005   ///< Error code: out of memory
#define E_NOACCES       0x06010000   ///< Error code: Unsupported access to an object
#define E_WRITEONLY     0x06010001   ///< Error code: Attempt to read a write-only object
#define E_READONLY      0x06010002   ///< Error code: Attempt to write a read-only object
#define E_PARAMINCOMP   0x06040043   ///< Error code: general parameter incompatibility
#define E_INTINCOMP     0x06040047   ///< Error code: general internal incompatibility in the device
#define E_HWERR         0x06060000   ///< Error code: access failed due to an hardware error
#define E_PRAGNEX       0x06090030   ///< Error code: value range of parameter exeeded
#define E_PARHIGH       0x06090031   ///< Error code: value of parameter written is too high
#define E_PARLOW        0x06090032   ///< Error code: value of parameter written is too low
#define E_PARREL        0x06090036   ///< Error code: maximum value is less than minimum value
/* maxon specific error codes */
#define E_NMTSTATE      0x0f00ffc0   ///< Error code: wrong NMT state
#define E_RS232         0x0f00ffbf   ///< Error code: rs232 command illegeal
#define E_PASSWD        0x0f00ffbe   ///< Error code: password incorrect
#define E_NSERV         0x0f00ffbc   ///< Error code: device not in service mode
#define E_NODEID        0x0f00fb9    ///< Error code: error in Node-ID
/* EPOS Statusword -- singe bits, see firmware spec 14.1.58 */
#define E_BIT15        0x8000      ///< bit code: position referenced to home position
#define E_BIT14        0x4000      ///< bit code: refresh cycle of power stage
#define E_BIT13        0x2000      ///< bit code: OpMode specific, some error
#define E_BIT12        0x1000      ///< bit code: OpMode specific
#define E_BIT11        0x0800      ///< bit code: NOT USED
#define E_BIT10        0x0400      ///< bit code: Target reached
#define E_BIT09        0x0200      ///< bit code: Remote (?)
#define E_BIT08        0x0100      ///< bit code: offset current measured (?)
#define E_BIT07        0x0080      ///< bit code: WARNING
#define E_BIT06        0x0040      ///< bit code: switch on disable
#define E_BIT05        0x0020      ///< bit code: quick stop
#define E_BIT04        0x0010      ///< bit code: voltage enabled
#define E_BIT03        0x0008      ///< bit code: FAULT
#define E_BIT02        0x0004      ///< bit code: operation enable
#define E_BIT01        0x0002      ///< bit code: switched on
#define E_BIT00        0x0001      ///< bit code: ready to switch on
/* EPOS modes of operation, firmware spec 14.1.59 (p.133, tbl. 72) */
#define E_HOMING      6 ///< EPOS operation mode: homing
#define E_PROFVEL     3 ///< EPOS operation mode: profile velocity mode
#define E_PROFPOS     1 ///< EPOS operation mode: profile position mode
// the modes below should not be used by user, defined here only for
// completeness
#define E_POSMOD     -1 ///< EPOS operation mode: position mode
#define E_VELMOD     -2 ///< EPOS operation mode: velocity mode
#define E_CURRMOD    -3 ///< EPOS operation mode: current mode
#define E_DIAGMOD    -4 ///< EPOS operation mode: diagnostics mode
#define E_MASTERENCMOD -5 ///< EPOS operation mode:internal
#define E_STEPDIRECMOD -6 ///< EPOS operation mode:internal

/************************************************************/
/*           implementation of functions are following      */
/************************************************************/

epos_base::epos_base() :
	device_opened(false)
{
}

epos_base::~epos_base()
{
}

/************************************************************/
/*          high-level read functions */
/************************************************************/

epos::epos(epos_base & _device, uint8_t _nodeId) :
	device(_device), nodeId(_nodeId)
{
}

/* read EPOS status word */
UNSIGNED16 epos::readStatusWord()
{
	return ReadObjectValue<UNSIGNED16>(0x6041, 0x00);
}

void epos::printEPOSstatusword(WORD s)
{
	printf("\nmeaning of EPOS statusword %#06x is:\n", s);

	printf("15: position referenced to home position: ");
	if ((s & E_BIT15) == E_BIT15)
		printf("true\n");
	else
		printf("false\n");

	printf("14: refresh cycle of power stage:         ");
	if ((s & E_BIT14) == E_BIT14)
		printf("true\n");
	else
		printf("false\n");

	printf("13: OpMode specific, some error:          ");
	if ((s & E_BIT13) == E_BIT13)
		printf("true\n");
	else
		printf("false\n");

	printf("12: OpMode specific:                      ");
	if ((s & E_BIT12) == E_BIT12)
		printf("true\n");
	else
		printf("false\n");

	printf("11: NOT USED                              ");
	if ((s & E_BIT11) == E_BIT11)
		printf("true\n");
	else
		printf("false\n");

	printf("10: Target reached:                       ");
	if ((s & E_BIT10) == E_BIT10)
		printf("true\n");
	else
		printf("false\n");

	printf("09: Remote (?)                            ");
	if ((s & E_BIT09) == E_BIT09)
		printf("true\n");
	else
		printf("false\n");

	printf("08: offset current measured (?)           ");
	if ((s & E_BIT08) == E_BIT08)
		printf("true\n");
	else
		printf("false\n");

	printf("07: WARNING                               ");
	if ((s & E_BIT07) == E_BIT07)
		printf("true\n");
	else
		printf("false\n");

	printf("06: switch on disable                     ");
	if ((s & E_BIT06) == E_BIT06)
		printf("true\n");
	else
		printf("false\n");

	printf("05: quick stop                            ");
	if ((s & E_BIT05) == E_BIT05)
		printf("true\n");
	else
		printf("false\n");

	printf("04: voltage enabled                       ");
	if ((s & E_BIT04) == E_BIT04)
		printf("true\n");
	else
		printf("false\n");

	printf("03: FAULT                                 ");
	if ((s & E_BIT03) == E_BIT03)
		printf("true\n");
	else
		printf("false\n");

	printf("02: operation enable                      ");
	if ((s & E_BIT02) == E_BIT02)
		printf("true\n");
	else
		printf("false\n");

	printf("01: switched on                           ");
	if ((s & E_BIT01) == E_BIT01)
		printf("true\n");
	else
		printf("false\n");

	printf("00: ready to switch on                    ");
	if ((s & E_BIT00) == E_BIT00)
		printf("true\n");
	else
		printf("false\n");
}

/*! check EPOS state, firmware spec 8.1.1

 \return EPOS status as defined in firmware specification 8.1.1

 */
int epos::checkEPOSstate()
{
	WORD w = readStatusWord();

	/* state 'start' (0)
	 fedc ba98  7654 3210
	 w == x0xx xxx0  x000 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && !bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (0);

	/* state 'not ready to switch on' (1)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (1);

	/* state 'switch on disabled' (2)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x100 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (2);

	/* state 'ready to switch on' (3)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x010 0001 */
	if (bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (3);

	/* state 'switched on' (4)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x010 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (4);

	/* state 'refresh' (5)
	 fedc ba98  7654 3210
	 w == x1xx xxx1  x010 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && bitcmp(w, E_BIT14))
		return (5);

	/* state 'measure init' (6)
	 fedc ba98  7654 3210
	 w == x1xx xxx1  x011 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04)
			&& bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && bitcmp(w, E_BIT14))
		return (6);

	/* state 'operation enable' (7)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x011 0111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04)
			&& bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (7);

	/* state 'quick stop active' (8)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x001 0111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (8);

	/* state 'fault reaction active (disabled)' (9)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 1111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (9);

	/* state 'fault reaction active (enabled)' (10)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x001 1111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (10);

	/* state 'fault' (11)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 1000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (11);

	// if we get down here, statusword has a unknown value!
	fprintf(stderr, "WARNING: EPOS status word %#06x is an unkown state!\n", w);
	fprintf(stderr, "(function %s() in file %s, line %d)\n", __func__, __FILE__, __LINE__);

	return (-2);
}

/* pretty-print EPOS state */
int epos::printEPOSstate()
{
	printf("\nEPOS is in state ");

	switch (checkEPOSstate()) {
		case 0:
			printf("start\n");
			break;
		case 1:
			printf("Not ready to switch on.\n");
			break;
		case 2:
			printf("Switch on disabled.\n");
			break;
		case 3:
			printf("Ready to switch on.\n");
			break;
		case 4:
			printf("Switched on.\n");
			break;
		case 5:
			printf("Refresh.\n");
			break;
		case 6:
			printf("Measure init.\n");
			break;
		case 7:
			printf("Operation enable.\n");
			break;
		case 8:
			printf("Quick stop active\n");
			break;
		case 9:
			printf("Fault reaction active (disabled)\n");
			break;
		case 10:
			printf("Fault reaction active (enabled)\n");
			break;
		case 11:
			printf("FAULT\n");
			break;

		default:
			printf("UNKNOWN!\n");
			return (-1);
	}
	return (0);
}

/* change EPOS state according to firmware spec 8.1.3 */
void epos::changeEPOSstate(state_t state)
{
	WORD dw[2];

	dw[1] = 0x0000; // high WORD of DWORD is not used here

	/* ! DO NOT READ OLD CONTROLWORD BACK, JUST SET THE BITS. It works
	 this way, but does NOT work otherways! -- mh, 07.07.06
	 */

	dw[0] = 0x0000;

	switch (state) {
		case ST_DISABLED: //shutdown, controlword: 0xxx x110
			dw[0] &= ~E_BIT15; // bit 15 ->0
			dw[0] |= E_BIT02; // bit 02 ->1
			dw[0] |= E_BIT01;
			dw[0] &= ~E_BIT00;

			WriteObject(0x6040, 0x00, dw);
			break;

		case ST_ENABLED: // switch on, controllword: 0xxx x111
			dw[0] &= ~E_BIT15;
			dw[0] |= E_BIT02;
			dw[0] |= E_BIT01;
			dw[0] |= E_BIT00;

			WriteObject(0x6040, 0x00, dw);
			break;

		case ST_QUICKSTOP: // disable voltage, controllword: 0xxx xx0x
			dw[0] &= ~E_BIT15;
			dw[0] &= ~E_BIT02;

			WriteObject(0x6040, 0x00, dw);
			break;

		case ST_FAULT: // quick stop, controllword: 0xxx x01x
			dw[0] &= ~E_BIT15;
			dw[0] &= ~E_BIT02;
			dw[0] |= E_BIT02;

			WriteObject(0x6040, 0x00, dw);
			break;
#if 0
		case 4: // disable operation, controllword: 0xxx 0111
			dw[0] &= ~E_BIT15;
			dw[0] &= ~E_BIT03;
			dw[0] |= E_BIT02;
			dw[0] |= E_BIT01;
			dw[0] |= E_BIT00;

			WriteObject(0x6040, 0x00, dw);
			break;

		case 5: // enable operation, controllword: 0xxx 1111
			dw[0] &= ~E_BIT15;
			dw[0] |= E_BIT03;
			dw[0] |= E_BIT02;
			dw[0] |= E_BIT01;
			dw[0] |= E_BIT00;

			WriteObject(0x6040, 0x00, dw);
			break;

		case 6: // fault reset, controllword: 1xxx xxxx

			//dw[0] |= E_BIT15; this is according to firmware spec 8.1.3,
			//but does not work!
			dw[0] |= E_BIT07; // this is according to firmware spec 14.1.57
			// and IS working!


			/*       WORD estatus = 0x0; */
			/*       if ( ( n = readStatusWord(&estatus) ) < 0) checkEPOSerror(); */
			/*       printEPOSstatusword(estatus); */

			WriteObject(0x6040, 0x00, dw);

			/*       if ( ( n = readStatusWord(&estatus) ) < 0) checkEPOSerror(); */
			/*       printEPOSstatusword(estatus); */

			break;
#endif
		default:
			throw epos_error() << reason("ERROR: demanded state is UNKNOWN!"); // TODO: state
	}
}

/* returns software version as HEX  --  14.1.33*/
UNSIGNED16 epos::readSWversion()
{
	return ReadObjectValue<UNSIGNED16>(0x2003, 0x01);
}

/* read digital input functionality polarity -- firmware spec 14.1.47 */
UNSIGNED16 epos::readDInputPolarity()
{
	return ReadObjectValue<UNSIGNED16>(0x2071, 0x03);
}

/* set home switch polarity -- firmware spec 14.1.47 */
void epos::setHomePolarity(int pol)
{
	if (pol != 0 && pol != 1) {
		throw epos_error() << reason("polarity must be 0 (height active) or 1 (low active)");
	}

	// read present functionalities polarity mask
	WORD mask = readDInputPolarity();

	// set bit 2 (==home switch) to 0 or 1:
	if (pol == 0)
		mask &= ~E_BIT02;
	else if (pol == 1)
		mask |= E_BIT02;

	WriteObjectValue(0x2071, 0x03, mask);
}

/* read EPOS control word (firmware spec 14.1.57) */
UNSIGNED16 epos::readControlword()
{
	return ReadObjectValue<UNSIGNED16>(0x6040, 0x00);
}

/* pretty-print Controlword */
void epos::printEPOScontrolword(WORD s)
{
	printf("\nmeaning of EPOS controlword %#06x is:\n", s);
	// bit 15..11 not in use
	// bit 10, 9 reserved
	printf("  HALT:                                 ");
	if ((s & E_BIT08) == E_BIT08)
		printf("true\n");
	else
		printf("false\n");

	printf("  fault reset                           ");
	if ((s & E_BIT07) == E_BIT07)
		printf("true\n");
	else
		printf("false\n");

	printf("  Op mode specific                      ");
	if ((s & E_BIT06) == E_BIT06)
		printf("true\n");
	else
		printf("false\n");

	printf("  Op mode specific                      ");
	if ((s & E_BIT05) == E_BIT05)
		printf("true\n");
	else
		printf("false\n");

	printf("  Op mode specific                      ");
	if ((s & E_BIT04) == E_BIT04)
		printf("true\n");
	else
		printf("false\n");

	printf("  enable operation                      ");
	if ((s & E_BIT03) == E_BIT03)
		printf("true\n");
	else
		printf("false\n");

	printf("  quick stop                            ");
	if ((s & E_BIT02) == E_BIT02)
		printf("true\n");
	else
		printf("false\n");

	printf("  enable voltage                        ");
	if ((s & E_BIT01) == E_BIT01)
		printf("true\n");
	else
		printf("false\n");

	printf("  switch on                             ");
	if ((s & E_BIT00) == E_BIT00)
		printf("true\n");
	else
		printf("false\n");
}

/* set mode of operation --- 14.1.59 */
void epos::setOpMode(operational_mode_t m)
{
	WORD dw[2];

	dw[1] = 0x0000; // high WORD of DWORD is not used here
	dw[0] = (int8_t) m;

	WriteObject(0x6060, 0x00, dw);
}

/* read mode of operation --- 14.1.60 */
INTEGER8 epos::readOpMode()
{
	return ReadObjectValue<INTEGER8>(0x6061, 0x00);
}

/* read demand position; 14.1.61 */
INTEGER32 epos::readDemandPosition()
{
	return ReadObjectValue<INTEGER32> (0x6062, 0x00);
}

/* read actual position; firmware description 14.1.62 */
INTEGER32 epos::readActualPosition()
{
	return ReadObjectValue<INTEGER32> (0x6064, 0x00);
}

/* read position window; 14.1.64 */
UNSIGNED32 epos::readPositionWindow()
{
	return ReadObjectValue<UNSIGNED32> (0x6067, 0x00);
}

/* write  position window; 14.1.64 */
void epos::writePositionWindow(UNSIGNED32 val)
{
	WriteObjectValue(0x6067, 0x00, val);
}

void epos::writePositionProfileVelocity(UNSIGNED32 val)
{
	WriteObjectValue(0x6081, 0x00, val);
}

void epos::writePositionProfileAcceleration(UNSIGNED32 val)
{
	WriteObjectValue(0x6083, 0x00, val);
}

void epos::writePositionProfileDeceleration(UNSIGNED32 val)
{
	WriteObjectValue(0x6084, 0x00, val);
}

void epos::writePositionProfileQuickStopDeceleration(UNSIGNED32 val)
{
	WriteObjectValue(0x6085, 0x00, val);
}

void epos::writePositionProfileMaxVelocity(UNSIGNED32 val)
{
	WriteObjectValue(0x607F, 0x00, val);
}

void epos::writePositionProfileType(INTEGER16 type)
{
	WriteObjectValue(0x6086, 0x00, type);
}

UNSIGNED32 epos::readPositionProfileVelocity()
{
	return ReadObjectValue<UNSIGNED32>(0x6081, 0x00);
}

UNSIGNED32 epos::readPositionProfileAcceleration()
{
	return ReadObjectValue<UNSIGNED32>(0x6083, 0x00);
}

UNSIGNED32 epos::readPositionProfileDeceleration()
{
	return ReadObjectValue<UNSIGNED32>(0x6084, 0x00);
}

UNSIGNED32 epos::readPositionProfileQuickStopDeceleration()
{
	return ReadObjectValue<UNSIGNED32>(0x6085, 0x00);
}

UNSIGNED32 epos::readPositionProfileMaxVelocity()
{
	return ReadObjectValue<UNSIGNED32>(0x607F, 0x00);
}

INTEGER16 epos::readPositionProfileType()
{
	return ReadObjectValue<INTEGER16>(0x6086, 0x00);
}

// by Martí Morta
/* Velocity Notation index 14.1.83 */
epos::velocity_notation_t epos::readVelocityNotationIndex()
{
	return (velocity_notation_t) ReadObjectValue<INTEGER8>(0x608B, 0x00);
}

/* Velocity Notation index 14.1.83  1=0x01(1), 2=0x02(2).. 0=0x00(0), -1=0xFF(255), -2=0xFE(254) */
void epos::writeVelocityNotationIndex(velocity_notation_t val)
{
	WriteObjectValue(0x608B, 0x00, val);
}

// by Martí Morta
/* read sensorConfiguration-sensor Pulses; 14.1.57 */
UNSIGNED32 epos::readSensorPulses()
{
	return ReadObjectValue<UNSIGNED32>(0x2210, 0x01);
}

/* read sensorConfiguration-sensor Type; 14.1.57 */
epos::sensor_type_t epos::readSensorType()
{
	return (sensor_type_t) ReadObjectValue<UNSIGNED16>(0x2210, 0x02);
}

/* read sensorPolarity-sensor Type; 14.1.57 */
UNSIGNED16 epos::readSensorPolarity()
{
	return ReadObjectValue<UNSIGNED16>(0x2210, 0x04);
}

/* write sensorConfiguration-sensor Pulses; 14.1.57 */
void epos::writeSensorPulses(UNSIGNED32 val)
{
	WriteObjectValue(0x2210, 0x01, val);
}

/* write sensorConfiguration-sensor Type; 14.1.57 */
void epos::writeSensorType(sensor_type_t val)
{
	WriteObjectValue(0x2210, 0x02, val);
}

/* write sensorPolarity-sensor Pulses; 14.1.57 */
void epos::writeSensorPolarity(UNSIGNED16 val)
{
	WriteObjectValue(0x2210, 0x04, val);
}

UNSIGNED16 epos::readRS232Baudrate()
{
	return ReadObjectValue<UNSIGNED16>(0x2002, 0x00);
}

void epos::writeRS232Baudrate(UNSIGNED16 val)
{
	WriteObjectValue(0x2002, 0x00, val);
}

// by Martí Morta
/* read P position; 14.1.92 */
INTEGER16 epos::readP()
{
	return ReadObjectValue<INTEGER16>(0x60FB, 0x01);
}

/* read I; 14.1.92 */
INTEGER16 epos::readI()
{
	return ReadObjectValue<INTEGER16>(0x60FB, 0x02);
}

/* read D; 14.1.92 */
INTEGER16 epos::readD()
{
	return ReadObjectValue<INTEGER16>(0x60FB, 0x03);
}

/* read Velocity Feed Forward; 14.1.92 */
UNSIGNED16 epos::readVFF()
{
	return ReadObjectValue<UNSIGNED16>(0x60FB, 0x04);
}

/* read Acceleration feed forward; 14.1.92 */
UNSIGNED16 epos::readAFF()
{
	return ReadObjectValue<UNSIGNED16> (0x60FB, 0x05);
}

/* write P; 14.1.92 */
void epos::writeP(INTEGER16 val)
{
	WriteObjectValue(0x60FB, 0x01, val);
}

/* write I; 14.1.92 */
void epos::writeI(INTEGER16 val)
{
	WriteObjectValue(0x60FB, 0x02, val);
}

/* write D; 14.1.92 */
void epos::writeD(INTEGER16 val)
{
	WriteObjectValue(0x60FB, 0x03, val);
}

/* write VFF; 14.1.92 */
void epos::writeVFF(UNSIGNED16 val)
{
	WriteObjectValue(0x60FB, 0x04, val);
}

/* write AFF; 14.1.92 */
void epos::writeAFF(UNSIGNED16 val)
{
	WriteObjectValue(0x60FB, 0x05, val);
}

/* read P current; 14.1.92 */
INTEGER16 epos::readPcurrent()
{
	return ReadObjectValue<INTEGER16>(0x60F6, 0x01);
}

/* read I current; 14.1.92 */
INTEGER16 epos::readIcurrent()
{
	return ReadObjectValue<INTEGER16>(0x60F6, 0x02);
}

/* write P current; 14.1.92 */
void epos::writePcurrent(INTEGER16 val)
{
	WriteObjectValue(0x60F6, 0x01, val);
}

/* write I current; 14.1.92 */
void epos::writeIcurrent(INTEGER16 val)
{
	WriteObjectValue(0x60F6, 0x02, val);
}

// by Martí Morta
/* save all parameters home; 14.1.55 */
void epos::saveParameters()
{
	WORD dw[2];

	// write sensor type
	dw[0] = (WORD) (0x6173);
	dw[1] = (WORD) (0x6576);

	WriteObject(0x1010, 0x01, dw);
}

// by Martí Morta
/* write home; 14.1.55 */
INTEGER32 epos::readHomePosition()
{
	return ReadObjectValue<INTEGER32>(0x2081, 0x00);
}

void epos::writeHomePosition(INTEGER32 val)
{
	WriteObjectValue(0x2081, 0x00, val);
}

// by Martí Morta
/* Motor Data 14.95 */
// Continous Current limit
UNSIGNED16 epos::readMotorContinousCurrentLimit()
{
	return ReadObjectValue<UNSIGNED16>(0x6410, 0x01);
}

void epos::writeMotorContinousCurrentLimit(UNSIGNED16 cur)
{
	WriteObjectValue(0x6410, 0x01, cur);
}

// Output Current limit
UNSIGNED16 epos::readMotorOutputCurrentLimit()
{
	return ReadObjectValue<UNSIGNED16>(0x6410, 0x02);
}

void epos::writeMotorOutputCurrentLimit(UNSIGNED16 cur)
{
	WriteObjectValue(0x6410, 0x02, cur);
}

// Pole Pairs -> 8 BITS
UNSIGNED8 epos::readMotorPolePair()
{
	return ReadObjectValue<UNSIGNED8>(0x6410, 0x03);
}

void epos::writeMotorPolePair(UNSIGNED8 cur)
{
	WriteObjectValue(0x6410, 0x03, cur);
}

// Max Speed in current mode
UNSIGNED32 epos::readMotorMaxSpeedCurrent()
{
	return ReadObjectValue<UNSIGNED32>(0x6410, 0x04);
}

void epos::writeMotorMaxSpeedCurrent(UNSIGNED32 val)
{
	WriteObjectValue(0x2081, 0x00, val);
}

// Thermal time constant in winding
UNSIGNED16 epos::readMotorThermalConstant()
{
	return ReadObjectValue<UNSIGNED16>(0x6410, 0x05);
}

void epos::writeMotorThermalConstant(UNSIGNED16 val)
{
	WriteObjectValue(0x6410, 0x05, val);
}

//------------- fi martí


/* read demand position; 14.1.67 */
INTEGER32 epos::readDemandVelocity()
{
	return ReadObjectValue<INTEGER32>(0x606b, 0x00);
}

/* read actual position; 14.1.68 */
INTEGER32 epos::readActualVelocity()
{
	return ReadObjectValue<INTEGER32>(0x606c, 0x00);
}

/* read actual motor current, see firmware description 14.1.69 */
INTEGER16 epos::readActualCurrent()
{
	return ReadObjectValue<INTEGER16> (0x6078, 0x00);
}

/* read EPOS target position; firmware description 14.1.70 */
INTEGER32 epos::readTargetPosition()
{
	return ReadObjectValue<INTEGER32> (0x607a, 0x00);
}

/* read manufacturer device name string firmware */
std::string epos::readDeviceName()
{
	WORD answer[8];
	ReadObject(answer, 8, 0x1008, 0x00);

	std::string str;

	str += (char) (answer[3] & 0x00FF);
	str += (char) ((answer[3] & 0xFF00) >> 8);
	str += (char) (answer[4] & 0x00FF);
	str += (char) ((answer[4] & 0xFF00) >> 8);
	// TODO: iterate until end of string

	return str;
}

/* firmware spec 14.1.35 */
UNSIGNED16 epos::readRS232timeout()
{
	return ReadObjectValue<UNSIGNED16> (0x2005, 0x00);
}

/* run the HomingMode, get the coordinate system zeropoint correct

 this is done as shown in "EPOS Application Note: device Programming,
 3: Homing Mode"

 */
int epos::doHoming(homing_method_t method, INTEGER32 start)
{
	//move motor to a pre-defined position before the reference
	//point. This will speed-up things if the coordinates are not too
	//wrong.

	moveAbsolute(start);

	// wait for positioning to finish, set timeout to approx. 30sec
	// CAUSES BIG PROBLEMS IF WE DO NOT WAIT!
	waitForTarget(30);
	//monitorStatus();

	// switch to homing mode
	setOpMode(OMD_HOMING_MODE);

	// homing speeds are left at default values.. (firmware 14.1.86)

	// set homing method
	WORD dw[2];
	dw[0] = method; // NO hex number here!
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	WriteObject(0x6098, 0x00, dw);

	// switch on
	dw[0] = 0x000f;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	WriteObject(0x6040, 0x00, dw);

	// start homing mode
	dw[0] = 0x001f;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	WriteObject(0x6040, 0x00, dw);

	//read/print status
	monitorHomingStatus();

	WORD w = readStatusWord();
	if ((w & E_BIT13) == E_BIT13) {
		fprintf(stderr, "\a *** got a HomingError! ***\n");
		return (-1);
	}

	if ((w & E_BIT12) == E_BIT12) {
		printf("homing finished!\n");
		return (0);
	} else {
		//  can this be reached? position finished, no homing error but
		//  homing NOT finished? I guess not..
		return (-5);
	}
}

void epos::moveRelative(INTEGER32 steps)
{
	// check, if we are in Profile Position Mode
	if (readOpMode() != OMD_PROFILE_POSITION_MODE) {
		setOpMode(OMD_PROFILE_POSITION_MODE);
	}

	// write intended target position
	// firmware 14.1.70
	WriteObjectValue(0x607A, 0x00, steps);

	// switch to relative positioning BY WRITING TO CONTROLWORD, finish
	// possible ongoing operation first!  ->maxon applicattion note:
	// device programming 2.1
	WriteObjectValue(0x6040, 0x00, 0x005f);
}

void epos::moveAbsolute(INTEGER32 steps)
{
#ifdef DEBUG
	printf("-> %s(): will move to %ld (%#010lx)\n", __func__, steps, steps);
#endif

	// check, if we are in Profile Position Mode
	if (readOpMode() != OMD_PROFILE_POSITION_MODE) {
		setOpMode(OMD_PROFILE_POSITION_MODE);
	}
#ifdef DEBUG
	printf("-> OpMode is (now) 'Profile Position Mode'. That's OK!\n");
#endif

	// write intended target position, is signed 32bit int
	// firmware 14.1.70
	WriteObjectValue(0x607A, 0x00, steps);

	// switch to absolute positioning, cancel possible ongoing operation
	// first!  ->maxon application note: device programming 2.1
	WriteObjectValue(0x6040, 0x00, 0x3f);
}

// monitor device status
void epos::monitorStatus()
{
	long int postarget, posactual, veldemand, velactual;
	short curactual;
	UNSIGNED16 status;

	printf("\nEPOS operating figures (note: update here is done AS FAST AS POSSIBLE!):\n");
	int i = 0;
	do {
		i++;
		postarget = readTargetPosition();
		posactual = readActualPosition();
		veldemand = readDemandVelocity();
		velactual = readActualVelocity();
		curactual = readActualCurrent();

		printf("\rEPOS: pos=%+10ld |%+10ld (%ld to go); v= %+4ld | %+4ld[rpm]; I=%+4dmA", postarget, posactual, postarget
				- posactual, veldemand, velactual, curactual);
		fflush(stdout);

		status = readStatusWord();
	} while ((status & E_BIT10) != E_BIT10); // bit 10 says: target reached!

	// update values a last time to get a nicer output:
	i++;
	postarget = readTargetPosition();
	posactual = readActualPosition();
	veldemand = readDemandVelocity();
	velactual = readActualVelocity();
	curactual = readActualCurrent();

	printf("\r%d EPOS: pos=%+10ld |%+10ld (%ld to go); v= %+4ld | %+4ld[rpm]; I=%+4dmA\n", i, postarget, posactual, postarget
			- posactual, veldemand, velactual, curactual);
	printf("target reached\n");
}

void epos::monitorHomingStatus()
{
	long int posactual, velactual;
	short curactual;
	UNSIGNED16 status;

	printf("\nEPOS operating figures (note: update here is done AS FAST AS POSSIBLE!):\n");
	int i = 0;
	do {
		i++;
		posactual = readActualPosition();
		velactual = readActualVelocity();
		curactual = readActualCurrent();

		status = readStatusWord();

		printf("\r%d EPOS: pos=%+10ld; v =  %+4ldrpm I=%+3dmA status = %#06x ", i, posactual, velactual, curactual, status);

		fflush(stdout);

		status = readStatusWord();

		if ((status & E_BIT13) == E_BIT13) {
			throw epos_error() << reason("HOMING ERROR!");
		}

	} while (((status & E_BIT10) != E_BIT10) && ((status & E_BIT12) != E_BIT12));
	// bit 10 says: target reached!, bit 12: homing attained
	//printEPOSstatusword(status);

	i++;
	posactual = readActualPosition();
	velactual = readActualVelocity();
	curactual = readActualCurrent();

	status = readStatusWord();

	printf("\r%d EPOS: pos=%+10ld; v =  %+4ldrpm I=%+3dmA status = %#06x\n", i, posactual, velactual, curactual, status);
	printf("homing finished! Position should now be '0'\n");
}

/* waits for positoning to finish, argument is timeout in
 seconds. give timeout==0 to disable timeout */
int epos::waitForTarget(unsigned int t)
{

	WORD status;
	unsigned int i = 0, st = (unsigned int) 1e4;

	do {
		if (t != 0) { // use timeout?
			if (++i > t * 1e2)
				return (1);
		}
		usleep(st);
		status = readStatusWord();
	} while ((status & E_BIT10) != E_BIT10); // bit 10 says: target reached!


	return (0);
}

/*
 *************************************************************
 check EPOS error code
 ****************************************************************
 */

/* check the global variable E_error for EPOS error code */
int epos::checkEPOSerror(DWORD E_error)
{
	const char *msg;
	switch (E_error) {
		case E_NOERR:
			return (0);
			break;
		case E_ONOTEX:
			msg = "requested object does not exist!";
			break;
		case E_SUBINEX:
			msg = "requested subindex does not exist!";
			break;
		case E_OUTMEM:
			msg= "out of memory!";
			break;
		case E_NOACCES:
			msg= "unsupported access to an object!";
			break;
		case E_WRITEONLY:
			msg= "attempt to read a write-only object!";
			break;
		case E_READONLY:
			msg= "attempt to write a read-only object!";
			break;
		case E_PARAMINCOMP:
			msg= "general parameter incompatibility!";
			break;
		case E_INTINCOMP:
			msg= "general internal incompatibility in the device!";
			break;
		case E_HWERR:
			msg= "access failed due to an HARDWARE ERROR!";
			break;
		case E_PRAGNEX:
			msg= "value range of parameter exceeded!";
			break;
		case E_PARHIGH:
			msg= "value of parameter written is too high!";
			break;
		case E_PARLOW:
			msg= "value of parameter written is too low!";
			break;
		case E_PARREL:
			msg= "maximum value is less than minimum value!";
			break;
		case E_NMTSTATE:
			msg= "wrong NMT state!";
			break;
		case E_RS232:
			msg= "rs232 command illegal!";
			break;
		case E_PASSWD:
			msg= "password incorrect!";
			break;
		case E_NSERV:
			msg= "device not in service mode!";
			break;
		case E_NODEID:
			msg= "error in Node-ID!";
			break;
		default:
			msg = "unknown EPOS error code"; //TODO: %x\n", E_error);
			break;
	}
	//EPOS responds with error:
	return (-1);
}

/* copied from EPOS Communication Guide, p.8 */
WORD epos_base::CalcFieldCRC(const WORD *pDataArray, WORD numberOfWords)
{
	WORD shifter, c;
	WORD carry;
	WORD CRC = 0;

	//Calculate pDataArray Word by Word
	while (numberOfWords--) {
		shifter = 0x8000; //Initialize BitX to Bit15
		c = *pDataArray++; //Copy next DataWord to c
		do {
			carry = CRC & 0x8000; //Check if Bit15 of CRC is set
			CRC <<= 1; //CRC = CRC * 2
			if (c & shifter)
				CRC++; //CRC = CRC + 1, if BitX is set in c
			if (carry)
				CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true
			shifter >>= 1; //Set BitX to next lower Bit,
			//shifter = shifter/2
		} while (shifter);
	}

	//printf("checksum == %#06x\n", CRC);
	return CRC;
}

unsigned int epos::ReadObject(WORD *ans, unsigned int ans_len,
		WORD index, BYTE subindex, uint8_t nodeId)
{
	WORD frame[4];

	frame[0] = 0x0210;
	frame[1] = index;
	/*
	 * high BYTE: 0x00(Node-ID == 0)
	 * low BYTE: subindex
	 */
	frame[2] = ((nodeId << 8 ) | subindex);
	frame[3] = 0x0000;

	device.sendCommand(frame);

	// read response
	return (device.readAnswer(ans, ans_len));
}

#if 0
/*! NOT USED IN libEPOS so far -> untested!
 */
static int InitiateSegmentedRead(WORD index, BYTE subindex ) {

	WORD frame[4], **ptr;

	frame[0] = 0x1201; // fixed, opCode==0x12, (len-1) == 1
	frame[1] = index;
	frame[2] = 0x0000 | subindex; /* high BYTE: 0x00 (Node-ID == 0)
	 low BYTE: subindex */
	frame[3] = 0x000; // ZERO word, will be filled with checksum

	sendCommand(frame);

	// read response
	return( readAnswer(ptr) ); // answer contains only DWORD ErrorCode
	// here...
}

/*! NOT USED IN libEPOS so far -> untested! */
static int SegmentRead(WORD **ptr) {

	WORD frame[3];
	int n;

	frame[0] = 0x1400; // fixed, opCode==0x14, (len-1) == 0
	frame[1] = 0x0000; // WHAT IS THE 'TOGGLE' BIT????
	frame[2] = 0x0000; // ZERO word, will be filled with checksum

	sendCommand(frame);

	readAnswer(ptr);

	return(0);
}
#endif

/* Low-level function to write an object to EPOS memory. Is called by
 writing libEPOS functions. */
void epos::WriteObject(WORD index, BYTE subindex, const WORD data[2], uint8_t nodeId)
{
	WORD frame[6];

	frame[0] = 0x0411; // fixed: (len-1) == 3, WriteObject
	frame[1] = index;
	frame[2] = ((nodeId << 8 ) | subindex); /* high BYTE: 0x00(Node-ID == 0), low BYTE: subindex */
	// data to transmit
	frame[3] = data[0];
	frame[4] = data[1];
	frame[5] = 0x00; // ZERO word, will be filled with checksum

	device.sendCommand(frame);

	// read response
	WORD answer[8];
	device.readAnswer(answer, 8);

	checkEPOSerror(device.E_error);
}

void epos::WriteObjectValue(WORD index, BYTE subindex, uint32_t data)
{
	WORD dw[2];

	// write data
	dw[0] = (WORD) (data & 0x0000FFFF);
	dw[1] = (WORD) (data >> 16);

	WriteObject(index, subindex, dw);
}

/* compare WORD a with WORD b bitwise */
bool epos::bitcmp(WORD a, WORD b)
{
	return ((a & b) == b) ? true : false;
}

//! \@}

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */

