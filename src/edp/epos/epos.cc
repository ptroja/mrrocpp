/*! \file c

\brief libEPOS - a library to control an EPOS 24/1

*/

/*! \mainpage libEPOS - a library to control an EPOS motor control unit

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

\author Marcus Hauser, LSW Heidelberg
\date July 2006



*/

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <stdlib.h>
#include <stdint.h>  /* int types with given size */
#include <math.h>

#include "epos.h"



/* ********************************************* */
/*    definitions used only internal in c   */
/* ********************************************* */


/*! starting point for (slow!) homing movement. If the zero point is
  not off too far, this will speed things up enormously!
*/
#define E_STARTPOS_HOMING -200000




/* EPOS codes */

#define E_OK      0x4f  ///< EPOS answer code for <em>all fine</em>
#define E_FAIL    0x46  ///< EPOS answer code to indicate a <em>failure</em>
#define E_ANS     0x00  ///< EPOS code to indicate an answer <em>frame</em>





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



/************************************************************/
/*            create/delete EPOS object                     */
/************************************************************/



epos::epos(const std::string & _device) :
	device(_device), ep(-1), gMarker (0) {
}

epos::~epos() {
  if (ep >= 0)
    closeEPOS();
}


/************************************************************/
/*            open/close device                             */
/************************************************************/



// von hans (atomsps.c, opensps() )
/*! establish the connection to EPOS

\param dev string describing the device on which the EPOS is connected
to, e.g. "/dev/ttyS0"
\param br baudrate for the transmission.

\retval 0 success
\retval -1 failure

*/
int epos::openEPOS(tcflag_t br) {
 int i;

 /* EPOS transfer format is:
    1 start bit
    8 data bits
    no parity
    1 stop bit
 */

  if(ep >=0) return(-1);

  for(i=0;i<5;i++) {
    if((ep=open(device.c_str(),O_RDWR|O_NOCTTY|O_NDELAY))>=0) break;
    sleep(1);
  }

  if(ep==-1) {
    perror("open serial port");
    return(-1);
  }

  if(tcgetattr(ep,&options)<0) {
    perror("tcgetattr");
  }

  memset(&options,0,sizeof(options));

  //  options.c_cflag |= B9600;
  options.c_cflag |= br;
  options.c_cflag |= CS8;           //8 bits per byte


  options.c_cflag |= CLOCAL|CREAD;

  tcflush(ep,TCIFLUSH);

  if(tcsetattr(ep,TCSANOW,&options)<0) {
    perror("tcsetattr");
  }

  fcntl(ep,F_SETFL,FNDELAY);

  return(0);
}

int epos::closeEPOS(){
  close(ep);
  return(0);
}






/************************************************************/
/*          high-level read functions */
/************************************************************/



/*! read EPOS status word

\param pointer on the EPOS object.
\param status pointer to WORD, the content of EPOS statusword will be placed there

\retval 0 success
\retval -1 failure

*/
int epos::readStatusword(WORD *status){

  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x6041, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }

  // check error code
  checkEPOSerror();

#ifdef DEBUG
  printf("==> EPOS status word: %#06x\n", answer[3]);
#endif
  *status = answer[3];
  free(answer); // memory is allocated by readAnswer()
  return(0);
}


/*! pretty-print Statusword to stdout

\param s WORD variable holding the statusword

*/
int epos::printEPOSstatusword(WORD s){

  printf("\nmeaning of EPOS statusword %#06x is:\n", s);


  printf("15: position referenced to home position: ");
  if ( (s & E_BIT15) == E_BIT15 ) printf("true\n");
  else printf("false\n");

  printf("14: refresh cycle of power stage:         ");
  if ( (s & E_BIT14) == E_BIT14 ) printf("true\n");
  else printf("false\n");

  printf("13: OpMode specific, some error:          ");
  if ( (s & E_BIT13) == E_BIT13 ) printf("true\n");
  else printf("false\n");

  printf("12: OpMode specific:                      ");
  if ( (s & E_BIT12) == E_BIT12 ) printf("true\n");
  else printf("false\n");

  printf("11: NOT USED                              ");
  if ( (s & E_BIT11) == E_BIT11 ) printf("true\n");
  else printf("false\n");

  printf("10: Target reached:                       ");
  if ( (s & E_BIT10) == E_BIT10 ) printf("true\n");
  else printf("false\n");

  printf("09: Remote (?)                            ");
  if ( (s & E_BIT09) == E_BIT09 ) printf("true\n");
  else printf("false\n");

  printf("08: offset current measured (?)           ");
  if ( (s & E_BIT08) == E_BIT08 ) printf("true\n");
  else printf("false\n");

  printf("07: WARNING                               ");
  if ( (s & E_BIT07) == E_BIT07 ) printf("true\n");
  else printf("false\n");

  printf("06: switch on disable                     ");
  if ( (s & E_BIT06) == E_BIT06 ) printf("true\n");
  else printf("false\n");

  printf("05: quick stop                            ");
  if ( (s & E_BIT05) == E_BIT05 ) printf("true\n");
  else printf("false\n");

  printf("04: voltage enabled                       ");
  if ( (s & E_BIT04) == E_BIT04 ) printf("true\n");
  else printf("false\n");

  printf("03: FAULT                                 ");
  if ( (s & E_BIT03) == E_BIT03 ) printf("true\n");
  else printf("false\n");

  printf("02: operation enable                      ");
  if ( (s & E_BIT02) == E_BIT02 ) printf("true\n");
  else printf("false\n");

  printf("01: switched on                           ");
  if ( (s & E_BIT01) == E_BIT01 ) printf("true\n");
  else printf("false\n");

  printf("00: ready to switch on                    ");
  if ( (s & E_BIT00) == E_BIT00 ) printf("true\n");
  else printf("false\n");

  return(0);
}




/*! check EPOS state, firmware spec 8.1.1

\param pointer on the EPOS object.
\return EPOS status as defined in firmware specification 8.1.1

*/
int epos::checkEPOSstate(){

  WORD w = 0x0;
  int n;

  if ( (n=readStatusword(&w)) < 0) {
    fprintf(stderr, " *** %s: readStatusword() returned %d **\n",
            __func__, n);
    return(-1);
  }

  /* state 'start' (0)
       fedc ba98  7654 3210
  w == x0xx xxx0  x000 0000 */
  if (   !bitcmp(w, E_BIT00 ) && !bitcmp(w, E_BIT01 )
      && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && !bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(0);

  /* state 'not ready to switch on' (1)
          fedc ba98  7654 3210
     w == x0xx xxx1  x000 0000 */
  if (   !bitcmp(w, E_BIT00 ) && !bitcmp(w, E_BIT01 )
      && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(1);


  /* state 'switch on disabled' (2)
          fedc ba98  7654 3210
     w == x0xx xxx1  x100 0000 */
  if (   !bitcmp(w, E_BIT00 ) && !bitcmp(w, E_BIT01 )
      && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
      && bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(2);

  /* state 'ready to switch on' (3)
          fedc ba98  7654 3210
     w == x0xx xxx1  x010 0001 */
  if (   bitcmp(w, E_BIT00 ) && !bitcmp(w, E_BIT01 )
      && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(3);

  /* state 'switched on' (4)
          fedc ba98  7654 3210
     w == x0xx xxx1  x010 0011 */
  if (   bitcmp(w, E_BIT00 ) && bitcmp(w, E_BIT01 )
      && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(4);

  /* state 'refresh' (5)
          fedc ba98  7654 3210
     w == x1xx xxx1  x010 0011 */
  if (   bitcmp(w, E_BIT00 ) && bitcmp(w, E_BIT01 )
      && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && bitcmp(w, E_BIT14) ) return(5);

  /* state 'measure init' (6)
          fedc ba98  7654 3210
     w == x1xx xxx1  x011 0011 */
  if (   bitcmp(w, E_BIT00 ) && bitcmp(w, E_BIT01 )
      && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && bitcmp(w, E_BIT14) ) return(6);

  /* state 'operation enable' (7)
          fedc ba98  7654 3210
     w == x0xx xxx1  x011 0111 */
  if (   bitcmp(w, E_BIT00 ) && bitcmp(w, E_BIT01 )
      && bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(7);

  /* state 'quick stop active' (8)
          fedc ba98  7654 3210
     w == x0xx xxx1  x001 0111 */
  if (   bitcmp(w, E_BIT00 ) && bitcmp(w, E_BIT01 )
      && bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
      && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(8);

  /* state 'fault reaction active (disabled)' (9)
          fedc ba98  7654 3210
     w == x0xx xxx1  x000 1111 */
  if (   bitcmp(w, E_BIT00 ) && bitcmp(w, E_BIT01 )
      && bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03)
      && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(9);

  /* state 'fault reaction active (enabled)' (10)
          fedc ba98  7654 3210
     w == x0xx xxx1  x001 1111 */
  if (   bitcmp(w, E_BIT00 ) && bitcmp(w, E_BIT01 )
      && bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03)
      && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(10);

  /* state 'fault' (11)
          fedc ba98  7654 3210
     w == x0xx xxx1  x000 1000 */
  if (   !bitcmp(w, E_BIT00 ) && !bitcmp(w, E_BIT01 )
      && !bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03)
      && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
      && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
      && !bitcmp(w, E_BIT14) ) return(11);


  // if we get down here, statusword has a unknown value!
  fprintf(stderr, "WARNING: EPOS status word %#06x is an unkown state!\n", w );
  fprintf(stderr, "(function %s() in file %s, line %d)\n",
          __func__, __FILE__, __LINE__);

  return(-2);
}


/* pretty-print EPOS state */
int epos::printEPOSstate(){

  printf("\nEPOS is in state ");

  switch( checkEPOSstate() )
    {
    case 0: printf("start\n"); break;
    case 1: printf("Not ready to switch on.\n"); break;
    case 2: printf("Switch on disabled.\n"); break;
    case 3: printf("Ready to switch on.\n"); break;
    case 4: printf("Switched on.\n"); break;
    case 5: printf("Refresh.\n"); break;
    case 6: printf("Measure init.\n"); break;
    case 7: printf("Operation enable.\n"); break;
    case 8: printf("Quick stop active\n"); break;
    case 9: printf("Fault reaction active (disabled)\n"); break;
    case 10: printf("Fault reaction active (enabled)\n"); break;
    case 11: printf("FAULT\n"); break;

    default:
      printf("UNKNOWN!\n");
      return(-1);
    }
  return(0);
}


/* change EPOS state according to firmware spec 8.1.3 */
int epos::changeEPOSstate(int state){
  WORD dw[2];
  int n;

  dw[1] = 0x0000; // high WORD of DWORD is not used here

  /* ! DO NOT READ OLD CONTROLWORD BACK, JUST SET THE BITS. It works
     this way, but does NOT work otherways! -- mh, 07.07.06
  */

  dw[0] = 0x0000;

  switch (state)
    {
    case 0: //shutdown, controlword: 0xxx x110
      dw[0] &= ~E_BIT15;  // bit 15 ->0
      dw[0] |= E_BIT02;   // bit 02 ->1
      dw[0] |= E_BIT01;
      dw[0] &= ~E_BIT00;

      n = WriteObject(0x6040, 0x00, dw );
      if (n<0) {
        fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return(-1);
      }
      break;

    case 1: // switch on, controllword: 0xxx x111
      dw[0] &= ~E_BIT15;
      dw[0] |= E_BIT02;
      dw[0] |= E_BIT01;
      dw[0] |= E_BIT00;

      n = WriteObject(0x6040, 0x00, dw );
      if (n<0) {
        fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return(-1);
      }
      break;

    case 2: // disable voltage, controllword: 0xxx xx0x
      dw[0] &= ~E_BIT15;
      dw[0] &= ~E_BIT02;

      n = WriteObject(0x6040, 0x00, dw );
      if (n<0) {
        fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return(-1);
      }
      break;

    case 3: // quick stop, controllword: 0xxx x01x
      dw[0] &= ~E_BIT15;
      dw[0] &= ~E_BIT02;
      dw[0] |= E_BIT02;

      n = WriteObject(0x6040, 0x00, dw );
      if (n<0) {
        fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return(-1);
      }
      break;

    case 4: // disable operation, controllword: 0xxx 0111
      dw[0] &= ~E_BIT15;
      dw[0] &= ~E_BIT03;
      dw[0] |= E_BIT02;
      dw[0] |= E_BIT01;
      dw[0] |= E_BIT00;

      n = WriteObject(0x6040, 0x00, dw );
      if (n<0) {
        fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return(-1);
      }
      break;


    case 5: // enable operation, controllword: 0xxx 1111
      dw[0] &= ~E_BIT15;
      dw[0] |= E_BIT03;
      dw[0] |= E_BIT02;
      dw[0] |= E_BIT01;
      dw[0] |= E_BIT00;

      n = WriteObject(0x6040, 0x00, dw );
      if (n<0) {
        fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return(-1);
      }
      break;



    case 6: // fault reset, controllword: 1xxx xxxx

      //dw[0] |= E_BIT15; this is according to firmware spec 8.1.3,
      //but does not work!
      dw[0] |= E_BIT07; // this is according to firmware spec 14.1.57
                        // and IS working!


/*       WORD estatus = 0x0; */
/*       if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror(); */
/*       printEPOSstatusword(estatus); */


      n = WriteObject(0x6040, 0x00, dw );
      if (n<0) {
        fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return(-1);
      }

/*       if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror(); */
/*       printEPOSstatusword(estatus); */

      break;


    default:
      fprintf(stderr, "ERROR: demanded state %d is UNKNOWN!\n", state);
      return(-1);
    }
  return(0);
}





/* returns software version as HEX  --  14.1.33*/
int epos::readSWversion(){
  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x2003, 0x01, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    return(-1);
  }


  // check error code
  checkEPOSerror();

#ifdef DEBUG
  printf("=x=> SW-Version: %x\n", answer[3]);
#endif

  n =  (int) answer[3];

  free(answer); // memory is allocated by readAnswer()
  return(n);
}






/* read digital input functionality polarity -- firmware spec 14.1.47 */
int epos::readDInputPolarity(WORD* w){

  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x2071, 0x03, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    return(-1);
  }


  // check error code
  checkEPOSerror();

#ifdef DEBUG
  printf("==> polarity mask: %x\n", answer[3]);
#endif

  *w = answer[3];


  free(answer); // memory is allocated by readAnswer()
  return(0);
}





/* set home switch polarity -- firmware spec 14.1.47 */
int epos::setHomePolarity(int pol){

  WORD mask = 0x00;
  WORD dw[2] = {0x0, 0x0};
  int n;

  if (pol!=0 && pol!=1) {
    fprintf(stderr, "ERROR: polarity must be 0 (hight active) or 1 (low active)\n");
    return(-1);
  }

  // read present functionalities polarity mask
  if ( readDInputPolarity(&mask) ) {
    fprintf(stderr, "\aERROR while reading digital input polarity!\n");
    return(-2);
  }


  // set bit 2 (==home switch) to 0 or 1:
  if (pol == 0)      mask &= ~E_BIT02;
  else if (pol == 1) mask |= E_BIT02;



  dw[1] = 0x0000; // high WORD of DWORD is not used here
  dw[0] = mask;

  n = WriteObject(0x2071, 0x03, dw);
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }

  return(0);
}






/* read EPOS control word (firmware spec 14.1.57) */
int epos::readControlword(WORD *w){

  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x6040, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }

  // check error code
  checkEPOSerror();

#ifdef DEBUG
  printf("==> EPOS control word: %#06x\n", answer[3]);
#endif
  *w = answer[3];
  free(answer); // memory is allocated by readAnswer()
  return(0);
}



/* pretty-print Controlword */
int epos::printEPOScontrolword(WORD s){
  printf("\nmeaning of EPOS controlword %#06x is:\n", s);
  // bit 15..11 not in use
  // bit 10, 9 reserved
  printf("  HALT:                                 ");
  if ( (s & E_BIT08) == E_BIT08 ) printf("true\n");
  else printf("false\n");

  printf("  fault reset                           ");
  if ( (s & E_BIT07) == E_BIT07 ) printf("true\n");
  else printf("false\n");

  printf("  Op mode specific                      ");
  if ( (s & E_BIT06) == E_BIT06 ) printf("true\n");
  else printf("false\n");

  printf("  Op mode specific                      ");
  if ( (s & E_BIT05) == E_BIT05 ) printf("true\n");
  else printf("false\n");

  printf("  Op mode specific                      ");
  if ( (s & E_BIT04) == E_BIT04 ) printf("true\n");
  else printf("false\n");

  printf("  enable operation                      ");
  if ( (s & E_BIT03) == E_BIT03 ) printf("true\n");
  else printf("false\n");

  printf("  quick stop                            ");
  if ( (s & E_BIT02) == E_BIT02 ) printf("true\n");
  else printf("false\n");

  printf("  enable voltage                        ");
  if ( (s & E_BIT01) == E_BIT01 ) printf("true\n");
  else printf("false\n");

  printf("  switch on                             ");
  if ( (s & E_BIT00) == E_BIT00 ) printf("true\n");
  else printf("false\n");

  return(0);
}






/* set mode of operation --- 14.1.59 */
int epos::setOpMode(int m){

  WORD dw[2] = {0x0, 0x0};
  int n;

  dw[1] = 0x0000; // high WORD of DWORD is not used here
  dw[0] = m;

  n = WriteObject(0x6060, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }


  return(0);
}




/** read mode of operation --- 14.1.60

\return RETURN(0) MEANS ERROR! -1 is a valid OpMode, but 0 is not!

 */
int epos::readOpMode(){
  WORD *answer;
  //short int *i;
  int8_t aa;
  int n;

  if ( (n =  ReadObject(0x6061, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(0);
  }


  //w = answer[3];
  aa = answer[3];
  free(answer);
  // check error code
  checkEPOSerror();

  /*
  // return value is a 8bit signed integer. To convert it to a 16bit
  // signed int, move bit 7 (old signum) to bit 15 (new signum). Set
  // bit 7 to 0
  if ((w & (~E_BIT07)) == 0xFFFF){     //bit 07 is set
    w &= ~E_BIT07;    // set bit  7 to '0'
    w |= E_BIT15;     // set bit 15 to '1'

  }
  // the compiler must give a warning about signedness here, but it is
  // ok! Don't know how to suppress it...
  i = &w;

  */
  // give warning, if internal mode is used
  if (aa<0) fprintf(stderr,"WARNING: EPOS is set to internal mode of operation (%hd).\n Make sure that this was really intended!\n", aa);

  //return(*i);
  return(aa);
}




/* read demand position; 14.1.61 */
int epos::readDemandPosition(long *pos){
  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x6062, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }
  // check error code
  checkEPOSerror();

  // return value is a 32bit integer (==long int)
  *pos = answer[3]  | (answer[4] << 16);
  free(answer);
#ifdef DEBUG
  printf("==> EPOS actual position: %ld\n", *pos);
#endif
  return(0);
}




/*! read actual position; firmware description 14.1.62

\retval 0 success
\retval <0 some error, check with checkEPOSerror()
*/
int epos::readActualPosition(long *pos){
  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x6064, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }
  // check error code
  checkEPOSerror();

  // return value is a 32bit integer (==long int)
  *pos = answer[3]  | (answer[4] << 16);
  free(answer);
#ifdef DEBUG
  printf("==> %s(): EPOS actual position: %ld\n", __func__, *pos);
#endif

  return(0);
}



/* read position window; 14.1.64 */
int epos::readPositionWindow(unsigned long int *pos){
  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x6067, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }
  // check error code
  checkEPOSerror();

  // return value is a 32bit integer (==long int)
  *pos = answer[3]  | (answer[4] << 16);
  free(answer);

#ifdef DEBUG
  printf("==> %s(): EPOS position window is %ld\n", __func__, *pos);
#endif

  return(0);
}


/* write  position window; 14.1.64 */
int epos::writePositionWindow(unsigned long int val){

  WORD dw[2];
  int n;

  // write intended position window
  dw[0] = (WORD) (val & 0x0000FFFF);
  dw[1] = (WORD) (val >> 16) ;

  n = WriteObject(0x6067, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }
  checkEPOSerror();

  return(0);
}

int epos::writePositionProfileVelocity(unsigned long int val){
	WORD dw[2];
	int n;

	  // write intended velocity
	dw[0] = (WORD) (val & 0x0000FFFF); //la part de baix
	dw[1] = (WORD) (val >> 16) ; //la part de dalt

	n = WriteObject(0x6081, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

int epos::writePositionProfileAcceleration(unsigned long int val){
	WORD dw[2];
	int n;

	// write intended acceleration
	dw[0] = (WORD) (val & 0x0000FFFF);
	dw[1] = (WORD) (val >> 16) ;

	n = WriteObject(0x6083, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

int epos::writePositionProfileDeceleration(unsigned long int val){
	WORD dw[2];
	int n;

	// write intended deceleration
	dw[0] = (WORD) (val & 0x0000FFFF);
	dw[1] = (WORD) (val >> 16) ;

	n = WriteObject(0x6084, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

int epos::writePositionProfileQuickStopDeceleration(unsigned long int val){
	WORD dw[2];
	int n;

	// write intended quick stop deceleration
	dw[0] = (WORD) (val & 0x0000FFFF);
	dw[1] = (WORD) (val >> 16) ;

	n = WriteObject(0x6085, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

int epos::writePositionProfileMaxVelocity(unsigned long int val){
	WORD dw[2];
	int n;

	// write intended max profile velocity
	dw[0] = (WORD) (val & 0x0000FFFF);
	dw[1] = (WORD) (val >> 16) ;

	n = WriteObject(0x607F, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

int epos::writePositionProfileType(int type){
	WORD dw[2];
	int n;

	// write intended type
	dw[0] = type;
	dw[1] = 0x0000 ;

	n = WriteObject(0x6086, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

int epos::readPositionProfileVelocity(unsigned long int *val){
	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6081, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*val = answer[3]  | (answer[4] << 16);
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS profile Velocity is %ld\n", __func__, *pos);
#endif

	return(0);
}

int epos::readPositionProfileAcceleration(unsigned long int *val){
	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6083, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*val = answer[3]  | (answer[4] << 16);
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS profile Acceleration is %ld\n", __func__, *pos);
#endif

	return(0);
}

int epos::readPositionProfileDeceleration(unsigned long int *val){
	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6084, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*val = answer[3]  | (answer[4] << 16);
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS profile deceleration is %ld\n", __func__, *pos);
#endif

	return(0);
}

int epos::readPositionProfileQuickStopDeceleration(unsigned long int *val){
	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6085, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*val = answer[3]  | (answer[4] << 16);
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS profile quick stop deceleration is %ld\n", __func__, *pos);
#endif

	return(0);
}

int epos::readPositionProfileMaxVelocity(unsigned long int *val){
	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x607F, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*val = answer[3]  | (answer[4] << 16);
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS profile max velocity is %ld\n", __func__, *pos);
#endif

	return(0);
}

int epos::readPositionProfileType(int *val){
	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6086, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*val = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS pofile type is %ld\n", __func__, *pos);
#endif

	return(0);
}



// by Martí Morta
/* Velocity Notation index 14.1.83 */
int epos::readVelocityNotationIndex(int *index){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x608B, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 8bit integer ( int)

	*index = answer[3];
	free(answer);

	#ifdef DEBUG
	printf("==> %s(): velocity notation index %d \n", __func__, *index);
	printf("answer: %d %d %d %d",answer[1],answer[2],answer[3],answer[4]);
	#endif

	return(0);
}
/* Velocity Notation index 14.1.83  1=0x01(1), 2=0x02(2).. 0=0x00(0), -1=0xFF(255), -2=0xFE(254) */
int epos::writeVelocityNotationIndex(unsigned int val){
	WORD dw[2];
	int n;

  // write
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x608B, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
		__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}


// by Martí Morta
/* read sensorConfiguration-sensor Pulses; 14.1.57 */
int epos::readSensorPulses(unsigned int *pulse){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x2210, 0x01, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

	// return 16 bits
	*pulse = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS sensor pulses are %d ms\n", __func__, *time);
#endif

	return(0);
}
/* read sensorConfiguration-sensor Type; 14.1.57 */
int epos::readSensorType(unsigned int *type){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x2210, 0x02, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*type = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS sensor pulses are %d ms\n", __func__, *time);
#endif

	return(0);
}
/* read sensorPolarity-sensor Type; 14.1.57 */
int epos::readSensorPolarity(unsigned int *polaritat){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x2210, 0x04, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*polaritat = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS sensor polarity are %d ms\n", __func__, *time);
#endif

	return(0);
}
/* write sensorConfiguration-sensor Pulses; 14.1.57 */
int epos::writeSensorPulses(unsigned int val){
	WORD dw[2];
	int n;

  // write sensor pulses
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x2210, 0x01, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}
/* write sensorConfiguration-sensor Type; 14.1.57 */
int epos::writeSensorType(unsigned int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x2210, 0x02, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
		__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}
/* write sensorPolarity-sensor Pulses; 14.1.57 */
int epos::writeSensorPolarity(unsigned int val){
	WORD dw[2];
	int n;

  // write sensor pulses
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x2210, 0x04, dw );
	if (n<0) {
 		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

int epos::readRS232Baudrate(unsigned int *type){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x2002, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*type = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS RS232 Baudrate is %d ms\n", __func__, *time);
#endif

	return(0);
}
int epos::writeRS232Baudrate(unsigned int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x2002, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}


// by Martí Morta
/* read P position; 14.1.92 */
int epos::readP(int *val){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x60FB, 0x01, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

	// return 16 bits
	*val = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS P %d ms\n", __func__, *time);
#endif

	return(0);
}
/* read I; 14.1.92 */
int epos::readI(int *val){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x60FB, 0x02, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

	// return 16 bits
	*val = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS I %d ms\n", __func__, *time);
#endif

	return(0);
}
/* read D; 14.1.92 */
int epos::readD(int *val){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x60FB, 0x03, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

	// return 16 bits
	*val = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS D %d ms\n", __func__, *time);
#endif

	return(0);
}
/* read Velocity Feed Forward; 14.1.92 */
int epos::readVFF(unsigned int *val){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x60FB, 0x04, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

	// return 16 bits
	*val = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS VFF %d ms\n", __func__, *time);
#endif

	return(0);
}
/* read Acceleration feed forward; 14.1.92 */
int epos::readAFF(unsigned int *val){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x60FB, 0x05, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

	// return 16 bits
	*val = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS AFF %d ms\n", __func__, *time);
#endif

	return(0);
}
/* write P; 14.1.92 */
int epos::writeP(int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x60FB, 0x01, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}
/* write I; 14.1.92 */
int epos::writeI(int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x60FB, 0x02, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}
/* write D; 14.1.92 */
int epos::writeD(int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x60FB, 0x03, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}
/* write VFF; 14.1.92 */
int epos::writeVFF(unsigned int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x60FB, 0x04, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}
/* write AFF; 14.1.92 */
int epos::writeAFF(unsigned int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x60FB, 0x05, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}
/* read P current; 14.1.92 */
int epos::readPcurrent(int *val){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x60F6, 0x01, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

	// return 16 bits
	*val = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS P current %d ms\n", __func__, *time);
#endif

	return(0);
}
/* read I current; 14.1.92 */
int epos::readIcurrent(int *val){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x60F6, 0x02, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

	// return 16 bits
	*val = answer[3];
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS I current %d ms\n", __func__, *time);
#endif

	return(0);
}
/* write P current; 14.1.92 */
int epos::writePcurrent(int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x60F6, 0x01, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}
/* write I current; 14.1.92 */
int epos::writeIcurrent(int val){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = val;
	dw[1] = 0x0000;

	n = WriteObject(0x60F6, 0x02, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

// by Martí Morta
/* save all parameters home; 14.1.55 */
int epos::saveParameters(){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = (WORD) (0x6173);
	dw[1] = (WORD) (0x6576) ;

	n = WriteObject(0x1010, 0x01, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

// by Martí Morta
/* write home; 14.1.55 */
int epos::readHomePosition(long int *val){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6081, 0x00, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
  // check error code
	checkEPOSerror();

  // return value is a 32bit integer (==long int)
	*val = answer[3]  | (answer[4] << 16);
	free(answer);

	return(0);
}
int epos::writeHomePosition(long int val){
	WORD dw[2];
	int n;


  	// write intended
	dw[0] = (WORD) (val & 0x0000FFFF);
	dw[1] = (WORD) (val >> 16) ;

	n = WriteObject(0x2081, 0x00, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}


// by Martí Morta
/* Motor Data 14.95 */
// Continous Current limit
int epos::readMotorContinousCurrentLimit(unsigned int *cur){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6410, 0x01, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
 	 // check error code
	checkEPOSerror();

 	 // return value is a 32bit integer (==long int)
	*cur = answer[3];
	free(answer);

	return(0);
}
int epos::writeMotorContinousCurrentLimit(unsigned int cur){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = cur;
	dw[1] = 0x0000;

	n = WriteObject(0x6410, 0x01, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

// Output Current limit
int epos::readMotorOutputCurrentLimit(unsigned int *cur){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6410, 0x02, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
 	 // check error code
	checkEPOSerror();

 	 // return value is a 32bit integer (==long int)
	*cur = answer[3];
	free(answer);

	return(0);
}
int epos::writeMotorOutputCurrentLimit(unsigned int cur){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = cur;
	dw[1] = 0x0000;

	n = WriteObject(0x6410, 0x02, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

// Pole Pairs -> 8 BITS
int epos::readMotorPolePair(unsigned int *cur){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6410, 0x03, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
 	 // check error code
	checkEPOSerror();

 	 // return value is a 32bit integer (==long int)
	*cur = answer[3];
	free(answer);

	return(0);
}
int epos::writeMotorPolePair(unsigned int cur){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = cur;
	dw[1] = 0x0000;

	n = WriteObject(0x6410, 0x03, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

// Max Speed in current mode
int epos::readMotorMaxSpeedCurrent(unsigned int *cur){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6410, 0x04, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
 	 // check error code
	checkEPOSerror();

 	 // return value is a 32bit integer (==long int)
	*cur = answer[3];
	free(answer);

	return(0);
}
int epos::writeMotorMaxSpeedCurrent(unsigned int cur){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = cur;
	dw[1] = 0x0000;

	n = WriteObject(0x6410, 0x04, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}

// Thermal time constant in winding
int epos::readMotorThermalConstant(unsigned int *cur){

	WORD *answer;
	int n;

	if ( (n =  ReadObject(0x6410, 0x05, &answer)) <0){

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
			__func__, n);
		free(answer); // memory is allocated by readAnswer()
		return(-1);
	}
 	 // check error code
	checkEPOSerror();

 	 // return value is a 32bit integer (==long int)
	*cur = answer[3];
	free(answer);

	return(0);
}
int epos::writeMotorThermalConstant(unsigned int cur){
	WORD dw[2];
	int n;

  // write sensor type
	dw[0] = cur;
	dw[1] = 0x0000;

	n = WriteObject(0x6410, 0x05, dw );
	if (n<0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
			__func__, n, __FILE__, __LINE__);
		return(-1);
	}
	checkEPOSerror();

	return(0);
}



//------------- fi martí







/* read demand position; 14.1.67 */
int epos::readDemandVelocity(long *val){
  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x606b, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }
  // check error code
  checkEPOSerror();

  // return value is a 32bit integer (==long int)
  *val = answer[3]  | (answer[4] << 16);

#ifdef DEBUG
  printf("==> EPOS demand velocity: %ld\n", *val);
#endif

  return(0);
}



/* read actual position; 14.1.68 */
int epos::readActualVelocity(long *val){
  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x606c, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }
  // check error code
  checkEPOSerror();

  // return value is a 32bit integer (==long int)
  *val = answer[3]  | (answer[4] << 16);

#ifdef DEBUG
  printf("==> EPOS actual velocity: %ld\n", *val);
#endif

  return(0);
}






/*! read actual motor current, see firmware description 14.1.69

\param pointer on the EPOS object.
\param val pointer to short int where the actual motor current will be
placed.

\retval 0 success
\retval -1 error

*/
int epos::readActualCurrent(short int *val){
  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x6078, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }
  // check error code
  checkEPOSerror();

  *val = answer[3];
  free(answer);
#ifdef DEBUG
  printf("==> EPOS actual current: %dmA\n", *val);
#endif

  return(0);
}



/*!  read EPOS target position; firmware description 14.1.70

\param pointer on the EPOS object.
\param val pointer to long int, will be filled with EPOS target position
\retval 0 success
\retval -1 error

*/
int epos::readTargetPosition(long *val){
  WORD *answer;
  int n;

  if ( (n =  ReadObject(0x607a, 0x00, &answer)) <0){

    fprintf(stderr, " *** %s: ReadObject() returned %d **\n",
            __func__, n);
    free(answer); // memory is allocated by readAnswer()
    return(-1);
  }
  // check error code
  checkEPOSerror();

  // return value is a 32bit integer (==long int)
  *val = (DWORD) answer[3]  | (answer[4] << 16);
  free(answer);
#ifdef DEBUG
  printf("==> EPOS target position: %ld\n", *val);
#endif

  return(0);
}







/*! readDeviceName: read manufactor device name string firmware

\param pointer on the EPOS object.
\param str previously allocated string, will be filled with device name
\retval 0 success
\retval -1 error


 */
int epos::readDeviceName(char *str){
  WORD *answer;
  int n;

  if ( ( n = ReadObject(0x1008, 0x00, &answer) ) < 0) {
    printf(" *** readObject returned %d at %s, line %d ***\n",
           n, __func__, __LINE__);
  }



  str[0] = (answer[3] & 0x00FF);
  str[1] = (answer[3] & 0xFF00) >> 8;
  str[2] = (answer[4] & 0x00FF);
  str[3] = (answer[4] & 0xFF00) >> 8;
  str[4] = '\0';  // end of string

#ifdef DEBUG
  printf("%s: %s \n", __func__, str);
#endif


  free( answer ); // memory is allocated by readAnswer()
  return(0);
}






/* firmware spec 14.1.35 */
int epos::readRS232timeout(){

  WORD *answer;
  int n;

  if ( ( n = ReadObject(0x2005, 0x00, &answer) ) < 0) {
    printf(" *** readObject returned %d at %s, line %d ***\n",
           n, __func__, __LINE__);
  }

#ifdef DEBUG
  printf("%s: RS232 timeout is %d msec\n", __func__, answer[3] );
#endif

  n =  (int) answer[3];
  free(answer);  // memory is allocated by ReadAnswer()
  return( n );
}




/* run the HomingMode, get the coordinate system zeropoint correct

 this is done as shown in "EPOS Application Note: device Programming,
 3: Homing Mode"

*/
int epos::doHoming(int method, long int start){

  WORD dw[2] = {0x0000, 0x0000};
  WORD w = 0x0000;
  int n, status=0;

  //move motor to a pre-defined position before the reference
  //point. This will speed-up things if the coordinates are not too
  //wrong.

  if ( moveAbsolute(start) ) {
    fprintf(stderr, "ERROR: could not move to homing starting point!\n");
    fprintf(stderr, "       (problem at %s; %s line %d)\n",
            __func__, __FILE__, __LINE__);
    return(-1);
  }
  // wait for positioning to finish, set timeout to approx. 30sec
  // CAUSES BIG PROBLEMS IF WE DO NOT WAIT!
  waitForTarget(30);
  //monitorStatus();


  // switch to homing mode
  if( setOpMode(E_HOMING) ) {
    fprintf(stderr, "ERROR: problem at %s; %s line %d\n",
            __func__, __FILE__, __LINE__);
    return(-1);
  }


  // homing speeds are left at default values.. (firmware 14.1.86)


  // set homing method
  dw[0] = method; // NO hex number here!
  dw[1] = 0x0000; // high WORD of DWORD is not used here
  n = WriteObject(0x6098, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }
  checkEPOSerror();


  // switch on
  dw[0] = 0x000f;
  dw[1] = 0x0000; // high WORD of DWORD is not used here
  n = WriteObject(0x6040, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }
  // start homing mode
  dw[0] = 0x001f;
  dw[1] = 0x0000; // high WORD of DWORD is not used here
  n = WriteObject(0x6040, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }


  checkEPOSerror();


  //read/print status
  status = monitorHomingStatus();
  if ( status ) {
    // something was wrong during homing...
    if (status == 1) {
      fprintf(stderr, "We did more that 2 complete turns without finding the home switch!\n");
      fprintf(stderr, "\aDEVICE IS BROKEN!!!\n");
      exit(2);
    }
    else {
      fprintf(stderr, "got %d as response from monitorHoming()...this is BAD!\n", status);
      fprintf(stderr, "[ %s: at %s, line %d ]\n",
              __func__, __FILE__, __LINE__);
    }
  }

  readStatusword(&w);
  if ( (w & E_BIT13) == E_BIT13) {
    fprintf(stderr, "\a *** got a HomingError! ***\n");
    return(-1);
  }

  if ( (w & E_BIT12)  == E_BIT12) {
    printf("homing finished!\n");
    return(0);
  } else {
    //  can this be reached? position finished, no homing error but
    //  homing NOT finished? I guess not..
    return(-5);
  }
}






int epos::moveRelative(long int steps){

  WORD dw[2];
  int n;

  // check, if we are in Profile Position Mode
  if (readOpMode() != E_PROFPOS) {
    if( setOpMode(E_PROFPOS) ) {
      fprintf(stderr, "ERROR: problem at %s; %s line %d\n",
              __func__, __FILE__, __LINE__);
      return(-1);
    }
  }


  // write intended target position
  // firmware 14.1.70
  dw[0] = (WORD) (steps & 0x0000FFFF);
  dw[1] = (WORD) (steps >> 16) ;

  n = WriteObject(0x607A, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }
  checkEPOSerror();

  // switch to relative positioning BY WRITING TO CONTROLWORD, finish
  // possible ongoing operation first!  ->maxon applicattion note:
  // device programming 2.1
  dw[0] = 0x005f;
  dw[1] = 0x0000; // high WORD of DWORD is not used here
  n = WriteObject(0x6040, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }
  checkEPOSerror();


  return(0);
}



int epos::moveAbsolute(long int steps){

  WORD dw[2];
  int n;

#ifdef DEBUG
  printf("-> %s(): will move to %ld (%#010lx)\n", __func__, steps, steps);
#endif

  // check, if we are in Profile Position Mode
  if (readOpMode() != E_PROFPOS) {
    if( setOpMode(E_PROFPOS) ) {
      fprintf(stderr, "ERROR: problem at %s; %s line %d\n",
              __func__, __FILE__, __LINE__);
      return(-1);
    }
  }
#ifdef DEBUG
    printf("-> OpMode is (now) 'Profile Position Mode'. That's OK!\n");
#endif


  // write intended target position, is signed 32bit int
  // firmware 14.1.70
  dw[0] = (WORD) (steps & 0x0000FFFF);
  dw[1] = (WORD) (steps >> 16) ;

#ifdef DEBUG
  printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

  n = WriteObject(0x607A, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }
  checkEPOSerror();

  // switch to absolute positioning, cancel possible ongoing operation
  // first!  ->maxon application note: device programming 2.1
  dw[0] = 0x3f;
  dw[1] = 0x0000; // high WORD of DWORD is not used here
  n = WriteObject(0x6040, 0x00, dw );
  if (n<0) {
    fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
            __func__, n, __FILE__, __LINE__);
    return(-1);
  }
  checkEPOSerror();


  return(0);
}





// monitor device status
int epos::monitorStatus(){
  int  n;
  long int postarget, posactual, veldemand, velactual;
  short curactual;
  WORD status;

  printf("\nEPOS operating figures (note: update here is done AS FAST AS POSSIBLE!):\n");
  int i = 0;
  do {
    i++;
    if  ( (n=readTargetPosition(  &postarget ) ) ){
      printf("ERROR while readActualPosition() [%d]\n", n);
      break;
    }
/*     if  ( (n=readDemandPosition( &posdemand ) ) ){ */
/*       printf("ERROR while readDemandPosition() [%d]\n", n); */
/*       break; */
/*     } */
    if  ( (n=readActualPosition(  &posactual ) ) ){
      printf("ERROR while readActualPosition() [%d]\n", n);
      break;
    }
    if  ( (n=readDemandVelocity(  &veldemand ) ) ){
      printf("ERROR while readDemandVelocity() [%d]\n", n);
      break;
    }
    if  ( (n=readActualVelocity(  &velactual ) ) ){
      printf("ERROR while readActualVelicity() [%d]\n", n);
      break;
    }
    if  ( (n=readActualCurrent(  &curactual ) ) ){
      printf("ERROR while readActualCurrent() [%d]\n", n);
      break;
    }

    printf("\rEPOS: pos=%+10ld |%+10ld (%ld to go); v= %+4ld | %+4ld[rpm]; I=%+4dmA",
           postarget, posactual,postarget-posactual,
           veldemand, velactual, curactual);
    fflush(stdout);

    readStatusword(&status);
  } while ( ( status & E_BIT10) != E_BIT10) ; // bit 10 says: target reached!

  // update values a last time to get a nicer output:
  i++;
  if  ( (n=readTargetPosition(  &postarget ) ) ){
    printf("ERROR while readActualPosition() [%d]\n", n);

  }
  if  ( (n=readActualPosition(  &posactual ) ) ){
    printf("ERROR while readActualPosition() [%d]\n", n);
  }
  if  ( (n=readDemandVelocity(  &veldemand ) ) ){
    printf("ERROR while readDemandVelocity() [%d]\n", n);
  }
  if  ( (n=readActualVelocity(  &velactual ) ) ){
    printf("ERROR while readActualVelicity() [%d]\n", n);
  }
  if  ( (n=readActualCurrent(  &curactual ) ) ){
    printf("ERROR while readActualCurrent() [%d]\n", n);
  }

  printf("\r%d EPOS: pos=%+10ld |%+10ld (%ld to go); v= %+4ld | %+4ld[rpm]; I=%+4dmA\n",
         i, postarget, posactual,postarget-posactual,
         veldemand, velactual, curactual);
  printf("target reached\n");

  return(0);
}



int epos::monitorHomingStatus(){
  int  n;
  long int posactual, velactual;
  short curactual;
  WORD status = 0x0;

  printf("\nEPOS operating figures (note: update here is done AS FAST AS POSSIBLE!):\n");
  int i = 0;
  do {
    i++;
    if  ( (n=readActualPosition(  &posactual ) ) ){
      printf("ERROR while readActualPosition() [%d]\n", n);
      break;
    }
    if  ( (n=readActualVelocity(  &velactual ) ) ){
      printf("ERROR while readActualVelicity() [%d]\n", n);
      break;
    }
    if  ( (n=readActualCurrent(  &curactual ) ) ){
      printf("ERROR while readActualCurrent() [%d]\n", n);
      break;
    }

    readStatusword(&status);


    printf("\r%d EPOS: pos=%+10ld; v =  %+4ldrpm I=%+3dmA status = %#06x ",
           i,  posactual, velactual, curactual, status);

    fflush(stdout);

    readStatusword(&status);

    if ( (status & E_BIT13) == E_BIT13) {
      printf("\aHOMING ERROR!\n");
      return(-2);
    }

  } while (
           ( ( status & E_BIT10) != E_BIT10)
           && ( (status & E_BIT12) != E_BIT12)
           );
  // bit 10 says: target reached!, bit 12: homing attained
  //printEPOSstatusword(status);

  i++;
  if  ( (n=readActualPosition(  &posactual ) ) ){
    printf("ERROR while readActualPosition() [%d]\n", n);
  }
  if  ( (n=readActualVelocity(  &velactual ) ) ){
    printf("ERROR while readActualVelicity() [%d]\n", n);
  }
  if  ( (n=readActualCurrent(  &curactual ) ) ){
    printf("ERROR while readActualCurrent() [%d]\n", n);
  }

  readStatusword(&status);


  printf("\r%d EPOS: pos=%+10ld; v =  %+4ldrpm I=%+3dmA status = %#06x\n",
         i,  posactual, velactual, curactual, status);
  printf("homing finished! Position should now be '0'\n");

  return(0);
}






/* waits for positoning to finish, argument is timeout in
   seconds. give timeout==0 to disable timeout */
int epos::waitForTarget(unsigned int t){

  WORD status;
  unsigned int i = 0, st= (unsigned int)1e4;

  do {
    if (t != 0){ // use timeout?
      if(++i > t*1e2) return(1);
    }
    usleep(st);
    readStatusword(&status);
  } while ( ( status & E_BIT10) != E_BIT10) ; // bit 10 says: target reached!


 return(0);
}




/*
*************************************************************
            check EPOS error code
****************************************************************
*/

/* check the global variable E_error for EPOS error code */
int epos::checkEPOSerror(){

  switch(E_error) {
  case E_NOERR:
    return(0);
    break;
  case E_ONOTEX:
    printf("EPOS responds with error: requested object does not exist!\n");
    break;
  case E_SUBINEX:
    printf("EPOS responds with error: requested subindex does not exist!\n");
    break;
  case E_OUTMEM:
    printf("EPOS responds with error: out of memory!\n");
    break;
  case E_NOACCES:
    printf("EPOS responds with error: unsupported access to an object!\n");
    break;
  case E_WRITEONLY:
    printf("EPOS responds with error: attempt to read a write-only object!\n");
    break;
  case E_READONLY:
    printf("EPOS responds with error: attempt to write a read-only object!\n");
    break;
  case E_PARAMINCOMP:
    printf("EPOS responds with error: general parameter incompatibility!\n");
    break;
  case E_INTINCOMP:
    printf("EPOS responds with error: general internal incompatibility in the device!\n");
    break;
  case E_HWERR:
    printf("EPOS responds with error: access failed due to an HARDWARE ERROR!\n");
    break;
  case E_PRAGNEX:
    printf("EPOS responds with error: value range of parameter exeeded!\n");
    break;
  case E_PARHIGH:
    printf("EPOS responds with error: value of parameter written is too high!\n");
    break;
  case E_PARLOW:
    printf("EPOS responds with error: value of parameter written is too low!\n");
    break;
  case E_PARREL:
    printf("EPOS responds with error: maximum value is less than minimum value!\n");
    break;
  case E_NMTSTATE:
    printf("EPOS responds with error: wrong NMT state!\n");
    break;
  case E_RS232:
    printf("EPOS responds with error: rs232 command illegeal!\n");
    break;
  case E_PASSWD:
    printf("EPOS responds with error: password incorrect!\n");
    break;
  case E_NSERV:
    printf("EPOS responds with error: device not in service mode!\n");
    break;
  case E_NODEID:
    printf("EPOS responds with error: error in Node-ID!\n");
    break;
  default:
    fprintf(stderr, "EPOS responds with error: unknown EPOS error code: %x\n",
            E_error);
    break;
  }
  return(-1);
}



/*
*************************************************************
            basic I/O functions
****************************************************************
*/


/*  write a single BYTE to EPOS */
int epos::writeBYTE(const BYTE *c){

#ifdef DDEBUG
    printf("sending %#04x \n", *c);
#endif
  if (  write(ep, c, 1) < 0 ) {
    perror("write ");
    return(-1);
  }
  return(0);
}



/*  write a single WORD to EPOS */
int epos::writeWORD(const WORD *w){

#ifdef DDEBUG
  printf("sending %#06x \n", *w);
#endif

  if (  write(ep, w, 2) < 0  ) {
    perror("write ");
    return(-1);
  }
  return(0);
}



/*  read a single BYTE from EPOS, timeout implemented */
int epos::readBYTE(BYTE *c){

  int i,n;

  for( i=0; i< NTRY; i++ ) {
    n = read(ep, c, 1);
    int errsv = errno;
    if ( n < 0 && errsv != EAGAIN) {
      perror("read ");
      return(-2);
    }
    if (n > 0) {
#ifdef DDEBUG
      printf("<< receiving: %#04x\n", *c);
#endif
      return(0);
    }
    else {
      if (gMarker==0){
        printf("/\b");
        fflush(stdout);
        gMarker=1;
      }
      else {
        printf("\\\b");
        fflush(stdout);
        gMarker = 0;
      }
      usleep(TRYSLEEP); /* sleep 100ms; EPOS gives timeout after 500ms*/
    }
  }

  // timeout
  return(-1);
}



/*  read a single WORD from EPOS, timeout implemented */
int epos::readWORD(WORD *w){

  int i,n;

  for( i=0; i< NTRY; i++ ) {
    n = read(ep, w, sizeof(WORD) );
    int errsv = errno;
    if ( n < 0 && errsv != EAGAIN) {
      perror("read ");
      return(-2);
    }
    if (n > 0) {
#ifdef DDEBUG
      printf("<<  receiving: %#04x\n", *w);
#endif
      return(0);
    }
    else {
      if (gMarker==0){
        printf("/\b");
        fflush(stdout);
        gMarker = 1;
      }
      else {
        printf("\\\b");
        fflush(stdout);
        gMarker = 0;
      }
      usleep(TRYSLEEP); /* sleep 100ms; EPOS gives timeout after 500ms*/
    }
  }
  // timeout
  return(-1);
}




/* copied from EPOS Communication Guide, p.8 */
WORD epos::CalcFieldCRC(const WORD *pDataArray, WORD numberOfWords)
{
  WORD shifter, c;
  WORD carry;
  WORD CRC = 0;


  //Calculate pDataArray Word by Word
  while(numberOfWords--)
    {
      shifter = 0x8000;                 //Initialize BitX to Bit15
      c = *pDataArray++;                //Copy next DataWord to c
      do
        {
          carry = CRC & 0x8000;    //Check if Bit15 of CRC is set
          CRC <<= 1;               //CRC = CRC * 2
          if(c & shifter) CRC++;   //CRC = CRC + 1, if BitX is set in c
          if(carry) CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true
          shifter >>= 1;           //Set BitX to next lower Bit,
                                   //shifter = shifter/2
        } while(shifter);
    }

  //printf("checksum == %#06x\n", CRC);
  return CRC;
}





/*  send command to EPOS, taking care of all neccessary 'ack' and
   checksum tests*/
int epos::sendCom(WORD *frame){

  BYTE c = 0x00;
  short i, len;
  int n;

  // need LSB of header WORD, contains (len-1). Complete Frame is
  // (len-1) +3 WORDS long
  len = ( ( frame[0] & 0x00FF) ) +3 ;
  /*
    printf("frame[0] = %#x; shifted = %#x; framelength = %d\n",
          frame[0], (frame[0] & 0x00FF),  len);
  */

  // add checksum to frame
  frame[len-1] =  CalcFieldCRC(frame, len);

#ifdef DEBUG
  printf(">> ");
  for (i=0; i<len; ++i){
    printf( "%#06x ", frame[i] );
  }
  printf("\n");
#endif

  /* sending to EPOS */
  //send header:
  c = (frame[0] & 0xFF00) >> 8 ;  //LSB
  if ( writeBYTE(&c) ) perror("writeByte");

  c = 0x77; // 0x77 is not used by EPOS
  // wait for "Ready Ack 'O'"
  if ( (n=readBYTE(&c))  < 0  ) fprintf(stderr, "readBYTE() returnd %d at %s, line %d\n", n, __func__, __LINE__);

  if (c != E_OK) {
    if (c == 0x77) {
      fprintf(stderr, "ERROR: no reply from EPOS recieved, is it on-line?\n");
      exit(2);
    }
    printf("EPOS not ready, reply was: %#04x\n", c);
    return(-1);

  }

  c = (frame[0] & 0x00FF)  ;  //MSB
  if ( writeBYTE(&c) ) perror("writeBYTE");

  // header done, data + CRC will follow
  for (i=1; i<len; i++) {
    if ( writeWORD(frame+i) ) perror("writeWORD");
  }

  // wait for "End Ack 'O'"
  if ( readBYTE(&c)  < 0  ) perror("readBYTE");
  if (c != E_OK) {
    printf("EPOS says: CRCerror!\n");
    return(-1);
  }
  return(0);
}





/*!  int readAnswer(WORD **ptr) - read an answer frame from EPOS

\param pointer on the EPOS object.
\param ptr WORD **ptr; pointer address where answer frame is placed.

\retval >0 number of WORDs recieved from EPOS. ptr points now to
     answer frame.
\retval <0 failure; ptr points to NULL. Global E_error is also set to
     returnd EPOS ErrorCode

  */
int epos::readAnswer(WORD **ptr){

  int i;
  BYTE c;
  WORD first=0x00 , w,  crc, framelen;
  static WORD *ans;

  E_error = 0x00;

  /*
  printf("******** sub: ptr= %p  &ptr = %p\n", ptr, &ptr);
  printf("******** sub: ans   = %p  &ans = %p\n", ans, &ans);
  */

  readBYTE(&c);
  first = (0xFF00 & c) << 8;
  //printf("first answer: %#04x; first: %#06x\n", c, first);


  if (c != E_ANS) {
    fprintf(stderr, "EPOS says: %#04x. This is no answer frame! \n", c);
    ptr = NULL;
    return(-1);
  }
  c = E_OK;
  writeBYTE(&c);

  // here is the (len-1) value coming
  readBYTE(&c);

  first = (0x00FF & c) ;
  //printf("second answer: %#04x; first: %#06x\n", c, first);

  framelen = c + 3;

  checkPtr( ans = (WORD*)malloc( framelen * sizeof(WORD)) );

  ans[0] = first;

  for(i=1; i<framelen; i++){
    readWORD(&w);
    ans[i] = w;
  }
#ifdef DEBUG
  printf("\n<< ");
  for(i=0; i<(framelen); i++){
    printf("%#06x ", ans[i]);
  }
  printf("\n"); fflush(stdout);

#endif

  // compute checksum
  crc = ans[framelen-1];
#ifdef DDEBUG
  printf("got this CRC: %#06x\n", crc);
#endif
  ans[framelen-1] =  0x0000;
  ans[framelen-1] = CalcFieldCRC(ans, framelen);


  if (crc == ans[framelen-1]) {
    c = E_OK;
    writeBYTE(&c);
#ifdef DEBUG
    printf("CRC test OK!\n");
#endif
  }
  else {
    c = E_FAIL;
    writeBYTE(&c);
    fprintf(stderr, "CRC test FAILED!\n");
    ptr = NULL;
    return(-1);
  }


  /* check for error code */

  /* just to get the bit's at the right place...*/
  //ans[1] = 0x1234; ans[2] = 0xABCD;
  E_error = ans[1] | (ans[2] << 16) ;
  //printf(" xxxxxxx ->%#010x<-\n", E_error);




  *ptr = ans;
  /*
  printf("******** sub: ptr= %p  &ptr = %p\n", ptr, &ptr);
  printf("******** sub: ans   = %p  &ans = %p\n", ans, &ans);
  */
  return(framelen);
}






int epos::ReadObject(WORD index, BYTE subindex, WORD **ptr ){

  WORD frame[4];
  int n;

  frame[0] = 0x1001; // fixed, ReadObject, (len-1) == 1
  frame[1] = index;
  frame[2] = (0x0000 | subindex); /* high BYTE: 0x00(Node-ID == 0)
                                      low BYTE: subindex */
  frame[3] = 0x000; // ZERO word, will be filled with checksum

  if( (n = sendCom(frame)) < 0){
    fprintf(stderr, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return(-1);
  }

  // read response
  return( readAnswer(ptr) );

}

#if 0
/*! NOT USED IN libEPOS so far -> untested!
 */
static int InitiateSegmentedRead(WORD index, BYTE subindex ){

  WORD frame[4], **ptr;
  int n=0;

  frame[0] = 0x1201; // fixed, opCode==0x12, (len-1) == 1
  frame[1] = index;
  frame[2] = 0x0000 | subindex; /* high BYTE: 0x00 (Node-ID == 0)
                                    low BYTE: subindex */
  frame[3] = 0x000; // ZERO word, will be filled with checksum

  if( (n = sendCom(frame)) < 0){
    fprintf(stderr, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return(-1);
  }

  // read response
  return( readAnswer(ptr) );  // answer contains only DWORD ErrorCode
                              // here...
}

/*! NOT USED IN libEPOS so far -> untested!



\retval >0  means sucess. Number of WORDs recieved from EPOS will be returned
(*ptr) points to answer frame

\retval <0 means failure: (*ptr) points to NULL

*/
static int SegmentRead(WORD **ptr){

  WORD frame[3];
  int n;

  frame[0] = 0x1400; // fixed, opCode==0x14, (len-1) == 0
  frame[1] = 0x0000; // WHAT IS THE 'TOGGLE' BIT????
  frame[2] = 0x0000;  // ZERO word, will be filled with checksum

  if( (n = sendCom(frame)) < 0){
    fprintf(stderr, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return(-1);
  }


  if ( (n = readAnswer(ptr)) < 0){
    fprintf(stderr, " *** %s: problems with readAns(), return value was %d ***\n ",  __func__, n);
    return(-1);
  }


  return(0);
}
#endif



/*! Low-level function to write an object to EPOS memory. Is called by
 writing libEPOS functions.

\param pointer on the EPOS object.

\param index WORD describing EPOS memory index for writing. See
firmware documentation for valid values

\param subindex BYTE describing EPOS memory subindex for writing. See
firmware documentation for valid values

\param data pointer to WORD array holding the data to be written to EPOS memory

\retval 0 success
\retval -1 error
*/
int epos::WriteObject(WORD index, BYTE subindex, const WORD *data) {

  WORD frame[6];
  WORD *ans = NULL;
  int n;

  frame[0] = 0x1103; // fixed, WriteObject, (len-1) == 3
  frame[1] = index;
  frame[2] = (0x0000 | subindex); /* high BYTE: 0x00(Node-ID == 0)
                                      low BYTE: subindex */
  // data to transmit
  frame[3] = data[0];
  frame[4] = data[1];

  frame[5] = 0x00; // ZERO word, will be filled with checksum

  if( (n = sendCom(frame)) < 0){
    fprintf(stderr, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return(-1);
  }


  // read response
  checkPtr( ans = (WORD*)calloc(3, sizeof(WORD) ) );

  if ( (n = readAnswer(&ans) )  <0 ){
    fprintf(stderr, " *** %s: problems with readAnswer(), return value was %d ***\n ",  __func__, n);
    free(ans);
    return(-1);
  }

  return( checkEPOSerror() );

}






/* compare WORD a with WORD b bitwise */
int epos::bitcmp(WORD a, WORD b){
  if ( (a & b) == b ) return(1);
  else return(0);
}





void epos::checkPtr(const void* ptr){
  if (ptr == NULL){
    fprintf(stderr, "malloc failed!\n");
    exit(-1);
  }
}




