/*
 * epos_access_rs232.cc
 *
 *  Created on: Jan 18, 2011
 *      Author: ptroja
 */

#include <cstring>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <cerrno>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/select.h>

#include "epos_access_rs232.h"

/*! \brief try NTRY times to read one byte from EPOS, then give up */
#define NTRY	5

/*! \brief wait TRYSLEEP usec between read() from EPOS, if no data available */
#define TRYSLEEP  ((unsigned int) 1e5)

namespace mrrocpp {
namespace edp {
namespace epos {

epos_access_rs232::epos_access_rs232(const std::string & _device) :
	device(_device),
	ep(-1), gMarker(0)
{
}

epos_access_rs232::~epos_access_rs232()
{
	if (device_opened) close();
}

void epos_access_rs232::open()
{
	/* EPOS transfer format is:
	 1 start bit
	 8 data bits
	 no parity
	 1 stop bit
	 */

	if (ep >= 0)
		throw epos_error() << reason("serial port already opened");

	for (int i = 0; i < 5; i++) {
		if ((ep = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) >= 0)
			break;
		sleep(1);
	}

	if (ep == -1) {
		perror("open()");
		throw epos_error() << reason("open serial port");
	}

	if (tcgetattr(ep, &options) < 0) {
		perror("tcgetattr");
	}

	memset(&options, 0, sizeof(options));

	options.c_cflag |= CS8; //8 bits per byte
	options.c_cflag |= CLOCAL | CREAD;

	// Speed defaults to 115200bps
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	tcflush(ep, TCIFLUSH);

	if (tcsetattr(ep, TCSANOW, &options) < 0) {
		perror("tcsetattr");
	}

//	fcntl(ep, F_SETFL, FNDELAY);

	device_opened = true;
}

void epos_access_rs232::close()
{
	if(ep >= 0) {
		if (::close(ep) != 0) {
			perror("close()");
			throw epos_error() << reason("serial port already opened");
		}
	}

	device_opened = false;
}

/* EPOS codes */

#define E_OK      0x4f  ///< EPOS answer code for <em>all fine</em>
#define E_FAIL    0x46  ///< EPOS answer code to indicate a <em>failure</em>
#define E_ANS     0x00  ///< EPOS code to indicate an answer <em>frame</em>

unsigned int epos_access_rs232::readAnswer(WORD *ans, unsigned int ans_len)
{
	E_error = 0x00;

	BYTE c = readBYTE(); // this is op-code, discard.
//	first = (0xFF00 & c) << 8;
	//printf("first answer: %#04x; first: %#06x\n", c, first);

	if (c != E_ANS) {
		throw epos_error() << reason("EPOS says: this is no answer frame!"); // TODO: , c);
	}
	c = E_OK;
	writeBYTE(c);

	// here is the (len-1) value coming
	c = readBYTE();

	WORD first = (0x00FF & c);
	//printf("second answer: %#04x; first: %#06x\n", c, first);

	WORD framelen = c + 3;

	if (ans_len < framelen) {
		throw epos_error() << reason("output buffer to short for a message");
	}

	ans[0] = first;

	for (int i = 1; i < framelen; i++) {
		ans[i] = readWORD();
	}
#ifdef DEBUG
	printf("\n<< ");
	for(i=0; i<(framelen); i++) {
		printf("%#06x ", ans[i]);
	}
	printf("\n"); fflush(stdout);
#endif

	// compute checksum
	WORD crc = ans[framelen - 1];
#ifdef DDEBUG
	printf("got this CRC: %#06x\n", crc);
#endif
	ans[framelen - 1] = 0x0000;
	{
		WORD anstab[framelen];
		for (int i = 0; i < framelen; i++) {
			anstab[i] = ans[i];
		}
		ans[framelen - 1] = CalcFieldCRC(anstab, framelen);
	}

	if (crc == ans[framelen - 1]) {
		c = E_OK;
		writeBYTE(c);
#ifdef DEBUG
		printf("CRC test OK!\n");
#endif
	} else {
		writeBYTE(E_FAIL);
		throw epos_error() << reason("CRC test FAILED");
	}

	/* check for error code */

	/* just to get the bit's at the right place...*/
	//ans[1] = 0x1234; ans[2] = 0xABCD;
	E_error = ans[1] | (ans[2] << 16);
	//printf(" xxxxxxx ->%#010x<-\n", E_error);

	/*
	 printf("******** sub: ptr= %p  &ptr = %p\n", ptr, &ptr);
	 printf("******** sub: ans   = %p  &ans = %p\n", ans, &ans);
	 */
	return framelen;
}

void epos_access_rs232::sendCommand(WORD *frame)
{
	// RS232 connection version
	if (ep >=0) {
		// need LSB of header WORD, contains (len-1). Complete Frame is
		// (len-1) +3 WORDS long
		unsigned short len = ((frame[0] & 0x00FF)) + 3;
		/*
		 printf("frame[0] = %#x; shifted = %#x; framelength = %d\n",
		 frame[0], (frame[0] & 0x00FF),  len);
		 */

		// add checksum to frame
		frame[len - 1] = CalcFieldCRC(frame, len);

	#ifdef DEBUG
		printf(">> ");
		for (int i=0; i<len; ++i) {
			printf( "%#06x ", frame[i] );
		}
		printf("\n");
	#endif

		/* sending to EPOS */
		BYTE c;

		//send header:
		c = (frame[0] & 0xFF00) >> 8; //LSB
		writeBYTE(c);

		// wait for "Ready Ack 'O'"
		c = readBYTE();

		if (c != E_OK) {
			throw epos_error() << reason("EPOS not ready"); //TODO: reply was: %#04x, c;
		}

		c = (frame[0] & 0x00FF); //MSB
		writeBYTE(c);

		// header done, data + CRC will follow
		for (int i = 1; i < len; i++) {
			writeWORD(frame[i]);
		}

		// wait for "End Ack 'O'"
		c = readBYTE();
		if (c != E_OK) {
			throw epos_error() << reason("EPOS says: CRCerror!");
		}
	}
}

/*
 *************************************************************
 basic I/O functions
 ****************************************************************
 */

/*  write a single BYTE to EPOS */
void epos_access_rs232::writeBYTE(BYTE c)
{
#ifdef DDEBUG
	printf("sending %#04x \n", c);
#endif

	if (write(ep, &c, 1) < 0) {
		throw epos_error() << errno_call("write") << errno_code(errno);
	}
}

/*  write a single WORD to EPOS */
void epos_access_rs232::writeWORD(WORD w)
{
#ifdef DDEBUG
	printf("sending %#06x \n", w);
#endif

	if (write(ep, &w, 2) < 0) {
		throw epos_error() << errno_call("write") << errno_code(errno);
	}
}

/*  read a single BYTE from EPOS, timeout implemented */
BYTE epos_access_rs232::readBYTE()
{
	for (int i = 0; i < NTRY; i++) {

		// read-ready file descriptor set
		fd_set rfds;

		// zero the set
		FD_ZERO(&rfds);

		// add ep to the set
		FD_SET(ep, &rfds);

		// EPOS gives timeout after 500ms timeout 500ms
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = TRYSLEEP;

		int s = select(ep+1, &rfds, NULL, NULL, &tv);
		if(s < 0) {
			throw epos_error() << errno_call("select") << errno_code(errno);
		} else if (s == 0) {
			continue;
		}

		if(!FD_ISSET(ep, &rfds)) {
			throw epos_error() << reason("EPOS filedescriptor not in read-ready set");
		}

		BYTE c;
		int n = read(ep, &c, 1);
		int errsv = errno;
		if (n < 0 && errsv != EAGAIN) {
			throw epos_error() << errno_call("read") << errno_code(errsv);
		}
		if (n > 0) {
#ifdef DDEBUG
			printf("<< receiving: %#04x\n", *c);
#endif
			return c;
		} else {
			if (gMarker == 0) {
				printf("/\b");
				fflush(stdout);
				gMarker = 1;
			} else {
				printf("\\\b");
				fflush(stdout);
				gMarker = 0;
			}
		}
	}

	// timeout
	throw epos_error() << reason("read timeout");
}

/*  read a single WORD from EPOS, timeout implemented */
WORD epos_access_rs232::readWORD()
{
	for (int i = 0; i < NTRY; i++) {

		// read-ready file descriptor set
		fd_set rfds;

		// zero the set
		FD_ZERO(&rfds);

		// add ep to the set
		FD_SET(ep, &rfds);

		// EPOS gives timeout after 500ms timeout 500ms
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = TRYSLEEP;

		int s = select(ep+1, &rfds, NULL, NULL, &tv);
		if(s < 0) {
			throw epos_error() << errno_call("select") << errno_code(errno);
		} else if (s == 0) {
			continue;
		}

		if(!FD_ISSET(ep, &rfds)) {
			throw epos_error() << reason("EPOS filedescriptor not in read-ready set");
		}

		WORD w;
		int n = read(ep, &w, sizeof(WORD));
		int errsv = errno;
		if (n < 0 && errsv != EAGAIN) {
			throw epos_error() << errno_call("read") << errno_code(errsv);
		}
		if (n > 0) {
#ifdef DDEBUG
			printf("<<  receiving: %#04x\n", *w);
#endif
			return w;
		} else {
			if (gMarker == 0) {
				printf("/\b");
				fflush(stdout);
				gMarker = 1;
			} else {
				printf("\\\b");
				fflush(stdout);
				gMarker = 0;
			}
		}
	}

	// timeout
	throw epos_error() << reason("read timeout");
}

void epos_access_rs232::SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier)
{
	WORD frame[4];

	frame[0] = (2 << 8) | 0x0E; // (len << 8) | OpCode
	frame[1] = nodeId;
	frame[2] = CmdSpecifier;
	frame[3] = 0x00; // ZERO word, will be filled with checksum

	sendCommand(frame);

	// Remark: no response with RS232
}

void epos_access_rs232::SendCANFrame(WORD Identifier, WORD Length, BYTE Data[8])
{
	WORD frame[4];

	frame[0] = (6 << 8) | 0x20; // (len << 8) | OpCode
	frame[1] = Identifier;
	frame[2] = Length;
	frame[3] = (Data[1] << 8)| Data[0];
	frame[4] = (Data[3] << 8)| Data[2];
	frame[5] = (Data[5] << 8)| Data[4];
	frame[6] = (Data[7] << 8)| Data[6];
	frame[7] = 0x00; // ZERO word, will be filled with checksum

	sendCommand(frame);

	// Remark: no response with RS232
}

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */
