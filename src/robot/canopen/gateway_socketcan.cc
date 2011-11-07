/*
 * gateway_socketcan.cc
 *
 *  Created on: May 14, 2011
 *      Author: ptroja
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <sys/select.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "gateway_socketcan.h"

namespace mrrocpp {
namespace edp {
namespace canopen {

gateway_socketcan::gateway_socketcan(const std::string & _iface) :
	iface(_iface), sock(-1)
{
	if(iface.length() >= IFNAMSIZ) {
		throw se_canopen_error() << reason("name of CAN device too long");
	}
}

gateway_socketcan::~gateway_socketcan()
{
	if (device_opened) close();
}

void gateway_socketcan::open()
{
	/* Create the socket */
	sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (sock == -1) {
		perror("socket()");
		throw se_canopen_error() << reason("failed to create a CAN socket");
	}

	/* Locate the interface you wish to use */
	struct ifreq ifr;
	// FIXME: use strncpy
	strcpy(ifr.ifr_name, iface.c_str());
	::ioctl(sock, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled
	 * with that device's index */

	/* Select that CAN interface, and bind the socket to it. */
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if(::bind(sock, (struct sockaddr*) &addr, sizeof(addr)) == -1) {
		perror("bind()");
		throw se_canopen_error() << reason("failed to bind to a CAN interface");
	}

	device_opened = true;
}

void gateway_socketcan::close()
{
	if(::close(sock) == -1) {
		throw se_canopen_error() << reason("failed to close CAN socket");;
	}

	device_opened = false;
}

canid_t gateway_socketcan::readFromWire(struct can_frame & frame)
{
	// Setup for read timeout
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(sock, &rfds);

	// 500ms timeout
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 500000;

	// Wait for data with timeout
	int ret = select(sock+1, &rfds, NULL, NULL, &tv);

	// Check the result
	if (ret == -1) {
		 throw se_canopen_error() << errno_call("select") << errno_code(errno);
	} else if (ret == 0) {
		 throw se_canopen_error() << reason("timeout reading from CAN interface");
	}

	// Assert, that data comes from the CAN interface
	if (!FD_ISSET(sock, &rfds)) {
		throw se_canopen_error() << reason("CAN interface not ready to read");
	}

    /* read frame */
    if (::read(sock, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("read()");
        BOOST_THROW_EXCEPTION(se_canopen_error() << reason("read from CAN socket failed"));
    }

    return (frame.can_id);
}

void gateway_socketcan::writeToWire(const struct can_frame & frame)
{
	// Setup for write timeout
	fd_set wfds;
	FD_ZERO(&wfds);
	FD_SET(sock, &wfds);

	// 10ms timeout
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 10000;

	// Wait for ready to write with timeout
	int ret = select(sock+1, NULL, &wfds, NULL, &tv);

	// Check the result
	if (ret == -1) {
		 throw se_canopen_error() << errno_call("select") << errno_code(errno);
	} else if (ret == 0) {
		 throw se_canopen_error() << reason("timeout writing to CAN interface");
	}

	// Assert, that data comes from the CAN interface
	if (!FD_ISSET(sock, &wfds)) {
		throw se_canopen_error() << reason("CAN interface not ready to write");
	}

    /* send frame */
    if (::write(sock, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("write()");
        BOOST_THROW_EXCEPTION(se_canopen_error() << reason("write to CAN socket failed"));
    }
}

#define CCS(x)				((x) >> 5)		// client command specifier
#define SCS(x)				((x) >> 5)		// server command specifier

#define TRANSFER_TYPE(x)	((x) & 0x02)	// set if expedited transfer mode
#define	TRANSFER_NORMAL		(0x00)			// flag not set in normal transfer mode
#define	TRANSFER_EXPEDITED	(0x02)			// flag set in expedited transfer mode

#define SIZE_INDICATOR(x)	((x) & 0x01)	// set if data set size is indicated
#define	SIZE_NOT_INDICATED	(0x00)			// flag not set in normal transfer mode
#define	SIZE_INDICATED		(0x01)			// flag set in expedited transfer mode

// number of bytes, which do not contain data;
// valid only in expedited transfer mode AND size indicator flag set
#define BYTES_WITHOUT_DATA(x)	(((x) >> 2) & 0x03)

void gateway_socketcan::handleCanOpenMgmt(const struct can_frame & frame)
{

}

unsigned int gateway_socketcan::ReadObject(WORD *ans, unsigned int ans_len, uint8_t nodeId, WORD index, BYTE subindex)
{
	struct can_frame frame;

	frame.can_id = 0x600 + nodeId;
	frame.can_dlc = 8;
	frame.data[0] = 0x40;	// Initiate Domain Upload, client => server
	frame.data[1] = (index & 0xFF); // index high BYTE
	frame.data[2] = (index >> 8);   // index low BYTE
	frame.data[3] = subindex;
	frame.data[4] = 0;	// don't care, should be zero
	frame.data[5] = 0;
	frame.data[6] = 0;
	frame.data[7] = 0;

	writeToWire(frame);

	// wait for reply
	while(readFromWire(frame) != (0x580 + nodeId)) {
		handleCanOpenMgmt(frame);
	}

	/*
	 * SCS: server command specifier replies
	 * 011 <= Initiate Domain Download (WriteObject)
	 * 001 <= Download Domain Segment
	 * 010 <= Initiate Domain Upload (ReadObject)
	 * 000 <= Upload Domain Segment
	 */

	// check for Abort Transfer message
	if (SCS(frame.data[0]) == 4) {
		E_error = *((uint32_t*) &frame.data[4]);
		// ??? do we also need to check the reply address (index, subindex)?
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("SDO transfer aborted"));
	} else {
		E_error = 0;
	}

	// check the reply SCS
	if (SCS(frame.data[0]) != 2) {
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("unexpected SCS (server command specifier) received"));
	}

	// check the reply "expedited" field
	if (TRANSFER_TYPE(frame.data[0]) != TRANSFER_EXPEDITED) {
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("expedited reply message expected"));
	}

	// check the size indicator
	if (SIZE_INDICATOR(frame.data[0]) != SIZE_INDICATED) {
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("object data size not indicated in reply"));
	}

	// adress (index, subindex) of the reply object
	WORD r_index = (frame.data[2] << 8) | (frame.data[1]);
	BYTE r_subindex = (frame.data[3]);

	if ((r_index != index) || (r_subindex != subindex)) {
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("unexpected reply object address (index, subindex)"));
	}

	switch (BYTES_WITHOUT_DATA(frame.data[0])) {
		// answer is 32 bit long
		case 0:
			*((DWORD *) &ans[3]) = *((DWORD *) &frame.data[4]);
			return 4;
		// answer is 16 bit long
		case 2:
			ans[3] = *((WORD *) &frame.data[4]);
			break;
		// answer is 8 bit long
		case 3:
			ans[3] = frame.data[4];
			break;
		default:
			BOOST_THROW_EXCEPTION(se_canopen_error() << reason("unsupported reply data size"));
			break;
	}

	return (4 - BYTES_WITHOUT_DATA(frame.data[0]));
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

void gateway_socketcan::WriteObject(uint8_t nodeId, WORD index, BYTE subindex, uint32_t data)
{
	struct can_frame frame;

	frame.can_id = 0x600 + nodeId;
	frame.can_dlc = 8;
	frame.data[0] = 0x22;	// Initiate Domain Download, expedited transfer, client => server,
	frame.data[1] = (index & 0xFF); // index high BYTE
	frame.data[2] = (index >> 8);   // index low BYTE
	frame.data[3] = subindex;

	*((uint32_t *) &frame.data[4]) = data;

	writeToWire(frame);

	// wait for reply
	while(readFromWire(frame) != (0x580 + nodeId)) {
		handleCanOpenMgmt(frame);
	}

	/*
	 * SCS: server command specifier replies
	 * 011 <= Initiate Domain Download (WriteObject)
	 * 001 <= Download Domain Segment
	 * 010 <= Initiate Domain Upload (ReadObject)
	 * 000 <= Upload Domain Segment
	 */

	// check for Abort Transfer message
	if (SCS(frame.data[0]) == 4) {
		E_error = *((uint32_t*) &frame.data[4]);
		// ??? do we also need to check the reply address (index, subindex)?
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("SDO transfer aborted"));
	} else {
		E_error = 0;
	}

	// check the reply SCS
	if (SCS(frame.data[0]) != 3) {
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("unexpected SCS (server command specifier) received"));
	}

	// adress (index, subindex) of the reply object
	WORD r_index = (frame.data[2] << 8) | (frame.data[1]);
	BYTE r_subindex = (frame.data[3]);

	if ((r_index != index) || (r_subindex != subindex)) {
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("unexpected reply object address (index, subindex)"));
	}
}

void gateway_socketcan::InitiateSementedWrite(uint8_t nodeId, WORD index, BYTE subindex, DWORD ObjectLength)
{
	struct can_frame frame;
	frame.can_dlc = 8;
}

void gateway_socketcan::SegmentedWrite(uint8_t nodeId, BYTE * ptr, std::size_t len)
{
	if (len > 63) {
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("Segmented write of > 63 bytes not allowed"));
	}

	struct can_frame frame;

	frame.can_id = 0x600 + nodeId;
	frame.can_dlc = 8;

	frame.data[0] = 0x40;	// Initiate Domain Upload, client => server
}

void gateway_socketcan::SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier)
{
	struct can_frame frame;

	frame.can_id = 0x0000;
	frame.can_dlc = 2;
	frame.data[0] = (uint8_t) CmdSpecifier;
	frame.data[1] = nodeId;

	writeToWire(frame);
}

void gateway_socketcan::SendCANFrame(WORD Identifier, WORD Length, const BYTE Data[8])
{
	if (Length > 8) {
		BOOST_THROW_EXCEPTION(se_canopen_error() << reason("Segmented write of > 63 bytes not allowed"));
	}

	struct can_frame frame;

	frame.can_id = Identifier;
	frame.can_dlc = Length;
	memcpy(frame.data, Data, Length);

	writeToWire(frame);
}


} /* namespace canopen */
} /* namespace edp */
} /* namespace mrrocpp */
