/*
 * gateway_usb.cc
 *
 *  Created on: Jan 18, 2011
 *      Author: ptroja
 */

#include <cstdio>

#include "gateway_epos_usb.h"

namespace mrrocpp {
namespace edp {
namespace canopen {

gateway_epos_usb::gateway_epos_usb(int _vendor, int _product, unsigned int _index) :
	vendor(_vendor), product(_product), index(_index)
{
	if (ftdi_init(&ftdic) < 0) {
		fprintf(stderr, "ftdi_init failed\n");
	}
	// Set the timeouts to 1 sec (the libftdi defaults to 5 sec)
	ftdic.usb_read_timeout = 1000;
	ftdic.usb_write_timeout = 1000;
}

gateway_epos_usb::~gateway_epos_usb()
{
	if (device_opened) close();
	ftdi_deinit(&ftdic);
}

void gateway_epos_usb::open()
{
	//! FTDI calls return code
	int ret;

	if ((ret = ftdi_usb_open(&ftdic, vendor, product)) < 0) {
		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	}

	//! reset FTDI device
	if ((ret = ftdi_usb_reset(&ftdic)) < 0) {
		fprintf(stderr, "unable to reset ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	}

	if ((ret = ftdi_set_line_property(&ftdic, BITS_8, STOP_BIT_1, NONE)) < 0) {
		fprintf(stderr, "unable to set ftdi line property: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
        throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
    }

	//! set flow control
	if ((ret = ftdi_setflowctrl(&ftdic, SIO_DISABLE_FLOW_CTRL)) < 0) {
	//if ((ret = ftdi_setflowctrl(&ftdic, SIO_RTS_CTS_HS)) < 0) {
		fprintf(stderr, "unable to set ftdi flow control: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
        throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
    }

	//! set latency timer
	if ((ret = ftdi_set_latency_timer(&ftdic, 1)) < 0) {
		fprintf(stderr, "unable to set ftdi latency timer: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	}

	//! set baud rate
	if ((ret = ftdi_set_baudrate(&ftdic, 1000000)) < 0) {
		fprintf(stderr, "unable to set ftdi baudrate: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	}
#if 0
	//! set write data chunk size
	if ((ret = ftdi_write_data_set_chunksize(&ftdic, 512)) < 0) {
		fprintf(stderr, "unable to set ftdi write chunksize: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	}

	//! set read data chunk size
	if ((ret = ftdi_read_data_set_chunksize(&ftdic, 512)) < 0) {
		fprintf(stderr, "unable to set ftdi read chunksize: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	}
#endif

	device_opened = true;
}

void gateway_epos_usb::close()
{
	if(ftdi_usb_close(&ftdic)) {
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	}

	device_opened = false;
}

#define DLE	(0x90)	// Data Link Escape
#define STX	(0x02)	// Start of Text

unsigned int gateway_epos_usb::readAnswer(WORD *ans, unsigned int ans_len)
{
	// reply buffer
	uint8_t buf[512];

	E_error = 0x00;

	// FTDI return code
	int wait_cnt = 500; // for Store() command the 352 was fine
	int ret = 0;

	while ((ret == 0) && (wait_cnt > 0)) {
		ret = ftdi_read_data(&ftdic, buf, sizeof(buf));
		wait_cnt--;
	}

	if (ret < 0) {
		fprintf(stderr, "ftdi device read failed (%d): (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	} else if (ret == 0) {
		throw fe_canopen_error() << reason("no data returned");
	}

	if (debug) {
		printf("<< ");
		for(int i=0; i<ret; i++) {
			printf("0x%02X ", buf[i]);
		}
		printf("\n");
	}

	// check DLE
	if (buf[0] != DLE) {
		throw fe_canopen_error() << reason("Datagram error (DLE expected)");
	}

	// check STX
	if (buf[1] != STX) {
		throw fe_canopen_error() << reason("Datagram error (STX expected)");
	}

	// read OpCode
	if (buf[2] != 0x00) {
		throw fe_canopen_error() << reason("Datagram error (0x00 Answer OpCode expected)");
	}

	// frame length
	WORD framelen = buf[3];

	if (ans_len < framelen) {
		throw fe_canopen_error() << reason("output buffer to short for a message");
	}

	ans[0] = (framelen << 8);

	int idx = 4; // data buffer index

	for (int i = 1; i <= framelen+1; ++i) {
		// LSB
		if (buf[idx] == DLE) idx++;
		ans[i] = buf[idx++];

		// MSB
		if (buf[idx] == DLE) idx++;
		ans[i] |= (buf[idx++] << 8);
	}

#ifdef DDEBUG
	printf("<< ");
	for(int i=0; i<= framelen+1; i++) {
		printf("%04x ", ans[i]);
	}
	printf("\n"); fflush(stdout);
#endif

	// compute checksum
	WORD crc = ans[framelen+1];
#ifdef DDEBUG
	printf("got this CRC: 0x%04x\n", crc);
#endif
	ans[framelen + 1] = 0x0000;
	ans[framelen + 1] = CalcFieldCRC(ans, framelen+2);

	if (crc == ans[framelen + 1]) {
#ifdef DEBUG
		printf("CRC test OK!\n");
#endif
	} else {
		fprintf(stderr, "CRC: %04x != %04x\n", crc, ans[framelen + 1]);
		throw fe_canopen_error() << reason("CRC test FAILED");
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

void gateway_epos_usb::sendCommand(WORD *frame)
{
	// USB connection version

	// need LSB of header WORD, contains (len-1). Complete Frame is
	// (len-1) + 3 WORDS long
	unsigned short len = (frame[0] >> 8) + 2;

	// add checksum to frame
	frame[len - 1] = CalcFieldCRC(frame, len);

	// calculate number of 'DLE' characters
	unsigned int stuffed = 0;
	for(int i = 1; i < len-1; i++) {
		if ((frame[i] & 0x00FF) == 0x90) {
			stuffed++;
		}
		if (((frame[i] & 0xFF00) >> 8) == 0x90) {
			stuffed++;
		}
	}

	// message datagram to write
	uint8_t datagram[2+2+len*2+stuffed+2]; // DLE,STX,OpCode,Len,len*2+stuffed,CRC
	unsigned int idx = 0; // current datagram byte index

	datagram[idx++] = 0x90;	// DLE: Data Link Escape
	datagram[idx++] = 0x02; // STX: Start of Text

	for(int i = 0; i < len; ++i) {
		// characeter to write
		uint8_t c;

		// LSB
		c = frame[i] & 0x00FF;
		if (c == DLE) {
			datagram[idx++] = DLE;
		}
		datagram[idx++] = c;

		// MSB
		c = (frame[i] >> 8);
		if (c == DLE) {
			datagram[idx++] = DLE;
		}
		datagram[idx++] = c;
	}

	if (debug > 0) {
		printf(">> ");
		for (unsigned int i=0; i<idx; ++i) {
			printf( "0x%02X ", datagram[i] );
		}
		printf("\n");
	}

	int w = ftdi_write_data(&ftdic, datagram, idx);

	if (w != (int) idx) {
		fprintf(stderr, "ftdi device write failed (%d/%d chars written): (%s)\n", w, idx, ftdi_get_error_string(&ftdic));
		throw fe_canopen_error() << reason(ftdi_get_error_string(&ftdic));
	}
}

unsigned int gateway_epos_usb::ReadObject(WORD *ans, unsigned int ans_len, uint8_t nodeId, WORD index, BYTE subindex)
{
	WORD frame[4];

	frame[0] = (2 << 8) | (ReadObject_Op);
	frame[1] = index;
	/*
	 * high BYTE: 0x00(Node-ID == 0)
	 * low BYTE: subindex
	 */
	frame[2] = ((nodeId << 8 ) | subindex);
	frame[3] = 0x0000;

	sendCommand(frame);

	// read response
	unsigned int ret = readAnswer(ans, ans_len);

	// check error code
	checkEPOSerror(E_error);

	return ret;
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

void gateway_epos_usb::WriteObject(uint8_t nodeId, WORD index, BYTE subindex, uint32_t data)
{
	try {
		WORD frame[6];

		// fixed: (Len << 8) | OpCode
		frame[0] = (4 << 8) | (gateway::WriteObject_Op);
		frame[1] = index;
		frame[2] = ((nodeId << 8 ) | subindex); /* high BYTE: Node-ID, low BYTE: subindex */
		// data to transmit
		frame[3] = (data & 0x0000FFFF);
		frame[4] = (data >> 16);
		frame[5] = 0x00; // ZERO word, will be filled with checksum

		sendCommand(frame);

		// read response
		WORD answer[8];
		readAnswer(answer, 8);

		checkEPOSerror(E_error);
	}
	catch (fe_canopen_error & e) {
		e << dictionary_index(index);
		e << dictionary_subindex(subindex);
		e << canId(nodeId);
		throw;
	}
}

void gateway_epos_usb::InitiateSementedWrite(uint8_t nodeId, WORD index, BYTE subindex, DWORD ObjectLength)
{
	try {
		WORD frame[6];

		frame[0] = (4 << 8) | (gateway::InitiateSegmentedWrite_Op); // fixed: (len-1) == 3, WriteObject
		frame[1] = index;
		frame[2] = ((nodeId << 8 ) | subindex); /* high BYTE: Node-ID, low BYTE: subindex */
		// data to transmit
		*((DWORD *) &frame[3]) = ObjectLength;
		// frame[4] = << this is filled by the 32bit assignment above >>;
		frame[5] = 0x00; // ZERO word, will be filled with checksum

		sendCommand(frame);

		// read response
		WORD answer[8];
		readAnswer(answer, 8);

		checkEPOSerror(E_error);

		toggle = true;
	}
	catch (fe_canopen_error & e) {
		e << dictionary_index(index);
		e << dictionary_subindex(subindex);
		e << canId(nodeId);
		throw;
	}
}

void gateway_epos_usb::SegmentedWrite(uint8_t nodeId, BYTE * ptr, std::size_t len)
{
	if (len > 63) {
		BOOST_THROW_EXCEPTION(fe_canopen_error() << reason("Segmented write of > 63 bytes not allowed"));
	}
	try {
		WORD frame[32+2];

		memset(frame, 0, sizeof(frame));
		frame[0] = ((1+len/2) << 8) | (gateway::SegmentedWrite_Op); // fixed: (len-1) == 3, WriteObject
		frame[1] = len | (toggle ? (0x80) : 0x40);
		memcpy(((char *)&frame[1]+1), ptr, len);

		sendCommand(frame);

		// read response
		WORD answer[8];
		readAnswer(answer, 8);

		checkEPOSerror(E_error);

		// change the toggle flag value
		toggle = (toggle) ? false : true;
	}
	catch (fe_canopen_error & e) {
		e << canId(nodeId);
		throw;
	}
}

void gateway_epos_usb::SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier)
{
	WORD frame[4];

	frame[0] = (2 << 8) | (gateway::SendNMTService_Op); // (len << 8) | OpCode
	frame[1] = nodeId;
	frame[2] = CmdSpecifier;
	frame[3] = 0x00; // ZERO word, will be filled with checksum

	sendCommand(frame);

	// read response
	WORD answer[8];
	readAnswer(answer, 8);

	checkEPOSerror(E_error);
}

void gateway_epos_usb::SendCANFrame(WORD Identifier, WORD Length, const BYTE Data[8])
{
	WORD frame[8];

	frame[0] = (6 << 8) | (gateway::SendCANFrame_Op); // (len << 8) | OpCode
	frame[1] = Identifier;
	frame[2] = Length;

	if(Length > 0) frame[3] = Data[0];
	if(Length > 1) frame[3] |= (Data[1] << 8);
	if(Length > 2) frame[4] = Data[2];
	if(Length > 3) frame[4] |= (Data[3] << 8);
	if(Length > 4) frame[5] = Data[4];
	if(Length > 5) frame[5] |= (Data[5] << 8);
	if(Length > 6) frame[6] = Data[6];
	if(Length > 7) frame[6] |= (Data[7] << 8);

	frame[7] = 0x00; // ZERO word, will be filled with checksum

	sendCommand(frame);

	// read response
	WORD answer[8];

	unsigned int a = readAnswer(answer, 8);

	if (a != 2) {
		BOOST_THROW_EXCEPTION(fe_canopen_error() << reason("unexpected answer"));
	}

	checkEPOSerror(E_error);
}

} /* namespace canopen */
} /* namespace edp */
} /* namespace mrrocpp */
