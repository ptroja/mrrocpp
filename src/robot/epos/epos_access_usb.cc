/*
 * epos_access_usb.cc
 *
 *  Created on: Jan 18, 2011
 *      Author: ptroja
 */

#include <cstdio>

#include "epos_access_usb.h"
#include "epos.h"

namespace mrrocpp {
namespace edp {
namespace epos {

epos_access_usb::epos_access_usb(int _vendor, int _product, unsigned int _index) :
	vendor(_vendor), product(_product), index(_index)
{
	if (ftdi_init(&ftdic) < 0) {
		fprintf(stderr, "ftdi_init failed\n");
	}
}

epos_access_usb::~epos_access_usb()
{
	if (device_opened) close();
	ftdi_deinit(&ftdic);
}

void epos_access_usb::open()
{
	//! FTDI calls return code
	int ret;

	if ((ret = ftdi_usb_open(&ftdic, vendor, product)) < 0) {
		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	}

	//! reset FTDI device
	if ((ret = ftdi_usb_reset(&ftdic)) < 0) {
		fprintf(stderr, "unable to reset ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	}

	if ((ret = ftdi_set_line_property(&ftdic, BITS_8, STOP_BIT_1, NONE)) < 0) {
		fprintf(stderr, "unable to set ftdi line property: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
        throw epos_error() << reason(ftdi_get_error_string(&ftdic));
    }

	//! set flow control
	if ((ret = ftdi_setflowctrl(&ftdic, SIO_DISABLE_FLOW_CTRL)) < 0) {
	//if ((ret = ftdi_setflowctrl(&ftdic, SIO_RTS_CTS_HS)) < 0) {
		fprintf(stderr, "unable to set ftdi flow control: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
        throw epos_error() << reason(ftdi_get_error_string(&ftdic));
    }

	//! set latency timer
	if ((ret = ftdi_set_latency_timer(&ftdic, 1)) < 0) {
		fprintf(stderr, "unable to set ftdi latency timer: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	}

	//! set baud rate
	if ((ret = ftdi_set_baudrate(&ftdic, 1000000)) < 0) {
		fprintf(stderr, "unable to set ftdi baudrate: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	}
#if 0
	//! set write data chunk size
	if ((ret = ftdi_write_data_set_chunksize(&ftdic, 512)) < 0) {
		fprintf(stderr, "unable to set ftdi write chunksize: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	}

	//! set read data chunk size
	if ((ret = ftdi_read_data_set_chunksize(&ftdic, 512)) < 0) {
		fprintf(stderr, "unable to set ftdi read chunksize: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	}
#endif

	device_opened = true;
}

void epos_access_usb::close()
{
	if(ftdi_usb_close(&ftdic)) {
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	}

	device_opened = false;
}

#define DLE	(0x90)	// Data Link Escape
#define STX	(0x02)	// Start of Text

unsigned int epos_access_usb::readAnswer(WORD *ans, unsigned int ans_len)
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
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	} else if (ret == 0) {
		throw epos_error() << reason("no data returned");
	}

#if 0
	printf("<< ");
	for(int i=0; i<ret; i++) {
		printf("0x%02x,", buf[i]);
	}
	printf("\n");
#endif

	// check DLE
	if (buf[0] != DLE) {
		throw epos_error() << reason("Datagram error (DLE expected)");
	}

	// check STX
	if (buf[1] != STX) {
		throw epos_error() << reason("Datagram error (STX expected)");
	}

	// read OpCode
	if (buf[2] != 0x00) {
		throw epos_error() << reason("Datagram error (0x00 Answer OpCode expected)");
	}

	// frame length
	WORD framelen = buf[3];

	if (ans_len < framelen) {
		throw epos_error() << reason("output buffer to short for a message");
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
#ifdef DEBUG
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

void epos_access_usb::sendCommand(WORD *frame)
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

#if 0
	printf(">> ");
	for (unsigned int i=0; i<idx; ++i) {
		printf( "0x%02x,", datagram[i] );
	}
	printf("\n");
#endif

	int w = ftdi_write_data(&ftdic, datagram, idx);

	if (w != (int) idx) {
		fprintf(stderr, "ftdi device write failed (%d/%d chars written): (%s)\n", w, idx, ftdi_get_error_string(&ftdic));
		throw epos_error() << reason(ftdi_get_error_string(&ftdic));
	}
}

void epos_access_usb::SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier)
{
	WORD frame[4];

	frame[0] = (2 << 8) | 0x0E; // (len << 8) | OpCode
	frame[1] = nodeId;
	frame[2] = CmdSpecifier;
	frame[3] = 0x00; // ZERO word, will be filled with checksum

	sendCommand(frame);

	// read response
	WORD answer[8];
	readAnswer(answer, 8);

	epos::checkEPOSerror(E_error);
}

void epos_access_usb::SendCANFrame(WORD Identifier, WORD Length, BYTE Data[8])
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

	// read response
	WORD answer[8];
	readAnswer(answer, 8);

	epos::checkEPOSerror(E_error);
}


} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */
