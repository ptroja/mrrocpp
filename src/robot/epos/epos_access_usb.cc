/*
 * epos_access_usb.cc
 *
 *  Created on: Jan 18, 2011
 *      Author: ptroja
 */

#include <cstdio>

#include "epos.h"
#include "epos_access_usb.h"

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
	int wait_cnt = 20;
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

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */
