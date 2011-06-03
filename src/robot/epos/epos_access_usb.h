/*
 * epos_access_usb.h
 *
 *  Created on: Jan 18, 2011
 *      Author: ptroja
 */

#ifndef EPOS_ACCESS_USB_H_
#define EPOS_ACCESS_USB_H_

#include <ftdi.h>

#include "epos_access.h"

namespace mrrocpp {
namespace edp {
namespace epos {

class epos_access_usb : public epos_access {
private:
	//! USB FTDI context
	struct ftdi_context ftdic;

	//! USB device identifiers
	const int vendor, product, index;

public:
	//! Send data to device via USB
	void sendCommand(WORD *frame);

	//! Read from device via USB
	unsigned int readAnswer(WORD *ans, unsigned int ans_len);

	//! Send a NMT service to, for example, change NMT state or reset the device.
	void SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier);

	//! Send CAN frame the the CAN bus
	void SendCANFrame(WORD Identifier, WORD Length, const BYTE Data[8]);

	/*! \brief create new USB EPOS object
	 *
	 * @param vendor USB device vendor ID
	 * @param product USB device vendor ID
	 * @param index USB device vendor ID
	 */
	epos_access_usb(int _vendor = 0x0403, int _product = 0xa8b0, unsigned int index = 0);

	//! Destructor
	~epos_access_usb();

	//! Open device
	void open();

	//! Close device
	void close();
};

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */

#endif /* EPOS_ACCESS_USB_H_ */
