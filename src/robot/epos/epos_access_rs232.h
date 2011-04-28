/*
 * epos_access_rs232.h
 *
 *  Created on: Jan 18, 2011
 *      Author: ptroja
 */

#ifndef EPOS_ACCESS_RS232_H_
#define EPOS_ACCESS_RS232_H_

#include <string>

#include <termios.h> /* POSIX terminal control definitions */

#include "epos_access.h"

namespace mrrocpp {
namespace edp {
namespace epos {

class epos_access_rs232 : public epos_access {
private:
	//! device name of EPOS port
	const std::string device;

	//! EPOS file descriptor
	int ep;

	//! serial port settings
	struct termios options;

	//! for internal progress character handling
	char gMarker;

	/* helper functions below */

	/*! \brief write a single BYTE to EPOS
	 *
	 * @param c BYTE to write
	 */
	void writeBYTE(BYTE c);

	/*! \brief  write a single WORD to EPOS
	 *
	 * @param w WORD to write
	 */
	void writeWORD(WORD w);

	/*! \brief  read a single BYTE from EPOS, timeout implemented
	 *
	 * @return readed data BYTE
	 */
	BYTE readBYTE();

	/*! \brief  read a single WORD from EPOS, timeout implemented
	 *
	 * @return readed data BYTE
	 */
	WORD readWORD();

public:
	/*! \brief create new EPOS object
	 *
	 * @param _device device string describing the device on which the EPOS is connected to, e.g. "/dev/ttyS0"
	 */
	epos_access_rs232(const std::string & _device);

	//! Destructor
	~epos_access_rs232();

	//! Send data to device
	void sendCommand(WORD *frame);

	//! Read from device
	unsigned int readAnswer(WORD *ans, unsigned int ans_len);

	//! Send a NMT service to, for example, change NMT state or reset the device.
	void SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier);

	//! Send CAN frame the the CAN bus
	void SendCANFrame(WORD Identifier, WORD Length, const BYTE Data[8]);

	//! Open device
	void open();

	//! Close device
	void close();
};

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */

#endif /* EPOS_ACCESS_RS232_H_ */
