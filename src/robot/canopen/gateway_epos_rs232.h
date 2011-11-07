/*!
 * \file epos_access_rs232.h
 * \brief RS232 transport layer
 */

#ifndef EPOS_ACCESS_RS232_H_
#define EPOS_ACCESS_RS232_H_

#include <string>

#include <termios.h> /* POSIX terminal control definitions */

#include "gateway.h"

namespace mrrocpp {
namespace edp {
namespace canopen {

//! Access to the EPOS with the RS232 transport layer
class gateway_epos_rs232 : public gateway {
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

	//! Send data to device
	void sendCommand(WORD *frame);

	//! Read from device
	unsigned int readAnswer(WORD *ans, unsigned int ans_len);

	//! toggle bit used for segmented write
	bool toggle;

public:
	/*! \brief create new EPOS object
	 *
	 * @param _device device string describing the device on which the EPOS is connected to, e.g. "/dev/ttyS0"
	 */
	gateway_epos_rs232(const std::string & _device);

	//! Destructor
	virtual ~gateway_epos_rs232();

	/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.1
	 *
	 * @param ans answer buffer
	 * @param ans_len of answer buffer
	 * @param nodeId CAN node ID
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @return answer array from the controller
	 */
	unsigned int ReadObject(WORD *ans, unsigned int ans_len, uint8_t nodeId, WORD index, BYTE subindex);

	/*! \brief write object value to EPOS
	 *
	 * @param nodeId CAN node ID
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param data 32bit object data
	 */
	void WriteObject(uint8_t nodeId, WORD index, BYTE subindex, uint32_t data);

	/*! \brief Initiate Write Object to EPOS memory (for 5 bytes and more)
	 *
	 * @param nodeId CAN node ID
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param ObjectLength object length
	 */
	void InitiateSementedWrite(uint8_t nodeId, WORD index, BYTE subindex, DWORD ObjectLength);

	/*! \brief write data segment of the object initiated with 'InitiateSegmentedWrite()'
	 *
	 * @param nodeId CAN node ID
	 * @param ptr pointer to data to be filled
	 * @param len length of the data to write
	 */
	void SegmentedWrite(uint8_t nodeId, BYTE * ptr, std::size_t len);

	//! Send a NMT service to, for example, change NMT state or reset the device.
	void SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier);

	//! Send CAN frame the the CAN bus
	void SendCANFrame(WORD Identifier, WORD Length, const BYTE Data[8]);

	//! Open device
	void open();

	//! Close device
	void close();
};

} /* namespace canopen */
} /* namespace edp */
} /* namespace mrrocpp */

#endif /* EPOS_ACCESS_RS232_H_ */
