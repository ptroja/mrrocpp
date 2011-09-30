/*!
 * \file gateway_epos_usb.h
 * \brief USB transport layer
 */

#ifndef EPOS_ACCESS_USB_H_
#define EPOS_ACCESS_USB_H_

#include <ftdi.h>

#include "gateway.h"

namespace mrrocpp {
namespace edp {
namespace canopen {

//! Access to the EPOS with the USB transport layer
class gateway_epos_usb : public gateway {
private:
	//! USB FTDI context
	struct ftdi_context ftdic;

	//! USB device identifiers
	const int vendor, product, index;

	//! Send data to device via USB
	void sendCommand(WORD *frame);

	//! Read from device via USB
	unsigned int readAnswer(WORD *ans, unsigned int ans_len);

	//! toggle bit used for segmented write
	bool toggle;

public:
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

	/*! \brief create new USB EPOS object
	 *
	 * @param _vendor USB device vendor ID
	 * @param _product USB device vendor ID
	 * @param index USB device vendor ID
	 */
	gateway_epos_usb(int _vendor = 0x0403, int _product = 0xa8b0, unsigned int index = 0);

	//! Destructor
	virtual ~gateway_epos_usb();

	//! Open device
	void open();

	//! Close device
	void close();
};

} /* namespace canopen */
} /* namespace edp */
} /* namespace mrrocpp */

#endif /* EPOS_ACCESS_USB_H_ */
