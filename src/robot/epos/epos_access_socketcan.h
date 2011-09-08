/*!
 * \file epos_access_socketcan.h
 * \brief SocketCAN transport layer
 */

#ifndef EPOS_ACCESS_SOCKETCAN_H_
#define EPOS_ACCESS_SOCKETCAN_H_

#include <string>

#include <sys/socket.h>
#include <linux/can.h>

#include "epos_access.h"

namespace mrrocpp {
namespace edp {
namespace epos {

//! Access to the EPOS with the SocketCAN transport layer
class epos_access_socketcan : public epos_access {
private:
	//! toggle bit used for segmented write
	bool toggle;

	//! interface name
	const std::string iface;

	//! socket descriptor
	int sock;

	//! write CAN data frame to the network interface
	void writeToWire(const struct can_frame & frame);

	//! read CAN data frame from the network interface
	canid_t readFromWire(struct can_frame & frame);

	//! handle the CanOpen protocol management messages
	void handleCanOpenMgmt(const struct can_frame & frame);

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
	 * @param iface SocketCAN interface to use (i.e. "can0")
	 */
	epos_access_socketcan(const std::string & iface);

	//! Destructor
	virtual ~epos_access_socketcan();

	//! Open device
	void open();

	//! Close device
	void close();
};

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */

#endif /* EPOS_ACCESS_SOCKETCAN_H_ */
