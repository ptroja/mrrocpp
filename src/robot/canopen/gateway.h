/*!
 * \file gateway.h
 * \brief Abstract types for transport-level access
 */

#ifndef EPOS_ACCESS_BASE_H_
#define EPOS_ACCESS_BASE_H_

#include <stdint.h>  /* int types with given size */
#include <string>

#include "canopen_exceptions.hpp"
#include <boost/type_traits/is_same.hpp>

namespace mrrocpp {
namespace edp {
namespace canopen {


/*!
 * Data types used for communication (Communication Guide reference)
 */

typedef uint32_t DWORD; ///< \brief 32bit type for EPOS data exchange
typedef uint16_t WORD; ///< \brief 16bit type for EPOS data exchange
typedef uint8_t BYTE; ///< \brief 8bit type for EPOS data exchange

//! Abstract class for access to the EPOS at the transport layer
class gateway {
protected:
	//! Flag indicating connection status
	bool device_opened;

	//! debug level
	int debug;

	//! EPOS error status
	DWORD E_error;

public:
	//! Constructor
	gateway() : device_opened(false), debug(0)
	{}

	//! Destructor
	virtual ~gateway()
	{}

	//! Operation codes of CANopen datagrams
	typedef enum _CanOpen_OpCode {
		Response_Op = 0x00,

		ReadObject_Op = 0x10,
		InitiateSegmentedRead_Op = 0x12,
		SegmentedRead_Op = 0x14,

		WriteObject_Op = 0x11,
		InitiateSegmentedWrite_Op = 0x13,
		SegmentedWrite_Op = 0x15,

		SendNMTService_Op = 0x0E,
		SendCANFrame_Op = 0x20,
		RequestCANFrame_Op = 0x21,
		SendLSSFrame_Op = 0x30,
		ReadLSSFrame_Op = 0x31
	} CanOpen_OpCode_t;

	/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.1
	 *
	 * @param ans answer buffer
	 * @param ans_len of answer buffer
	 * @param nodeId node-Id of the target device
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @return answer array from the controller
	 */
	virtual unsigned int ReadObject(WORD *ans, unsigned int ans_len, uint8_t nodeId, WORD index, BYTE subindex) = 0;

	std::string ReadObjectStringValue(uint8_t nodeId, canopen::WORD index, canopen::BYTE subindex)
	{
		canopen::WORD answer[8];
		unsigned int r = this->ReadObject(answer, 8, nodeId, index, subindex);

		std::string str;

		for(int i = 0; i < r; ++i) {
			char c = (answer[3+i] & 0x00FF);

			if(c == 0x00) {
				break;
			} else {
				str += c;
			}

			c = ((answer[3+i] & 0xFF00) >> 8);

			if(c == 0x00) {
				break;
			} else {
				str += c;
			}
		}

		return str;
	}

	template <class T>
	T ReadObjectValue(uint8_t nodeId, canopen::WORD index, canopen::BYTE subindex)
	{
		canopen::WORD answer[8];
		this->ReadObject(answer, 8, nodeId, index, subindex);

#ifdef DEBUG
		T val;
		printf("ReadObjectValue(%0x04x, 0x02x)==> %d\n", val);
#endif

		if ((boost::is_same <T, uint8_t>::value) || (boost::is_same <T, int8_t>::value)
				|| (boost::is_same <T, uint16_t>::value) || (boost::is_same <T, int16_t>::value)) {
			T val = (T) answer[3];
			return val;
		} else if ((boost::is_same <T, uint32_t>::value) || (boost::is_same <T, int32_t>::value)) {
			T val = (T) (answer[3] | (answer[4] << 16));
			return val;
		} else {
			throw canopen::se_canopen_error() << canopen::reason("Unsupported ReadObjectValue conversion");
		}
	}

#if 0
	/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.2
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 */
	int InitiateSegmentedRead(WORD index, BYTE subindex );

	/*! \brief read data segment of the object initiated with 'InitiateSegmentedRead()'
	 *
	 * @param ptr pointer to data to be filled
	 */
	int SegmentRead(WORD **ptr);
#endif

	/*! \brief write object value to EPOS (for up to 4 bytes)
	 *
	 * @param nodeId CAN node ID
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param data 32bit object data
	 */
	virtual void WriteObject(uint8_t nodeId, WORD index, BYTE subindex, uint32_t data) = 0;

	/*! \brief Initiate Write Object to EPOS memory (for 5 bytes and more)
	 *
	 * @param nodeId CAN node ID
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param ObjectLength length of the object to write
	 */
	virtual void InitiateSementedWrite(uint8_t nodeId, WORD index, BYTE subindex, DWORD ObjectLength) = 0;

	/*! \brief write data segment of the object initiated with 'InitiateSegmentedWrite()'
	 *
	 * @param nodeId CAN node ID
	 * @param ptr pointer to data to be filled
	 * @param len length of the data to write
	 */
	virtual void SegmentedWrite(uint8_t nodeId, BYTE * ptr, std::size_t len) = 0;

	//! CAN Network Management Commands
	typedef enum _NMT_Command
	{
		Start_Remote_Node = 1,
		Stop_Remote_Node = 2,
		Enter_Pre_Operational = 128,
		Reset_Node = 129,
		Reset_Communication = 130
	} NMT_COMMAND_t;

	/*! \brief Send a NMT service to, for example, change NMT state or reset the device.
	 *
	 *  \param nodeId CAN node ID
	 *  \param CmdSpecifier command specifier
	 */
	virtual void SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier) = 0;

	/*! \brief Send CAN frame the the CAN bus
	 *
	 *  @param Identifier CAN Frame 11-bit Identifier
	 *  @param Length CAN Frame Data Length Code (DLC)
	 *  @param Data CAN Frame Data
	 */
	virtual void SendCANFrame(WORD Identifier, WORD Length, const BYTE Data[8]) = 0;

	//! Open device
	virtual void open() = 0;

	//! Close device
	virtual void close() = 0;

	/*! \brief check for EPOS error code
	 *
	 * @param E_error epos error code
	 */
	static void checkEPOSerror(DWORD E_error);

	/*! \brief Checksum calculation
	 *
	 * Copied from EPOS Communication Guide, p.8
	 *
	 * @param pDataArray pointer to data for checksum calculation
	 * @param numberOfWords length of the data
	 */
	/*static*/ WORD CalcFieldCRC(const WORD *pDataArray, WORD numberOfWords);

	//! Set the debug level
	void setDebugLevel(int level) {
		debug = level;
	}
};

} /* namespace canopen_ */
} /* namespace edp */
} /* namespace mrrocpp */

#endif /* EPOS_ACCESS_BASE_H_ */
