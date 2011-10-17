/*! \file festo.h

 \brief libfesto - a library to control an FESTO CO2 field bus devices; definition

 \date Septrmber 2011
 \author Piotr Trojanek <piotr.trojanek@gmail.com>, Warsaw University of Technology

 * \defgroup libfesto Library for control of the FESTO field bus devices
 * @{
 */

#ifndef _CPV_H
#define _CPV_H

#include <stdint.h>  /* int types with given size */

#include <string>
#include <exception>

#include "robot/canopen/gateway.h"

#define FESTO_C1_GROUP 1
#define FESTO_C1_BIT (1<<0)
#define FESTO_C1_BIT_TO_SET 0

#define FESTO_C2_GROUP 2
#define FESTO_C2_BIT (1<<1)
#define FESTO_C2_BIT_TO_SET 1

#define FESTO_C3_GROUP 2
#define FESTO_C3_BIT (1<<0)
#define FESTO_C3_BIT_TO_SET 0

#define FESTO_CY11_GROUP 1
#define FESTO_CY11_BIT (1<<3)
#define FESTO_CY11_BIT_TO_SET 3

#define FESTO_CY12_GROUP 1
#define FESTO_CY12_BIT (1<<2)
#define FESTO_CY12_BIT_TO_SET 2

#define FESTO_CY21_GROUP 1
#define FESTO_CY21_BIT (1<<5)
#define FESTO_CY21_BIT_TO_SET 5

#define FESTO_CY22_GROUP 1
#define FESTO_CY22_BIT (1<<4)
#define FESTO_CY22_BIT_TO_SET 4

#define FESTO_CY31_GROUP 1
#define FESTO_CY31_BIT (1<<7)
#define FESTO_CY31_BIT_TO_SET 7

#define FESTO_CY32_GROUP 1
#define FESTO_CY32_BIT (1<<6)
#define FESTO_CY32_BIT_TO_SET 6

#define FESTO_CH1_GROUP 2
#define FESTO_CH1_BIT (1<<5)
#define FESTO_CH1_BIT_TO_SET 5

#define FESTO_CH2_GROUP 2
#define FESTO_CH2_BIT (1<<3)
#define FESTO_CH2_BIT_TO_SET 3

#define FESTO_CH3_GROUP 2
#define FESTO_CH3_BIT (1<<4)
#define FESTO_CH3_BIT_TO_SET 4

#define FESTO_A1_GROUP 1
#define FESTO_A1_BIT (1<<1)
#define FESTO_A1_BIT_TO_SET 1

#define FESTO_A2_GROUP 2
#define FESTO_A2_BIT (1<<2)
#define FESTO_A2_BIT_TO_SET 2

#define FESTO_A3_GROUP 2
#define FESTO_A3_BIT (1<<7)
#define FESTO_A3_BIT_TO_SET 7

#define FESTO_H1_GROUP 2
#define FESTO_H1_BIT (1<<6)
#define FESTO_H1_BIT_TO_SET 6

namespace mrrocpp {
namespace edp {
namespace festo {

/*!
 * Data types used for object dictionary ('Field bus protocol: CANopen' manual)
 */

//! unsigned 8-bit integer
typedef uint8_t U8;

//! unsigned 16-bit integer
typedef uint16_t U16;

//! unsigned 32-bit integer
typedef uint32_t U32;

//! \brief interface to FESTO device
class cpv
{
private:
	/* Implement read functions defined in EPOS Communication Guide */

	/*! \brief read Object Value
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @return object value
	 */
	template <class T>
	T ReadObjectValue(canopen::WORD index, canopen::BYTE subindex)
	{
		return device.ReadObjectValue <T>(nodeId, index, subindex);
	}

	/*! \brief write object value to EPOS
	 *
	 * @param index object entry index in a dictionary
	 * @param subindex object entry subindex of in a dictionary
	 * @param data object data
	 */
	template <class T>
	void WriteObjectValue(canopen::WORD index, canopen::BYTE subindex, T data)
	{
		device.WriteObject(nodeId, index, subindex, (uint32_t) data);
	}

	//! Object to access the device
	canopen::gateway & device;

	//! ID of the device on the CAN bus
	const uint8_t nodeId;

	//! remote operation enable bit
	bool remote;

public:
	/*! \brief create new controller object
	 *
	 * @param _device object to access the device
	 * @param _nodeId ID of the device on the CAN bus
	 */
	cpv(canopen::gateway & _device, uint8_t _nodeId);

	U32 readDeviceType();

	U8 readErrorRegister();

	U32 readManufacturerStatusRegister();

	U8 readNumberOfCurrentFaults();

	/*! \brief read most recent fault table
	 *
	 * @param field field id (0..10)
	 */
	U32 readMostRecentFault(uint8_t field);

	std::string readDeviceName();

	std::string readHardwareVersion();

	std::string readSoftwareVersion();

	/* Hardbeat protocol configuration not implemented */

	U32 readVendorID();

	U32 readProductCode();

	U32 readRevisionNumber();

	U32 readSerialNumber();

	U8 readNumberOfCPModulesConnected();

	/*! \brief read connected CP module type recent fault table
	 *
	 * @param module module id (1..3)
	 */
	U16 readModuleType(uint8_t module);

	/* Digital CP inputs support not implemented */

	/*! \brief read number of 8-output groups
	 *
	 * @return Number of 8-output groups (2 or 4)
	 */
	U8 readNumberOf8OutputGroups();

	/*! \brief read outputs status
	 *
	 * @param group outputs group id (1..4)
	 * @return outputs status
	 */
	U8 readOutputs(uint8_t group);

	/*! \brief read connected CP module type recent fault table
	 *
	 * @param group outputs group id (1..4)
	 */
	void writeOutputs(uint8_t group, uint8_t value);

	U8 readNumberOf8OutputGroupsErrorMode();

	U8 readOutputsErrorMode(uint8_t group);

	void writeOutputsErrorMode(uint8_t group, uint8_t value);

	U8 readNumberOf8OutputGroupsErrorValue();

	U8 readOutputsErrorValue(uint8_t group);

	void writeOutputsErrorValue(uint8_t group, uint8_t value);
};

} /* namespace festo */
} /* namespace edp */
} /* namespace mrrocpp */

//! @}

#endif
