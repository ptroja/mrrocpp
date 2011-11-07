#include "robot/canopen/gateway.h"

#include "cpv.h"

namespace mrrocpp {
namespace edp {
namespace festo {

cpv::cpv(canopen::gateway & _device, uint8_t _nodeId)
	: device(_device), nodeId(_nodeId)
{
}

U32 cpv::getDeviceType()
{
	return ReadObjectValue<U32>(0x1000, 0x00);
}

U8 cpv::getErrorRegister()
{
	return ReadObjectValue<U8>(0x1001, 0x00);
}

U32 cpv::getManufacturerStatusRegister()
{
	return ReadObjectValue<U32>(0x1002, 0x00);
}

U8 cpv::getNumberOfCurrentFaults()
{
	return ReadObjectValue<U8>(0x1003, 0x00);
}

U32 cpv::getMostRecentFault(uint8_t field)
{
	return ReadObjectValue<U32>(0x1003, field);
}

std::string cpv::getDeviceName()
{
	static std::string ret = "NOT IMPLEMENTED";
	return ret;
}

std::string cpv::getHardwareVersion()
{
	static std::string ret = "NOT IMPLEMENTED";
	return ret;
}

std::string cpv::getSoftwareVersion()
{
	static std::string ret = "NOT IMPLEMENTED";
	return ret;
}

U32 cpv::getVendorID()
{
	return ReadObjectValue<U32>(0x1018, 0x01);
}

U32 cpv::getProductCode()
{
	return ReadObjectValue<U32>(0x1018, 0x02);
}

U32 cpv::getRevisionNumber()
{
	return ReadObjectValue<U32>(0x1018, 0x03);
}

U32 cpv::getSerialNumber()
{
	return ReadObjectValue<U32>(0x1018, 0x04);
}

U8 cpv::readNumberOfCPModulesConnected()
{
	return ReadObjectValue<U8>(0x1027, 0x00);
}

U16 cpv::getModuleType(uint8_t module)
{
	return ReadObjectValue<U16>(0x1027, module);
}

U8 cpv::getNumberOf8OutputGroups()
{
	return ReadObjectValue<U16>(0x6200, 0x00);
}

U8 cpv::getOutputs(uint8_t group)
{
	return ReadObjectValue<U8>(0x6200, group);
}

void cpv::setOutputs(uint8_t group, uint8_t value)
{
	WriteObjectValue(0x6200, group, value);
}

U8 cpv::getNumberOf8OutputGroupsErrorMode()
{
	return ReadObjectValue<U16>(0x6206, 0x00);
}

U8 cpv::getOutputsErrorMode(uint8_t group)
{
	return ReadObjectValue<U8>(0x6206, group);
}

void cpv::setOutputsErrorMode(uint8_t group, uint8_t value)
{
	WriteObjectValue(0x6206, group, value);
}

U8 cpv::getNumberOf8OutputGroupsErrorValue()
{
	return ReadObjectValue<U16>(0x6207, 0x00);
}

U8 cpv::getOutputsErrorValue(uint8_t group)
{
	return ReadObjectValue<U8>(0x6207, group);
}

void cpv::setOutputsErrorValue(uint8_t group, uint8_t value)
{
	WriteObjectValue(0x6207, group, value);
}

#if 0

struct RPDO3 {
	uint16_t controlWord;
	int32_t targetPosition;
} __attribute__((packed));

const uint8_t nodeId = 10;

/*! \brief read Object Value from EPOS memory
 *
 * @param index object entry index in a dictionary
 * @param subindex object entry subindex of in a dictionary
 * @return object value
 */
template <class T>
T ReadObjectValue(gateway & device, WORD index, BYTE subindex)
{
	WORD answer[8];
	device.ReadObject(answer, 8, nodeId, index, subindex);

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
		throw canopen_error() << reason("Unsupported ReadObjectValue conversion");
	}
}

int main(int argc, char *argv[])
{
	gateway_epos_usb gateway;

	try {
		gateway.open();

		uint32_t DeviceType = ReadObjectValue<uint32_t>(gateway, 0x1000, 0x00);
		printf("Device type = 0x%08X\n", DeviceType);

		uint8_t ErrorRegister = ReadObjectValue<uint8_t>(gateway, 0x1001, 0x00);
		printf("Error register = 0x%02X\n", ErrorRegister);

		uint32_t ManufacturerStatusRegister = ReadObjectValue<uint32_t>(gateway, 0x1002, 0x00);
		printf("Manufacturer status register = 0x%08X\n", ManufacturerStatusRegister);


		// Get the 'Identity Object', see P.BE-CP-CO2-EN manual, p. 2-15.

		uint8_t NumberOfEntries = ReadObjectValue<uint8_t>(gateway, 0x1018, 0x00);
		printf("NumberOfEntries = %d\n", NumberOfEntries);

		uint32_t VendorID = ReadObjectValue<uint32_t>(gateway, 0x1018, 0x01);
		printf("Vendor ID = 0x%08X\n", VendorID);

		uint32_t ProductCode = ReadObjectValue<uint32_t>(gateway, 0x1018, 0x02);
		printf("Product code = 0x%08X\n", ProductCode);

		uint32_t RevisionNumber = ReadObjectValue<uint32_t>(gateway, 0x1018, 0x03);
		printf("Revision number = 0x%08X\n", RevisionNumber);

		uint32_t SerialNumber = ReadObjectValue<uint32_t>(gateway, 0x1018, 0x04);
		printf("Serial number = 0x%08X\n", SerialNumber);

		uint8_t NumberOfOutputGroups = ReadObjectValue<uint8_t>(gateway, 0x6200, 0x00);
		printf("Number of 8-output groups = %d\n", NumberOfOutputGroups);

		uint8_t Outputs07 = ReadObjectValue<uint8_t>(gateway, 0x6200, 0x01);
		printf("Status of outputs 0..7 = 0x%02x\n", Outputs07);

		gateway.SendNMTService(nodeId, gateway::Start_Remote_Node);

		while(1) {
		gateway.WriteObject(nodeId, 0x6200, 0x01, 0);
		sleep(1);
		gateway.WriteObject(nodeId, 0x6200, 0x01, 0x08+0x20+0x80);
		sleep(5);
		gateway.WriteObject(nodeId, 0x6200, 0x01, 0);
		sleep(1);
		gateway.WriteObject(nodeId, 0x6200, 0x01, 0x04+0x10+0x40);
		sleep(5);
		gateway.WriteObject(nodeId, 0x6200, 0x01, 0);
		}

//		gateway.SendNMTService(0, gateway::Start_Remote_Node);
//		gateway.SendNMTService(1, gateway::Start_Remote_Node);

		gateway.close();
	} catch (canopen_error & error) {
		std::cerr << "EPOS Error." << std::endl;

		if ( std::string const * r = boost::get_error_info<reason>(error) )
			std::cerr << " Reason: " << *r << std::endl;

		if ( std::string const * call = boost::get_error_info<errno_call>(error) )
			std::cerr << " Errno call: " << *call << std::endl;

		if ( int const * errno_value = boost::get_error_info<errno_code>(error) )
			std::cerr << "Errno value: " << *errno_value << std::endl;
	} catch (...) {
		std::cerr << "Unhandled exception" << std::endl;
	}

	return 0;
}
#endif

} /* namespace festo */
} /* namespace edp */
} /* namespace mrrocpp */
