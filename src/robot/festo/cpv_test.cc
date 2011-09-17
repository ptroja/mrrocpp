/*
 * cpv_test.cc
 *
 *  Created on: Sep 17, 2011
 *      Author: ptroja
 */

#include <cstdio>

#include "robot/canopen/gateway_epos_usb.h"

#include "cpv.h"

using namespace mrrocpp::edp::canopen;
using namespace mrrocpp::edp::festo;
using namespace std;

const uint8_t nodeId = 10;

int main(int argc, char *argv[])
{
	gateway_epos_usb gateway;

	cpv cpv10(gateway, nodeId);

	try {
		gateway.open();

		U32 DeviceType = cpv10.readDeviceType();
		printf("Device type = 0x%08X\n", DeviceType);

		U8 ErrorRegister = cpv10.readErrorRegister();
		printf("Error register = 0x%02X\n", ErrorRegister);

		U32 ManufacturerStatusRegister = cpv10.readManufacturerStatusRegister();
		printf("Manufacturer status register = 0x%08X\n", ManufacturerStatusRegister);

		uint8_t NumberOfOutputGroups = cpv10.readNumberOf8OutputGroups();
		printf("Number of 8-output groups = %d\n", NumberOfOutputGroups);

		uint8_t Outputs07 = cpv10.readOutputs(1);
		printf("Status of outputs 0..7 = 0x%02x\n", Outputs07);

		gateway.SendNMTService(nodeId, gateway::Start_Remote_Node);

		while(1) {
			cpv10.writeOutputs(1, 0x00);
			sleep(1);
			// Move the pistons up
			cpv10.writeOutputs(1, 0x08+0x20+0x80);
			sleep(5);
			cpv10.writeOutputs(1, 0x00);
			sleep(1);
			// Move the pistons down
			cpv10.writeOutputs(1, 0x04+0x10+0x40);
			sleep(5);
			cpv10.writeOutputs(1, 0x00);
		}

		gateway.close();
	} catch (canopen_error & error) {
		std::cerr << "CANopen Error." << std::endl;

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


