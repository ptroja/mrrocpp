/*
 * cpv_test.cc
 *
 *  Created on: Sep 17, 2011
 *      Author: ptroja and yoyek
 */

#include <cstdio>
#include <iostream>
#include <bitset>

#include "../canopen/gateway_epos_usb.h"

#include "../festo/cpv.h"
#include "../maxon/epos.h"

using namespace mrrocpp::edp::canopen;
using namespace mrrocpp::edp::festo;
using namespace mrrocpp::edp::maxon;

using namespace std;



const uint8_t nodeId = 10;

int main(int argc, char *argv[])
{
	gateway_epos_usb gateway;

	cpv cpv10(gateway, nodeId);

	try {
		gateway.open();

		epos node(gateway, 8);

		std::bitset <16> epos_digits;

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

		while (1) {
			cpv10.writeOutputs(1, 0x00);
			cpv10.writeOutputs(2, 0x00);
			sleep(2);
			// Move the pistons up
			cpv10.writeOutputs(FESTO_CY21_GROUP, FESTO_CY21_BIT);
			sleep(3);
			epos_digits = node.readDInput();
			std::cout << "epos digital inputs 1= " << epos_digits << std::endl;

			cpv10.writeOutputs(1, 0x00);
			cpv10.writeOutputs(2, 0x00);
			sleep(2);
			cpv10.writeOutputs(FESTO_CY22_GROUP, FESTO_CY22_BIT);
			sleep(5);
			epos_digits = node.readDInput();
			std::cout << "epos digital inputs 2= " << epos_digits << std::endl;
			cpv10.writeOutputs(1, 0x00);
			cpv10.writeOutputs(2, 0x00);
		}

		gateway.close();
	} catch (canopen_error & error) {
		std::cerr << "CANopen Error." << std::endl;

		if ( std::string const * r = boost::get_error_info<reason>(error))
			std::cerr << " Reason: " << *r << std::endl;

		if ( std::string const * call = boost::get_error_info<errno_call>(error))
			std::cerr << " Errno call: " << *call << std::endl;

		if ( int const * errno_value = boost::get_error_info<errno_code>(error))
			std::cerr << "Errno value: " << *errno_value << std::endl;
	} catch (...) {
		std::cerr << "Unhandled exception" << std::endl;
	}

	return 0;
}

