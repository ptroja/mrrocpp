/*
 * cpv_test.cc
 *
 *  Created on: Sep 17, 2011
 *      Author: ptroja
 */

#include <cstdio>
#include <iostream>

#include "robot/canopen/gateway_epos_usb.h"

#include "cpv.h"

using namespace mrrocpp::edp::canopen;
using namespace mrrocpp::edp::festo;
using namespace std;

#define FESTO_C1_GROUP 1
#define FESTO_C1_BIT (1<<0)

#define FESTO_C2_GROUP 2
#define FESTO_C2_BIT (1<<1)

#define FESTO_C3_GROUP 2
#define FESTO_C3_BIT (1<<0)

#define FESTO_CY11_GROUP 1
#define FESTO_CY11_BIT (1<<3)

#define FESTO_CY12_GROUP 1
#define FESTO_CY12_BIT (1<<2)

#define FESTO_CY21_GROUP 1
#define FESTO_CY21_BIT (1<<5)

#define FESTO_CY22_GROUP 1
#define FESTO_CY22_BIT (1<<4)

#define FESTO_CY31_GROUP 1
#define FESTO_CY31_BIT (1<<7)

#define FESTO_CY32_GROUP 1
#define FESTO_CY32_BIT (1<<6)

#define FESTO_CH1_GROUP 2
#define FESTO_CH1_BIT (1<<5)

#define FESTO_CH2_GROUP 2
#define FESTO_CH2_BIT (1<<3)

#define FESTO_CH3_GROUP 2
#define FESTO_CH3_BIT (1<<4)

#define FESTO_A1_GROUP 1
#define FESTO_A1_BIT (1<<1)

#define FESTO_A2_GROUP 2
#define FESTO_A2_BIT (1<<2)

#define FESTO_A3_GROUP 2
#define FESTO_A3_BIT (1<<7)

#define FESTO_H1_GROUP 2
#define FESTO_H1_BIT (1<<6)

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

		while (1) {
			cpv10.writeOutputs(1, 0x00);
			cpv10.writeOutputs(2, 0x00);
			sleep(2);
			// Move the pistons up
			cpv10.writeOutputs(FESTO_H1_GROUP, FESTO_H1_BIT);
			sleep(3);
			/*
			 cpv10.writeOutputs(1, 0x00);
			 cpv10.writeOutputs(2, 0x00);
			 sleep(2);
			 cpv10.writeOutputs(FESTO_CY32_GROUP, FESTO_CY32_BIT);
			 sleep(5);
			 */
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

