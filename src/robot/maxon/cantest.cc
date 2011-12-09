#include <iostream>
#include <cstdio>
#include <boost/exception/get_error_info.hpp>
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <sys/time.h>

#include "robot/canopen/gateway_epos_usb.h"
#include "epos.h"

using namespace mrrocpp::edp::canopen;
using namespace mrrocpp::edp::maxon;

struct RPDO3 {
	uint16_t controlWord;
	int32_t targetPosition;
} __attribute__((packed));

int main(int argc, char *argv[])
{
	gateway_epos_usb gateway;

	try {
		gateway.open();

		epos node0(gateway, 0);

		gateway.SendNMTService(0, gateway::Start_Remote_Node);
		gateway.SendNMTService(1, gateway::Start_Remote_Node);

		node0.reset();

		std::cout << "gateway CAN-ID: " << (int) node0.getNodeID() << std::endl;

		RPDO3 cmd;

		cmd.controlWord = 0x0102;
		cmd.targetPosition = 0xabcf;

		for (uint8_t cmdNode = 1; cmdNode < 7; ++cmdNode) {
			epos nodeX(gateway, cmdNode);

			nodeX.setTargetPosition(291);

			printf("0x%04x -> 0x%04x remote? %s\n", (int) cmdNode, nodeX.getTargetPosition(), (nodeX.getStatusWord() & (1 << 9)) ? "TRUE" : "FALSE");

			WORD cobID = 0x0400 | cmdNode;

			gateway.SendCANFrame(cobID, 8, (uint8_t *)&cmd);
		}

		//gateway.SendCANFrame(0x80, 0, (uint8_t *)&cmd);

		for (uint8_t cmdNode = 1; cmdNode < 7; ++cmdNode) {
			epos nodeX(gateway, cmdNode);
			printf("0x%04x -> 0x%04x remote? %s\n", (int) cmdNode, nodeX.getTargetPosition(), (nodeX.getStatusWord() & (1 << 9)) ? "TRUE" : "FALSE");
		}

		gateway.close();
	} catch (fe_canopen_error & error) {
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
