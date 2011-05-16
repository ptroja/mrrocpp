#include <iostream>
#include <boost/exception/get_error_info.hpp>
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <sys/time.h>

#include "epos_access_usb.h"
#include "epos.h"

using namespace mrrocpp::edp::epos;

int main(int argc, char *argv[])
{
	epos_access_usb gateway;

	try {
		// timestamps variables
		struct timespec t1, t2;

		gateway.open();

		epos node(gateway, 1);

		node.printEPOSstate();
//			node.SendNMTService(epos::Reset_Node);
//			usleep(500000);
//			node.SendNMTService(epos::Start_Remote_Node);
//			usleep(500000);

		// Check if in a FAULT state
		if(node.checkEPOSstate() == 11) {
			UNSIGNED8 errNum = node.readNumberOfErrors();
			std::cout << "readNumberOfErrors() = " << (int) errNum << std::endl;
			for(UNSIGNED8 i = 1; i <= errNum; ++i) {

				UNSIGNED32 errCode = node.readErrorHistory(i);

				std::cout << epos::ErrorCodeMessage(errCode) << std::endl;
			}
			if (errNum > 0) {
				node.clearNumberOfErrors();
			}
			node.changeEPOSstate(epos::FAULT_RESET);
		}

		// Change to the operational mode
		node.reset();

		node.clearPvtBuffer();

		uint16_t status;

		status = node.readStatusWord();
		std::cout << "node.remote = " << (int) (epos::isRemoteOperationEnabled(status)) << std::endl;

		gateway.SendNMTService(1, epos_access::Start_Remote_Node);

		status = node.readStatusWord();
		std::cout << "node.remote = " << (int) (epos::isRemoteOperationEnabled(status)) << std::endl;

		std::cout << "node.readActualBufferSize() = " << (int) node.readActualBufferSize() << std::endl;

		gateway.setDebugLevel(0);
		clock_gettime(CLOCK_MONOTONIC, &t1);
		node.writeInterpolationDataRecord(0, 0, 0);
		clock_gettime(CLOCK_MONOTONIC, &t2);
		double t = (t2.tv_sec + t2.tv_nsec/1e9) - (t1.tv_sec + t1.tv_nsec/1e9);
		printf("%.9f\n", t);
//		node.writeInterpolationDataRecord(1, 2, 3);
//		node.writeInterpolationDataRecord(1, 2, 3);
		gateway.setDebugLevel(0);

		std::cout << "node.readActualBufferSize() = " << (int) node.readActualBufferSize() << std::endl;

		gateway.close();
	} catch (epos_error & error) {
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
