#include <iostream>
#include <boost/exception/get_error_info.hpp>
#include <sys/time.h>

#include "epos_access_usb.h"
#include "epos.h"

using namespace mrrocpp::edp::epos;

int main(int argc, char *argv[])
{
	epos_access_usb gateway;

	epos node4(gateway, 5);

	try {
		gateway.open();

		node4.printEPOSstate();

		node4.reset();

		node4.printEPOSstate();

		printf("synchronized? %s\n", node4.isReferenced() ? "TRUE" : "FALSE");

		// Do homing
		node4.doHoming(epos::HM_CURRENT_THRESHOLD_POSITIVE_SPEED, -10000);

		printf("synchronized? %s\n", node4.isReferenced() ? "TRUE" : "FALSE");

		// Move back
		//node4.moveRelative(-50000);
		//node4.monitorStatus();

		gateway.close();
	} catch (epos_error & error) {
		std::cerr << "EPOS Error." << std::endl;

		if ( std::string const * r = boost::get_error_info<reason>(error) )
			std::cerr << " Reason: " << *r << std::endl;

		if ( std::string const * call = boost::get_error_info<errno_call>(error) )
			std::cerr << " Errno call: " << *call << std::endl;

		if ( int const * errno_value = boost::get_error_info<errno_code>(error) )
			std::cerr << "Errno value: " << *errno_value << std::endl;
	}

	return 0;
}
