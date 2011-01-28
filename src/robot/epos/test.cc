#include <iostream>
#include <boost/exception/get_error_info.hpp>
#include <sys/time.h>

#include "epos_access_usb.h"
#include "epos.h"

using namespace mrrocpp::edp::epos;

int main(int argc, char *argv[])
{
	epos_access_usb gateway;

	epos node4(gateway, 4);
	epos node5(gateway, 5);
	epos node6(gateway, 6);

	try {
		gateway.open();

		node4.printEPOSstate();

		node4.reset();
		node5.reset();
		node6.reset();

		node4.printEPOSstate();

		printf("synchronized? %s\n", node4.isReferenced() ? "TRUE" : "FALSE");

		// switch to homing mode
		node4.setOpMode(epos::OMD_HOMING_MODE);
		node5.setOpMode(epos::OMD_HOMING_MODE);
		node6.setOpMode(epos::OMD_HOMING_MODE);

		// Do homing
		node4.startHoming();
		node5.startHoming();
		node6.startHoming();

		for(;;) {
			bool node4ok = node4.isHomingFinished();
			bool node5ok = node5.isHomingFinished();
			bool node6ok = node6.isHomingFinished();

			if(node4ok && node5ok && node6ok) {
				break;
			}
		};

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
