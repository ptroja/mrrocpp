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

		// Check if in a FAULT state
		if(node4.checkEPOSstate() == 11) {
			UNSIGNED8 errNum = node4.readNumberOfErrors();
			std::cout << "readNumberOfErrors() = " << (int) errNum << std::endl;
			for(UNSIGNED8 i = 1; i <= errNum; ++i) {

				UNSIGNED32 errCode = node4.readErrorHistory(i);

				std::cout << epos::ErrorCodeMessage(errCode) << std::endl;
			}
		}

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
