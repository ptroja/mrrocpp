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
