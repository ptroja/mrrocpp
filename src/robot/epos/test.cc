#include <iostream>
#include <boost/exception/get_error_info.hpp>
#include <sys/time.h>

#include "epos_access_usb.h"
#include "epos.h"

using namespace mrrocpp::edp::epos;

int main(int argc, char *argv[])
{
	epos_access_usb gateway;

	epos node1(gateway, 1);
	epos node2(gateway, 6);

	try {
		gateway.open();

		for (int i = 0; i < 100; ++i) {
			struct timeval tv1, tv2;

			gettimeofday(&tv1, NULL);
			int32_t homepos = 0;// e.readHomePosition();
			uint16_t sw = node2.readSWversion();
			gettimeofday(&tv2, NULL);

			const double delta = (tv2.tv_sec + tv2.tv_usec / 1e6) - (tv1.tv_sec + tv1.tv_usec / 1e6);

			printf("%.6f sec: home %d sw %04x\n", delta, homepos, sw);
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
	}

	return 0;
}
