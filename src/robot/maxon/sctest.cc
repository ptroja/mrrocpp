/*
 * SocketCAN interface test
 *
 *  Created on: May 16, 2011
 *      Author: ptroja
 */

#include <iostream>
#include <boost/exception/get_error_info.hpp>
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <sys/time.h>

#include "robot/canopen/gateway_socketcan.h"
#include "robot/canopen/gateway_epos_usb.h"
#include "epos.h"

using namespace mrrocpp::edp::canopen;
using namespace mrrocpp::edp::maxon;

int main(int argc, char *argv[])
{
	gateway_socketcan gateway("can0");
	//gateway_usb gateway;

	clockid_t clk_id = CLOCK_REALTIME;

	struct timespec t1, t2;

	if(clock_getres(clk_id, &t1) == -1) {
		perror("clock_getres()");
	} else {
		printf("clock resolution is %ld.%09ld secs\n", t1.tv_sec, t1.tv_nsec);
	}

	try {
		gateway.open();

		WORD ans[4];

		epos node5(gateway, 5);

		clock_gettime(clk_id, &t1);
		for(int i = 0; i < 200; ++i) {
		gateway.ReadObject(ans, 4, 5, 0x2003, 0x01);
		}
		clock_gettime(clk_id, &t2);

		double t = (t2.tv_sec + t2.tv_nsec/1e9) - (t1.tv_sec + t1.tv_nsec/1e9);
		printf("%.9f\n", t);

		printf("software version: 0x%04X\n", ans[3]);

//		epos node1(gateway, 5);
//		printf("software version: 0x%04X\n", node1.readSWversion());

		gateway.close();
	} catch (se_canopen_error & error) {
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
