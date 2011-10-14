/*!
 * @file smb_synchro_test.cc
 * @brief File for testing of the smb synchronization
 *
 * @author tkornuta
 * @date 14-10-2011
 */

#include <iostream>
#include <sstream>
#include <boost/exception/get_error_info.hpp>
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <sys/time.h>

#include "robot/canopen/gateway_epos_usb.h"
#include "epos.h"

using namespace mrrocpp::edp::canopen;
using namespace mrrocpp::edp::maxon;
using namespace std;

int main(int argc, char *argv[])
{
	int mode;
	// Check number of arguments.
	try {
		// Check arguments.
		if (argc > 2)
			throw;

		// Try to convert mode.
		istringstream iss(argv[1]);
		iss >> mode;
		if ((mode <0)||(mode>2))
			throw;
	} catch (...) {
		cout << "Usage: smb_synchro_test X - test synchronization."
				<< "\n\t X=0 - only prints potentiometer readings."
				<< "\n\t X=1 - only first step - movement based on the potentiometer readings."
				<< "\n\t X=2 - full synchronization." << endl;
		exit(-1);
	}

	// Create gateway.
	gateway_epos_usb gateway;

	try {
		// Open gateway.
		gateway.open();

		// Create node related to the 9th epos maxon controller.
		epos node(gateway, 9);

		// Print state.
		node.printEPOSstate();

		// Check if in a FAULT state.
		if (node.checkEPOSstate() == 11) {
			// Print errors.
			UNSIGNED8 errNum = node.readNumberOfErrors();
			std::cout << "readNumberOfErrors() = " << (int) errNum << std::endl;
			for (UNSIGNED8 i = 1; i <= errNum; ++i) {

				UNSIGNED32 errCode = node.readErrorHistory(i);

				std::cout << node.ErrorCodeMessage(errCode) << std::endl;
			}
			if (errNum > 0) {
				node.clearNumberOfErrors();
			}

			// Reset errors.
			node.changeEPOSstate(epos::FAULT_RESET);
		}

		// Change to the operational mode.
		node.reset();

		// Get current potentiometer readings.
		int pot = node.readAnalogInput1();

		// Set coefficients.
		const double p1 = -0.0078258336;
		const double p2 = 174.7796278191;
		const double p3 = -507883.404901415;

		// Compute desired position.
		int position = -(pot * pot * p1 + pot * p2 + p3) - 120000;
		cout << "Retrieved potentiometer reading: " << pot << "\nComputed pose: " << position << endl;

		// Move to the relative position.
		if(mode>0) {
			node.moveRelative(position);
			while (!node.isTargetReached())
				usleep(200000);
				cout<< "~2.5V Position reached!\n";
		}
		usleep(2000000);



		// Activate homing mode.
		if(mode>1)
			node.doHoming(epos::HM_INDEX_NEGATIVE_SPEED, 0);

		// Close gateway.
		gateway.close();
	} catch (canopen_error & error) {
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

