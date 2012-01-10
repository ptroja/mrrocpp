/*!
 * @file smb_move_test.cc
 * @brief File for testing of the smb motion.
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
	int motor, position;
	try {
		// Check arguments.
		if (argc != 3)
			throw 1;

		// Try to convert motor.
		istringstream iss1(argv[1]);
		iss1 >> motor;
		cout << "MOTOR:" << motor << endl;
		if ((motor !=8) && (motor != 9))
			throw 2;

		// Try to convert position.
		istringstream iss2(argv[2]);
		iss2 >> position;
		cout << "ROTATION:" << position << endl;
	} catch (...) {
		cout << "Usage: smb_move_test MOTOR ROTATION - rotates the given motor to given relative position."
				<< "\n\t MOTOR - number of EPOS controlling the motor, currently only 8 (LEGS) and 9 (PKM) are allowed."
				<< "\n\t ROTATION - number of motor increments." << endl;
		return 1;
	}

	// Create gateway.
	gateway_epos_usb gateway;

	try {
		// Open gateway.
		gateway.open();

		// Create node related to the given epos maxon controller.
		epos node(gateway, motor);

		// Print state.
		node.printState();

		// Check if in a FAULT state.
		if (node.getState() == 11) {
			// Print errors.
			UNSIGNED8 errNum = node.getNumberOfErrors();
			std::cout << "readNumberOfErrors() = " << (int) errNum << std::endl;
			for (UNSIGNED8 i = 1; i <= errNum; ++i) {

				UNSIGNED32 errCode = node.getErrorHistory(i);

				std::cout << node.ErrorCodeMessage(errCode) << std::endl;
			}
			if (errNum > 0) {
				node.clearNumberOfErrors();
			}

			// Reset errors.
			node.setState(epos::FAULT_RESET);
		}

		// Change to the operational mode.
		node.reset();

		// Move to the relative position.
		node.moveRelative(position);

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

