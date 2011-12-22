#include "edp_e_spkm2.h"
#include "const_spkm2.h"

namespace mrrocpp {
namespace edp {
namespace spkm2 {

// Debug executed methods.
#define DEBUG_METHODS 1
#include "debug.hpp"


effector::effector(common::shell &_shell) :
	spkm::effector(_shell, lib::spkm2::ROBOT_NAME)
{
	DEBUG_METHOD;

	// Set default motor velocities, accelerations and decelerations.
	Vdefault = { 5000UL, 5000UL, 5000UL, 5000UL, 5000UL, 5000UL };
	Adefault = { 30000UL, 30000UL, 30000UL, 30000UL, 15000UL, 30000UL };
	Ddefault = { 30000UL, 30000UL, 30000UL, 30000UL, 15000UL, 30000UL };

	MotorVmax = { 5000UL, 5000UL, 5000UL, 5000UL, 5000UL, 5000UL };
	MotorAmax = { 30000UL, 30000UL, 30000UL, 30000UL, 15000UL, 30000UL };

	if (!robot_test_mode) {
		// Create epos objects according to CAN ID-mapping.
		axisA = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 5);
		axisB = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 4);
		axisC = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 6);
		axis1 = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 3);
		axis2 = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 2);
		axis3 = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 1);

		// Collect axes into common array container.
		axes[0] = &(*axisA);
		axesNames[0] = "A";
		axes[1] = &(*axisB);
		axesNames[1] = "B";
		axes[2] = &(*axisC);
		axesNames[2] = "C";
		axes[3] = &(*axis1);
		axesNames[3] = "1";
		axes[4] = &(*axis2);
		axesNames[4] = "2";
		axes[5] = &(*axis3);
		axesNames[5] = "3";
		// Setup the axis array for the IPM handler
		{
			boost::unique_lock <boost::mutex> lock(ipm_handler.mtx);
			ipm_handler.axes = this->axes;
		}
	}
}

} // namespace spkm2


namespace common {

// Create spkm effector.
effector* return_created_efector(common::shell &_shell)
{
	return new spkm2::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
