#include "edp_e_spkm2.h"
#include "const_spkm2.h"
#include "kinematic_model_spkm.h"
#include "kinematic_parameters_spkm2.h"

namespace mrrocpp {
namespace edp {
namespace spkm2 {

#include "base/lib/debug.hpp"


effector::effector(common::shell &_shell) :
	spkm::effector(_shell, lib::spkm2::ROBOT_NAME)
{
	DEBUG_METHOD;

	// Set default motor velocities, accelerations and decelerations for axis 0 - leg A.
	Vdefault[0] = 5000UL;
	MotorVmax[0] = 5000UL;
	Adefault[0] = 30000UL;
	Ddefault[0] = 30000UL;
	MotorAmax[0] = 30000UL;

	// Set default motor velocities, accelerations and decelerations for axis 1 - leg B.
	Vdefault[1] = 5000UL;
	MotorVmax[1] = 5000UL;
	Adefault[1] = 30000UL;
	Ddefault[1] = 30000UL;
	MotorAmax[1] = 30000UL;

	// Set default motor velocities, accelerations and decelerations for axis 2 - leg C.
	Vdefault[2] = 5000UL;
	MotorVmax[2] = 5000UL;
	Adefault[2] = 30000UL;
	Ddefault[2] = 30000UL;
	MotorAmax[2] = 30000UL;

	// Set default motor velocities, accelerations and decelerations for axis 3 - (lower wrist rotation - "axis 1").
	Vdefault[3] = 5000UL;
	MotorVmax[3] = 5000UL;
	Adefault[3] = 10000UL;
	Ddefault[3] = 10000UL;
	MotorAmax[3] = 10000UL;

	// Set default motor velocities, accelerations and decelerations for axis 4 - (lower wrist rotation - "axis 2").
	Vdefault[4] = 3000UL;
	MotorVmax[4] = 3000UL;
	Adefault[4] = 15000UL;
	Ddefault[4] = 15000UL;
	MotorAmax[4] = 15000UL;

	// Set default motor velocities, accelerations and decelerations for axis 5 - (lower wrist rotation - "axis 3").
	Vdefault[5] = 5000UL;
	MotorVmax[5] = 5000UL;
	Adefault[5] = 30000UL;
	Ddefault[5] = 30000UL;
	MotorAmax[5] = 30000UL;

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

	// Create SPKM kinematics.
	create_kinematic_models_for_given_robot();
}

void effector::create_kinematic_models_for_given_robot(void)
{
DEBUG_METHOD;

	// Add main SPKM kinematics.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm(kinematics::spkm2::kinematic_parameters_spkm2()));
	// Set active model
	set_kinematic_model(0);
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
