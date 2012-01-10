
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_smb1.h"
#include "const_smb1.h"
#include "base/edp/reader.h"
#include "robot/smb/kinematic_model_smb.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace smb1 {

// Access to kinematic parameters.
#define PARAMS ((mrrocpp::kinematics::smb::model*)this->get_current_kinematic_model())

// Konstruktor.
effector::effector(common::shell &_shell) :
	smb::effector(_shell, lib::smb1::ROBOT_NAME)
{

}

void effector::synchronise(void)
{
#if(DEBUG_METHODS)
	cout << "smb1::effector::synchronise\n";
	cout.flush();
#endif
	try {
		if (robot_test_mode) {
			controller_state_edp_buf.is_synchronised = true;
			return;
		}

		controller_state_edp_buf.is_synchronised = false;

		// Step 1: Setup velocity control with analog setpoint.
		pkm_rotation_node->setOperationMode(maxon::epos::OMD_VELOCITY_MODE);

		// Velocity and acceleration limits.
		pkm_rotation_node->setMaxProfileVelocity(50);
		pkm_rotation_node->setMaxAcceleration(1000);

		// NOTE: We assume, that scaling and offset are already set in the EPOS2

		// Get the target offset.
		const maxon::INTEGER16 offset = 2300;

		// Start motion.
		pkm_rotation_node->setControlword(0x000F);

		// Enable analog velocity setpoint.
		pkm_rotation_node->setAnalogInputFunctionalitiesExecutionMask(false, true, false);

		// Setup timer for monitoring.
		boost::system_time wakeup = boost::get_system_time();

		// Loop until reaching zero offset.
		while(pkm_rotation_node->getAnalogInput1() != offset) {
			std::cout << std::dec <<
					"AnalogVelocitySetpoint = " << (int) pkm_rotation_node->getAnalogVelocitySetpoint() <<
					" AnalogInput = " << (int) pkm_rotation_node->getAnalogInput1() <<
					" offset " << ((int) offset) << std::endl;

			// Sleep for a constant period of time
			wakeup += boost::posix_time::milliseconds(5);

			boost::thread::sleep(wakeup);
		};

		// Disable analog velocity setpoint.
		pkm_rotation_node->setAnalogInputFunctionalitiesExecutionMask(false, false, false);

		// Restore velocity and acceleration limits.
		pkm_rotation_node->setMaxProfileVelocity(Vdefault[1]);
		pkm_rotation_node->setMaxAcceleration(Adefault[1]);

		// Step 2: Homing.
		// Activate homing mode.
		pkm_rotation_node->doHoming(maxon::epos::HM_INDEX_POSITIVE_SPEED, 9850);
		// Step-by-step homing in order to omit the offset setting (the value will be stored in the EPOS for every agent separatelly).
/*		pkm_rotation_node->setOperationMode(maxon::epos::OMD_HOMING_MODE);
		pkm_rotation_node->reset();
		pkm_rotation_node->startHoming();
		pkm_rotation_node->monitorHomingStatus();*/

		// Compute joints positions in the home position
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

		// Homing of the motor controlling the legs rotation - set current position as 0.
		//legs_rotation_node->doHoming(mrrocpp::edp::maxon::epos::HM_ACTUAL_POSITION, 0);
		legs_relative_zero_position = legs_rotation_node->getActualPosition();

		// Set *extended* limits for PKM rotation.
		axes[1]->setMinimalPositionLimit(PARAMS->lower_pkm_motor_pos_limits - 1000);
		axes[1]->setMaximalPositionLimit(PARAMS->upper_pkm_motor_pos_limits + 1000);

		// Check whether the synchronization was successful.
		check_controller_state();

		// Throw non-fatal error - if synchronization wasn't successful.
		if (!controller_state_edp_buf.is_synchronised) {
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_synchronization_unsuccessful());
		}

	} catch (mrrocpp::lib::exception::non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_NON_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::system_error & e_) {
		// Standard error handling.
		HANDLE_EDP_SYSTEM_ERROR(e_)
	} catch (...) {
		HANDLE_EDP_UNKNOWN_ERROR()
	}
}

}
// namespace smb


namespace common {

effector* return_created_efector(common::shell &_shell)
{
	return new smb1::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

