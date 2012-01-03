#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp_e_smb2.h"
#include "const_smb2.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/smb/kinematic_model_smb.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace smb2 {

// Access to kinematic parameters.
#define PARAMS ((mrrocpp::kinematics::smb::model*)this->get_current_kinematic_model())

// Konstruktor.
effector::effector(common::shell &_shell) :
	smb::effector(_shell, lib::smb2::ROBOT_NAME)
{

}

int effector::relativeSynchroPosition(maxon::epos & node)
{
	// Wakeup time.
	boost::system_time wakeup;

	// Setup the wakeup time.
	wakeup = boost::get_system_time();

	// Set voltage-to-position interpolation coefficients.
	const double p1 = -0.0078258336;
	const double p2 = 174.7796278191;
	const double p3 = -507883.404901415;

	// Number of readings.
	const unsigned int filter = 7;
	std::vector <int> potTable(filter);

	// Get current potentiometer readings.
	for (int i = 0; i < filter; ++i) {
		potTable[i] = node.getAnalogInput1();
		// Increment the wakeup time
		wakeup += boost::posix_time::milliseconds(5);
		// Wait for device state to change
		boost::thread::sleep(wakeup);
	}
	// Sort readings table.
	std::sort(potTable.begin(), potTable.end());
	// Compute mean value.
	double pot = (potTable[2] + potTable[3] + potTable[4]) / 3.0;

	// Compute desired position.
	int position = -(pot * pot * p1 + pot * p2 + p3) - 120000;
	// Return computed position
	return position;
}

void effector::synchronise(void)
{
#if(DEBUG_METHODS)
	std::cout << "smb2::effector::synchronise\n";
	std::cout.flush();
#endif
	try {
		// TEMPORARY
		controller_state_edp_buf.is_synchronised = true;
		return;
		// END OF TEMPORARY
		if (robot_test_mode) {
			controller_state_edp_buf.is_synchronised = true;
			return;
		}

		controller_state_edp_buf.is_synchronised = false;
		// Two-step synchronization of the motor rotating the whole PKM.
		// Step1: Potentiometer.
		int position;
		// Compute desired position.
		position = relativeSynchroPosition(*pkm_rotation_node);
		std::cout << "Computed pose: " << position << std::endl;

		// Set "safe" velocity and acceleration values.
		pkm_rotation_node->setProfileVelocity(500);
		pkm_rotation_node->setProfileAcceleration(100);
		pkm_rotation_node->setProfileDeceleration(100);

		// Loop.
		do {
			// Move to the relative position.
			pkm_rotation_node->moveRelative(position);

			// Wakeup time.
			boost::system_time wakeup;

			// Setup the wakeup time.
			wakeup = boost::get_system_time();

			// Wait until end of the motion.
			while (!pkm_rotation_node->isTargetReached()) {
				// Sleep for a constant period of time
				wakeup += boost::posix_time::milliseconds(5);

				boost::thread::sleep(wakeup);
			}

			// Get current position.
			position = relativeSynchroPosition(*pkm_rotation_node);
		} while (abs(position) > 100);

		// Step2: Homing.
		// Activate homing mode.
		//pkm_rotation_node->doHoming(maxon::epos::HM_INDEX_NEGATIVE_SPEED, -1970);
		// Step-by-step homing in order to omit the offset setting (the value will be stored in the EPOS for every agent separatelly).
		pkm_rotation_node->setOperationMode(maxon::epos::OMD_HOMING_MODE);
		pkm_rotation_node->reset();
		pkm_rotation_node->startHoming();
		pkm_rotation_node->monitorHomingStatus();

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
	return new smb2::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

