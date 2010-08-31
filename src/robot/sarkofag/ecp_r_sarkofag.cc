/*!
 * @file
 * @brief File contains ecp robot class definition for Sarkofag
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sarkofag
 */

#include "robot/sarkofag/ecp_r_sarkofag.h"

namespace mrrocpp {
namespace ecp {
namespace sarkofag {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	robot::ecp_robot(lib::sarkofag::ROBOT_NAME, lib::sarkofag::NUM_OF_SERVOS, lib::sarkofag::EDP_SECTION, _config, _sr_ecp),
			kinematics_manager()
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task& _ecp_object) :
	robot::ecp_robot(lib::sarkofag::ROBOT_NAME, lib::sarkofag::NUM_OF_SERVOS, lib::sarkofag::EDP_SECTION, _ecp_object),
			kinematics_manager()
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota sarkofag.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::sarkofag::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);

}

} // namespace sarkofag
} // namespace ecp
} // namespace mrrocpp


