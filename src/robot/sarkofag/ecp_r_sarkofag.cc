// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - sarkofag
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "robot/sarkofag/ecp_r_sarkofag.h"

namespace mrrocpp {
namespace ecp {
namespace sarkofag {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp_robot(lib::ROBOT_SARKOFAG, SARKOFAG_NUM_OF_SERVOS, EDP_SARKOFAG_SECTION, _config, _sr_ecp),
			kinematics_manager()
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task& _ecp_object) :
	ecp_robot(lib::ROBOT_SARKOFAG, SARKOFAG_NUM_OF_SERVOS, EDP_SARKOFAG_SECTION, _ecp_object),
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


