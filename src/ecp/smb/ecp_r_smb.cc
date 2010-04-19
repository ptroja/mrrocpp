// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/smb/ecp_r_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp_robot(lib::ROBOT_SMB, SMB_NUM_OF_SERVOS, EDP_SMB_SECTION, _config,
			_sr_ecp),
			kinematics_manager() {
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}
;
robot::robot(common::task::task& _ecp_object) :
	ecp_robot(lib::ROBOT_SMB, SMB_NUM_OF_SERVOS, EDP_SMB_SECTION, _ecp_object),
	kinematics_manager() {
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void) {
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::smb::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

