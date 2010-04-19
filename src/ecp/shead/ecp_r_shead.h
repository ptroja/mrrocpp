// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_SHEAD_H)
#define _ECP_R_SHEAD_H

#include "ecp/common/ecp_robot.h"
#include "lib/robot_consts/shead_const.h"
#include "kinematics/common/kinematics_manager.h"
#include "kinematics/shead/kinematic_model_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {

// ---------------------------------------------------------------
class robot: public common::ecp_robot,
		public kinematics::common::kinematics_manager {
	// Klasa dla robota irp6_postument (sztywnego)

protected:
	void create_kinematic_models_for_given_robot(void);

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
