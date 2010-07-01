// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_on_track
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_SARKOFAG_H)
#define _ECP_R_SARKOFAG_H

#include "base/ecp/ecp_robot.h"
#include "robot/sarkofag/sarkofag_const.h"

#include "base/kinematics/kinematics_manager.h"
#include "robot/sarkofag/kinematic_model_sarkofag.h"

namespace mrrocpp {
namespace ecp {
namespace sarkofag {

// ---------------------------------------------------------------
class robot : public common::ecp_robot, public kinematics::common::kinematics_manager
{
	// Klasa dla robota irp6_on_track
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na torze.
	virtual void create_kinematic_models_for_given_robot(void);

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

}; // end: class ecp_irp6_on_track_robot
// ---------------------------------------------------------------

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
