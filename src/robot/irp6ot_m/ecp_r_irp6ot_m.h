#if !defined(_ECP_R_IRP6OT_M_H)
#define _ECP_R_IRP6OT_M_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for IRp6 on track manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_m
 */

#include "base/ecp/ecp_robot.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"

#include "base/kinematics/kinematics_manager.h"
#include "robot/irp6ot_m/kinematic_model_irp6ot_with_track.h"
#include "robot/irp6ot_m/kinematic_model_calibrated_irp6ot_with_wrist.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {

// ---------------------------------------------------------------
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
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
