#if !defined(_ECP_R_SHEAD_H)
#define _ECP_R_SHEAD_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "base/ecp/ecp_robot.h"
#include "robot/shead/const_shead.h"
#include "base/kinematics/kinematics_manager.h"
#include "robot/shead/kinematic_model_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {

// ---------------------------------------------------------------
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{
	// Klasa dla robota irp6_postument (sztywnego)

protected:
	//bufory wejsciowe z generatora
	//shead.h
	lib::single_thread_port <lib::shead::HEAD_SOLIDIFICATION> shead_head_soldification_data_port;

	lib::single_thread_port <lib::shead::VACUUM_ACTIVATION> shead_vacuum_activation_data_port;

	// bufory wyjsciowe do generatora
	// shead.h
	lib::single_thread_request_port <lib::shead::reply> shead_reply_data_request_port;

	// bufory do edp
	lib::shead::cbuffer ecp_edp_cbuffer;
	lib::shead::rbuffer edp_ecp_rbuffer;

	void create_kinematic_models_for_given_robot(void);

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

	void create_command();
	void get_reply();

}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
