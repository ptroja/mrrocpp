// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_SHEAD_H)
#define _ECP_R_SHEAD_H

#include "base/ecp/ecp_robot.h"
#include "robot/shead/const_shead.h"
#include "base/kinematics/kinematics_manager.h"
#include "robot/shead/kinematic_model_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {

// ---------------------------------------------------------------
class robot : public common::ecp_robot, public kinematics::common::kinematics_manager
{
	// Klasa dla robota irp6_postument (sztywnego)

protected:
	//bufory wejsciowe z generatora
	//shead.h
	lib::single_thread_port <lib::SHEAD_HEAD_SOLIDIFICATION> shead_head_soldification_data_port;
	lib::SHEAD_HEAD_SOLIDIFICATION shead_head_soldification_structure;

	lib::single_thread_port <lib::SHEAD_VACUUM_ACTIVATION> shead_vacuum_activation_data_port;
	lib::SHEAD_VACUUM_ACTIVATION shead_vacuum_activation_structure;

	// bufory wyjsciowe do generatora
	// shead.h
	lib::single_thread_request_port <lib::shead_reply> shead_reply_data_request_port;
	lib::shead_reply shead_reply_structure;

	// bufory do edp
	lib::shead_cbuffer ecp_edp_cbuffer;
	lib::shead_rbuffer edp_ecp_rbuffer;

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
