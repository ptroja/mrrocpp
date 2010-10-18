#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "ecp_mp_t_swarmitfix.h"

#include "robot/shead/ecp_r_shead.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_sleep.h"
#include "ecp_g_shead.h"
#include "ecp_t_shead.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/ecp_mp_g_sleep.h"
#include "ecp_mp_g_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace task {

// KONSTRUKTORY
swarmitfix::swarmitfix(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = new robot(*this);

	gt = new common::generator::transparent(*this);
	//sg = new common::generator::smooth(*this, true);
	g_sleep = new common::generator::sleep(*this);
	g_head_soldify = new generator::head_soldify(*this);
	g_head_desoldify = new generator::head_desoldify(*this);
	g_head_vacuum_on = new generator::head_vacuum_on(*this);
	g_head_vacuum_off = new generator::head_vacuum_off(*this);

	sr_ecp_msg->message("ecp shead loaded");
}

void swarmitfix::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TRANSPARENT) {
		gt->throw_kinematics_exceptions = (bool) mp_command.ecp_next_state.mp_2_ecp_next_state_variant;
		gt->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {
		std::string path(mrrocpp_network_path);
		path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;

		switch ((ecp_mp::task::SMOOTH_MOTION_TYPE) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
		{
			case ecp_mp::task::RELATIVE:
				//sg->set_relative();
				break;
			case ecp_mp::task::ABSOLUTE:
				//sg->set_absolute();
				break;
			default:
				break;
		}

		//sg->load_file_with_path(path.c_str());
		//sg->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_SLEEP) {
		g_sleep->init_time(mp_command.ecp_next_state.mp_2_ecp_next_state_variant);
		g_sleep->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::shead::generator::ECP_GEN_HEAD_SOLDIFY) {
		g_head_soldify->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::shead::generator::ECP_GEN_HEAD_DESOLDIFY) {
		g_head_desoldify->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::shead::generator::ECP_GEN_VACUUM_ON) {
		g_head_vacuum_on->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::shead::generator::ECP_GEN_VACUUM_OFF) {
		g_head_vacuum_off->Move();
	}

}

}
} // namespace shead

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new shead::task::swarmitfix(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
