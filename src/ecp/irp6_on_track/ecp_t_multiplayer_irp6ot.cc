#include <stdio.h>
#include <string.h>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_multiplayer.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/irp6_on_track/ecp_t_multiplayer_irp6ot.h"

#include "ecp/irp6_on_track/ecp_g_vis_sac_lx.h"
#include "ecp_mp/ecp_mp_s_vis_sac_lx.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

multiplayer::multiplayer(lib::configurator &_config) : task(_config)
{
	ecp_m_robot = new robot (*this);

	// powolanie czujnikow
	sensor_m[lib::SENSOR_CAMERA_SA] =
		new ecp_mp::sensor::vis_sac_lx (lib::SENSOR_CAMERA_SA, "[vsp_vis]", *this); //change if SENSOR_CAMERA_SA used for nonnn recog (vsp_vis_pbeolsac)

	// Konfiguracja wszystkich czujnikow

	for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	//powolanie generatorow
	befg = new common::generator::bias_edp_force (*this);

	sg = new common::generator::smooth (*this, true);
	wmg = new common::generator::weight_meassure(*this, -0.3, 2);
	gt = new common::generator::transparent (*this);

	go_st = new common::task::ecp_sub_task_gripper_opening(*this);

	takeg = new generator::vis_sac_lx (*this, 4);
	rgg = new common::generator::tff_rubik_grab (*this, 8);

	//przydzielenie czujnikow generatorom
	takeg->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP loaded");
}

void multiplayer::main_task_algorithm(void)
{
	for (;;) {

		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");

		switch ( (ecp_mp::task::RCSC_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
			case ecp_mp::task::ECP_WEIGHT_MEASURE_GENERATOR:
				wmg->Move();
				break;
			case ecp_mp::task::ECP_GEN_TRANSPARENT:
				gt->Move();
				break;
			case ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE:
				befg->Move();
				break;
			case ecp_mp::task::MULTIPLAYER_GRIPPER_OPENING:
				switch ( (ecp_mp::task::MULTIPLAYER_GRIPPER_OP) mp_command.ecp_next_state.mp_2_ecp_next_state_variant) {
					case ecp_mp::task::MULTIPLAYER_GO_VAR_1:
						go_st->configure(0.002, 1000);
						go_st->execute();
						break;
					case ecp_mp::task::MULTIPLAYER_GO_VAR_2:
						go_st->configure(0.02, 1000);
						go_st->execute();
						break;
					default:
						break;
				}
				break;
			case ecp_mp::task::ECP_GEN_SMOOTH:
			{
				std::string path(mrrocpp_network_path);
				path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;
				sg->load_file_with_path(path.c_str());
				//printf("\nTRACK ECP_GEN_SMOOTH :%s\n\n", path1);
				sg->Move();
				break;
			}
			default:
				break;

			case ecp_mp::task::ECP_GEN_TAKE_FROM_ROVER:
				sr_ecp_msg->message("MOVE");
				takeg->Move();
				sr_ecp_msg->message("STOP MOVE");
			break;

			case ecp_mp::task::ECP_GEN_GRAB_FROM_ROVER:
			sr_ecp_msg->message("GRAB");
				rgg->configure(0.057, 0.00005, 0);
				rgg->Move();
				sr_ecp_msg->message("STOP GRAB");
			break;
		}

		ecp_termination_notice();
	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot::task::multiplayer(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


