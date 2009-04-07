#include <stdio.h>
#include <string.h>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_multiplayer.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/irp6_on_track/ecp_t_multiplayer_irp6ot.h"

#include "ecp/irp6_on_track/ecp_g_vis_sac_lx.h"
#include "ecp_mp/ecp_mp_s_vis_sac_lx.h"

ecp_task_multiplayer_irp6ot::ecp_task_multiplayer_irp6ot(configurator &_config) :
	ecp_task(_config)
{
}

// methods for ECP template to redefine in concrete classes
void ecp_task_multiplayer_irp6ot::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);

	// powolanie czujnikow
	sensor_m[SENSOR_CAMERA_SA] =
		new ecp_mp::sensor::ecp_mp_vis_sac_lx_sensor (SENSOR_CAMERA_SA, "[vsp_vis]", *this); //change if SENSOR_CAMERA_SA used for nonnn recog (vsp_vis_pbeolsac)

	// Konfiguracja wszystkich czujnikow

	for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	//powolanie generatorow
	befg = new bias_edp_force_generator (*this);

	sg = new ecp_smooth_generator (*this, true);
	wmg = new weight_meassure_generator(*this, -0.3, 2);
	gt = new ecp_generator_t (*this);

	go_st = new ecp_sub_task_gripper_opening(*this);

	takeg = new ecp_vis_sac_lx_generator (*this, 4);
	rgg = new ecp_tff_rubik_grab_generator (*this, 8);

	//przydzielenie czujnikow generatorom
	takeg->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP loaded");
}

void ecp_task_multiplayer_irp6ot::main_task_algorithm(void)
{
	for (;;) {
		int size;
		char * path1;

		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");

		switch ( (RCSC_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
			case ECP_WEIGHT_MEASURE_GENERATOR:
				wmg->Move();
				break;
			case ECP_GEN_TRANSPARENT:
				gt->Move();
				break;
			case ECP_GEN_BIAS_EDP_FORCE:
				befg->Move();
				break;
			case MULTIPLAYER_GRIPPER_OPENING:
				switch ( (MULTIPLAYER_GRIPPER_OP) mp_command.ecp_next_state.mp_2_ecp_next_state_variant) {
					case MULTIPLAYER_GO_VAR_1:
						go_st->configure(0.002, 1000);
						go_st->execute();
						break;
					case MULTIPLAYER_GO_VAR_2:
						go_st->configure(0.02, 1000);
						go_st->execute();
						break;
					default:
						break;
				}
				break;
			case ECP_GEN_SMOOTH:
				size = strlen(mrrocpp_network_path)
						+ strlen(mp_command.ecp_next_state.mp_2_ecp_next_state_string) + 1;
				path1 = new char[size];
				// Stworzenie sciezki do pliku.
				sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				sg->load_file_with_path(path1);
				//printf("\nTRACK ECP_GEN_SMOOTH :%s\n\n", path1);
				delete[] path1;
				sg->Move();
				break;
			default:
				break;

			case ECP_GEN_TAKE_FROM_ROVER:
				sr_ecp_msg->message("MOVE");
				takeg->Move();
				sr_ecp_msg->message("STOP MOVE");
			break;

			case ECP_GEN_GRAB_FROM_ROVER:
			sr_ecp_msg->message("GRAB");
				rgg->configure(0.057, 0.00005, 0);
				rgg->Move();
				sr_ecp_msg->message("STOP GRAB");
			break;
		}

		ecp_termination_notice();
	}
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_task_multiplayer_irp6ot(_config);
}
