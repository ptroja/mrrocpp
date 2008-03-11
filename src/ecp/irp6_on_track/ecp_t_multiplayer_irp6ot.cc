#include <stdio.h>
#include <string.h>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_multiplayer.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/irp6_on_track/ecp_t_multiplayer_irp6ot.h"

ecp_task_multiplayer_irp6ot::ecp_task_multiplayer_irp6ot(configurator &_config) :
	ecp_task(_config)
{
}

ecp_task_multiplayer_irp6ot::~ecp_task_multiplayer_irp6ot()
{
}

// methods for ECP template to redefine in concrete classes
void ecp_task_multiplayer_irp6ot::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);

	// powolanie czujnikow


	befg = new bias_edp_force_generator (*this);

	sg = new ecp_smooth_generator (*this, true);
	wmg = new weight_meassure_generator(*this, 1);
	gt = new ecp_generator_t (*this);

	go_st = new ecp_sub_task_gripper_opening(*this);

	sr_ecp_msg->message("ECP loaded");
}

void ecp_task_multiplayer_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP multiplayer irp6ot  - wcisnij start");
	ecp_wait_for_start();
	for (;;) { // Wewnetrzna petla nieskonczona

		for (;;) {
			int size;
			char * path1;

			sr_ecp_msg->message("Waiting for MP order");

			get_next_state();

			sr_ecp_msg->message("Order received");

			switch ( (RCSC_ECP_STATES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state) {
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
					switch ( (MULTIPLAYER_GRIPPER_OP) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_variant) {
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
							+ strlen(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string) + 1;
					path1 = new char[size];
					// Stworzenie sciezki do pliku.
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					sg->load_file_with_path(path1);
					//printf("\nTRACK ECP_GEN_SMOOTH :%s\n\n", path1);
					delete[] path1;
					sg->Move();
					break;
				default:
					break;
			}

			ecp_termination_notice();

		}

		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop();
		break;
	}
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_task_multiplayer_irp6ot(_config);
}
