#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
//#include "ecp_mp/task/ecp_mp_t_legobrick.h"
//#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "ecp/irp6_postument/ecp_t_legobrick_irp6p.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

lego_brick::lego_brick(lib::configurator &_config) :
	task(_config)
{
    // the robot is choose dependendat on the section of configuration file sent as argv[4]
    ecp_m_robot = new robot (*this);

    befg = new common::generator::bias_edp_force (*this);
    sg = new common::generator::smooth (*this, true);
    afg = new common::generator::legobrick_attach_force(*this, 10);
    dfg = new common::generator::legobrick_detach_force(*this, 10);

    sr_ecp_msg->message("ECP loaded");
}

void lego_brick::main_task_algorithm(void)
{
	lib::POSE_SPECIFICATION ps = lib::JOINT;

	double coordinates[MAX_SERVOS_NR];
	double vp[MAX_SERVOS_NR];
	double vk[MAX_SERVOS_NR];
	double v[MAX_SERVOS_NR];
	double a[MAX_SERVOS_NR];

	coordinates[0] = -0.100633;
	coordinates[1] = -1.341654;
	coordinates[2] = 0.048600;
	coordinates[3] = 0.000002;
	coordinates[4] = 4.609996;
	coordinates[5] = 0.999997;
	coordinates[6] = 0.067666;
	coordinates[7] = 0.0;
	for(int i = 0; i < 7; ++i){
		vp[i] = 0.0;
		vk[i] = 0.0;
		v[i] = 0.1;
		a[i] = 0.02;
	}
	sg->insert_pose_list_element(ps, vp, vk, v, a, coordinates);

	// generator oparty na detekcji sily
	// legobrick_detach_force_generator force_gen(*this, 10);
	sr_ecp_msg->message("Ruch smooth");
	sg->Move();
	sr_ecp_msg->message("Ruch bias");
	befg->Move();
	sr_ecp_msg->message("Ruch attach");
	afg->Move();
	sr_ecp_msg->message("Ruch detach");
	dfg->Move();
	sr_ecp_msg->message("Ruch smooth");
	sg->Move();

    /*
	for(;;)
	{
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state ();

		sr_ecp_msg->message("Order received");

		switch ( (RCSC_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
		{
		case ECP_WEIGHT_MEASURE_GENERATOR:
			wmg->Move();
			break;
		case ECP_GEN_TRANSPARENT:
			gt->Move();
			break;
		case ECP_GEN_BIAS_EDP_FORCE:
			befg->Move();
			break;
		case ECP_GEN_TFF_NOSE_RUN:
			nrg->Move();
			break;
		case ECP_GEN_TFF_RUBIK_GRAB:
			switch ( (RCSC_RUBIK_GRAB_PHASES) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
			{
			case RCSC_RG_FROM_OPEARTOR_PHASE_1:
				rgg->configure(0.057, 0.00005, 0);
				break;
			case RCSC_RG_FROM_OPEARTOR_PHASE_2:
				rgg->configure(0.057, 0.00005, 50);
				break;
			case RCSC_RG_FCHANGE_PHASE_1:
				rgg->configure(0.072, 0.00005, 0, false);
				break;
			case RCSC_RG_FCHANGE_PHASE_2:
				rgg->configure(0.062, 0.00005, 0);
				break;
			case RCSC_RG_FCHANGE_PHASE_3:
				rgg->configure(0.057, 0.00005, 0);
				break;
			case RCSC_RG_FCHANGE_PHASE_4:
				rgg->configure(0.057, 0.00005, 50);
				break;
			default:
				break;
			}
			rgg->Move();
			break;
		case ECP_GEN_TFF_GRIPPER_APPROACH:
			gag->configure(0.01 , 150);
			gag->Move();
			break;
		case ECP_GEN_TFF_RUBIK_FACE_ROTATE:
			switch ( (RCSC_TURN_ANGLES) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
			{
			case RCSC_CCL_90:
				rfrg->configure(-90.0);
				break;
			case RCSC_CL_0:
				rfrg->configure(0.0);
				break;
			case RCSC_CL_90:
				rfrg->configure(90.0);
				break;
			case RCSC_CL_180:
				rfrg->configure(180.0);
				break;
			default:
				break;
			}
			rfrg->Move();
			break;
		case RCSC_GRIPPER_OPENING:
			switch ( (RCSC_TURN_ANGLES) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
			{
			case RCSC_GO_VAR_1:
				go_st->configure(0.002, 1000);
				go_st->execute();
				break;
			case RCSC_GO_VAR_2:
				go_st->configure(0.02, 1000);
				go_st->execute();
				break;
			default:
				break;
			}
			break;
		case ECP_GEN_TEACH_IN:
			size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			path1 = new char[size];
			// Stworzenie sciezki do pliku.
			strcpy(path1, mrrocpp_network_path);
			sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			tig->flush_pose_list();
			tig->load_file_with_path (path1);
			//		printf("\nTRACK ECP_GEN_TEACH_IN :%s\n\n", path1);
			tig->initiate_pose_list();
			delete[] path1;
			tig->Move();
			break;
		case ECP_GEN_SMOOTH:
			size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			path1 = new char[size];
			// Stworzenie sciezki do pliku.
			strcpy(path1, mrrocpp_network_path);
			sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			sg->load_file_with_path (path1);
			//				printf("\nTRACK ECP_GEN_SMOOTH :%s\n\n", path1);
			delete[] path1;
			sg->Move();
			break;
		default:
			break;
		}

		ecp_termination_notice();

	} //end for
*/

}

}
} // namespace irp6p

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6p::task::lego_brick(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
