// ------------------------------------------------------------------------
//             mp_m_pr.cc - powielanie rysunku - wersja wielorobotowa
// 
//                      MASTER PROCESS (MP) - main()
// 
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_vis_sac_lx.h"
#include "mp/mp_g_vis_sac_lx.h"
#include "mp/mp_g_force.h"
#include "ecp_mp/ecp_mp_s_vis_sac_lx.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

namespace mrrocpp {
namespace mp {
namespace task {

mp_task* return_created_mp_task(configurator &_config)
{
	return new mp_task_vis_sac_lx(_config);
}

mp_task_vis_sac_lx::mp_task_vis_sac_lx(configurator &_config) :
	mp_task(_config)
{
}

// methods fo mp template to redefine in concete class
void mp_task_vis_sac_lx::task_initialization(void)
{
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] = new ecp_mp::sensor::schunk (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

	sensor_m[SENSOR_CAMERA_SA] = new ecp_mp::sensor::vis_sac_lx (SENSOR_CAMERA_SA, "[vsp_vis]", *this); //change if SENSOR_CAMERA_SA used for nonnn recog (vsp_vis_pbeolsac)

	// Konfiguracja wszystkich czujnikow	

	for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
			!= sensor_m.end(); sensor_m_iterator++) {
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);
	sr_ecp_msg->message("MP vis lx loaded");
}
;

void mp_task_vis_sac_lx::main_task_algorithm(void)
{


	generator::mp_vis_sac_lx_generator eyegen(*this, 4);
	eyegen.robot_m[ROBOT_IRP6_ON_TRACK] = robot_m[ROBOT_IRP6_ON_TRACK];
	eyegen.sensor_m[SENSOR_CAMERA_SA] = sensor_m[SENSOR_CAMERA_SA];


			sr_ecp_msg->message("New loop");

			//mp_seven_eye_generator eyegen(*this, 4);
			//eyegen.robot_m = robot_m;
			//eyegen.sensor_m = sensor_m;
			//po cholere biasujemy jeszcze raz te czujniki i co to w ogole oznacza???
			/*
			 for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
			 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			 {
			 printf("SSSSSSS\n");
			 sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
			 sensor_m_iterator->second->configure_sensor();
			 printf("TTTTTTTT\n");
			 }
			 */
			//dojazd by lapka byla widziana
			/*	
			 set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "../trj/rcsc/irp6ot_ap_1.trj", 1, ROBOT_IRP6_ON_TRACK);
			 
			 
			 run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
			 (1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK);
			 
			 
			 set_next_ecps_state ((int) ECP_GEN_TEACH_IN, 0, "../trj/irp6ot_vis_2.trj", 1, ROBOT_IRP6_ON_TRACK);
			 run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
			 (1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK);
			 */
			printf("AAAAAAAAA\n");

			// wlaczenie generatora transparentnego w obu robotach
			set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 1, "", 1, ROBOT_IRP6_ON_TRACK);

			// opcjonalne serwo wizyjne
			//if (mode)
			//{

			printf("BBBBBBBBBBBBBB\n");

			eyegen.Move();
			//}

			send_end_motion_to_ecps(1, ROBOT_IRP6_ON_TRACK);

			// wlaczenie generatora zacisku na kostce w robocie irp6ot
			set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TFF_RUBIK_GRAB, (int) ecp_mp::task::RCSC_RG_FROM_OPEARTOR_PHASE_1, "", 1, ROBOT_IRP6_ON_TRACK);
			// uruchomienie generatora empty_gen
			run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK);
			// wlaczenie generatora zacisku na kostce w robocie irp6ot
			set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TFF_RUBIK_GRAB, (int) ecp_mp::task::RCSC_RG_FROM_OPEARTOR_PHASE_2, "", 1, ROBOT_IRP6_ON_TRACK);
			// uruchomienie generatora empty_gen
			run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK);

		
}

} // namespace task
} // namespace mp
} // namespace mrrocpp

