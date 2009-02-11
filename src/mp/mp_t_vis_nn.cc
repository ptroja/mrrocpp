// ------------------------------------------------------------------------
//             mp_m_pr.cc - powielanie rysunku - wersja wielorobotowa
// 
//                      MASTER PROCESS (MP) - main()
// 
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_vis_nn.h"
#include "mp/mp_g_vis_nn.h"
#include "mp/mp_g_force.h"
#include "ecp_mp/ecp_mp_s_vis_nn.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

mp_task* return_created_mp_task(configurator &_config)
{
	return new mp_task_vis_nn(_config);
}

mp_task_vis_nn::mp_task_vis_nn(configurator &_config) :
	mp_task(_config)
	{
	}

// methods fo mp template to redefine in concete class
void mp_task_vis_nn::task_initialization(void)
{
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] = new ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

	sensor_m[SENSOR_CAMERA_SA] = new ecp_mp_vis_nn_sensor (SENSOR_CAMERA_SA, "[vsp_nn]", *this); //change if SENSOR_CAMERA_SA used for nonnn recog (vsp_vis_pbeolsac)

	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
	!= sensor_m.end(); sensor_m_iterator++) {
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);
	sr_ecp_msg->message("MP vis nn loaded");
}
;

void mp_task_vis_nn::main_task_algorithm(void)
{

	mp_nn_eye_generator eyegen(*this, 4);
	eyegen.robot_m[ROBOT_IRP6_ON_TRACK] = robot_m[ROBOT_IRP6_ON_TRACK];
	eyegen.sensor_m[SENSOR_CAMERA_SA] = sensor_m[SENSOR_CAMERA_SA];


	sr_ecp_msg->message("New loop");

	//mp_seven_eye_generator eyegen(*this, 4);
	//eyegen.robot_m = robot_m;
	//eyegen.sensor_m = sensor_m;
	//po cholere biasujemy jeszcze raz te czujniki i co to w ogole oznacza???
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin(); sensor_m_iterator
	!= sensor_m.end(); sensor_m_iterator++) {
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	set_next_ecps_state((int) ECP_GEN_TEACH_IN, 0, "../trj/rcsc/irp6ot_ap_1.trj", 1, ROBOT_IRP6_ON_TRACK);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state((int) ECP_GEN_TEACH_IN, 0, "../trj/rcsc/irp6ot_ap_2.trj", 1, ROBOT_IRP6_ON_TRACK);
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK);

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state((int) ECP_GEN_TRANSPARENT, (int) 1, "", 1, ROBOT_IRP6_ON_TRACK);

	// opcjonalne serwo wizyjne
	//if (mode)
	//{

	eyegen.Move();
	//}

	send_end_motion_to_ecps(1, ROBOT_IRP6_ON_TRACK);

	// wlaczenie generatora zacisku na kostce w robocie irp6ot
	set_next_ecps_state((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FROM_OPEARTOR_PHASE_1, "", 1, ROBOT_IRP6_ON_TRACK);
	// uruchomienie generatora empty_gen
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK);
	// wlaczenie generatora zacisku na kostce w robocie irp6ot
	set_next_ecps_state((int) ECP_GEN_TFF_RUBIK_GRAB, (int) RCSC_RG_FROM_OPEARTOR_PHASE_2, "", 1, ROBOT_IRP6_ON_TRACK);
	// uruchomienie generatora empty_gen
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_ON_TRACK);

}
