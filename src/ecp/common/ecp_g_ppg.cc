// -------------------------------------------------------------------------
//                            ecp_gen_playerpos.cc
//            Effector Control Process (ECP) - rysowanie
// 			Funkcje do tworzenia procesow ECP z rysowaniem
// 			Ostatnia modyfikacja: 01.06.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_ppg.h"
#include "ecp_mp/ecp_mp_tr_player.h"

playerpos_generator::playerpos_generator(ecp_task& _ecp_task, int step):
	ecp_generator (_ecp_task, true)
{	
	step_no = step;
};  

bool playerpos_generator::first_step ( ) 
{
	run_counter = 0;
	second_step = false;
	ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);

	ecp_t.get_mp_command ();
	node_counter = 0;
	//td.interpolation_node_no = 1;
	//td.internode_step_no = step_no;

	switch ( ecp_t.mp_command_type() ) 
	{
		case NEXT_POSE:
			/*
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
			the_robot->EDP_data.set_type = ARM_DV;

			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.get_arm_type = JOINT;

			the_robot->EDP_data.motion_type = ABSOLUTE;
			//the_robot->EDP_data.motion_steps = td.internode_step_no;
			//the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

			the_robot->create_command ();
			*/
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			printf("first step in mp comm: %d\n", ecp_t.mp_command_type());
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	} // end: switch

	return true;
}; // end: bool playerpos_generator::first_step (map <SENSOR_ENUM, sensor*>& sensor_m, robot& the_robot )
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool playerpos_generator::next_step ( ) 
{
	if (ecp_t.pulse_check()) 
	{
		ecp_t.get_mp_command ();
		return false;
	}
	else
	{
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.get_mp_command ();
	}

    transmitter_m[TRANSMITTER_PLAYER]->t_read(1);
    printf("odometry: [%f, %f %f]\n",
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_position.px,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_position.py,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_position.pa
          );
	
	switch ( ecp_t.mp_command_type() ) 
	{
		case NEXT_POSE:
			//the_robot->create_command ();
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			printf("next step in mp comm: %d\n", ecp_t.mp_command_type());
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	} // end: switch

	return true;
}; // end: bool playerpos_generator::next_step (, robot& the_robot )
