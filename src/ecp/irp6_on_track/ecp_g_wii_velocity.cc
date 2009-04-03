#include "ecp/irp6_on_track/ecp_g_wii_velocity.h"

#include "common/impconst.h"
#include "common/com_buf.h"
#include "math.h"

ecp_wii_velocity_generator::ecp_wii_velocity_generator (ecp_task& _ecp_task) : ecp_tff_nose_run_generator(_ecp_task,0)
{
	configure_behaviour(UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION);
}

bool ecp_wii_velocity_generator::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);

	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->EDP_data.set_type = ARM_DV;
	the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
	the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
	the_robot->EDP_data.set_arm_type = PF_VELOCITY;
	the_robot->EDP_data.get_arm_type = FRAME;
	the_robot->EDP_data.motion_type = RELATIVE;
	the_robot->EDP_data.next_interpolation_type = TCIM;
	the_robot->EDP_data.motion_steps = td.internode_step_no;
	the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		 the_robot->EDP_data.next_behaviour[i] = generator_edp_data.next_behaviour[i];
		 the_robot->EDP_data.next_velocity[i] = generator_edp_data.next_velocity[i];
		 the_robot->EDP_data.next_force_xyz_torque_xyz[i] = generator_edp_data.next_force_xyz_torque_xyz[i];
		 the_robot->EDP_data.next_reciprocal_damping[i] = generator_edp_data.next_reciprocal_damping[i];
		 the_robot->EDP_data.next_inertia[i] = generator_edp_data.next_inertia[i];
	}


	return true;
}
; // end: ecp_wii_velocity_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_wii_velocity_generator::next_step()
{
	char buffer[200];
	try
	{
		sensor_m[SENSOR_WIIMOTE]->get_reading();
	}
	catch(...)
	{
	}

	++step_no;


	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	// cout << "next_step" << endl;

	if (pulse_check_activated && check_and_null_trigger())
	{ // Koniec odcinka
		//	ecp_t.set_ecp_reply (TASK_TERMINATED);

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->EDP_data.instruction_type = SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter==1)
	{
		the_robot->EDP_data.next_gripper_coordinate
				= the_robot->EDP_data.current_gripper_coordinate;
	}

	// wyrzucanie odczytu sil

	if(force_meassure)
	{
		Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
		current_frame_wo_offset.remove_translation();

		Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);

		std::cout<<"force: "<<force_torque<<std::endl;
	}
	return true;

}
; // end: bool ecp_wii_velocity_generator::next_step ()


// metoda przeciazona bo nie chcemy rzucac wyjatku wyjscia poza zakres ruchu - UWAGA napisany szkielet skorygowac cialo funkcji


void ecp_wii_velocity_generator::execute_motion(void)
{
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
	/*
	 // maskowanie sygnalu SIGTERM
	 // w celu zapobierzenia przerwania komunikacji ECP z EDP pomiedzy SET a QUERY - usuniete

	 sigset_t set;

	 sigemptyset( &set );
	 sigaddset( &set, SIGTERM );

	 if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1)
	 {
	 printf ("blad w ECP procmask signal\n");
	 }
	 */
	// komunikacja wlasciwa
	the_robot->send();
	if (the_robot->reply_package.reply_type == ERROR) {

		the_robot->query();
		throw ecp_robot::ECP_error (NON_FATAL_ERROR, EDP_ERROR);

	}
	the_robot->query();

	/*
	 // odmaskowanie sygnalu SIGTERM

	 sigemptyset( &set );

	 if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1)
	 {
	 printf ("blad w ECP procmask signal\n");
	 }
	 */
	if (the_robot->reply_package.reply_type == ERROR) {


		switch ( the_robot->reply_package.error_no.error0 ) {
			case BEYOND_UPPER_D0_LIMIT:
			case BEYOND_UPPER_THETA1_LIMIT:
			case BEYOND_UPPER_THETA2_LIMIT:
			case BEYOND_UPPER_THETA3_LIMIT:
			case BEYOND_UPPER_THETA4_LIMIT:
			case BEYOND_UPPER_THETA5_LIMIT:
			case BEYOND_UPPER_THETA6_LIMIT:
			case BEYOND_UPPER_THETA7_LIMIT:
			case BEYOND_LOWER_D0_LIMIT:
			case BEYOND_LOWER_THETA1_LIMIT:
			case BEYOND_LOWER_THETA2_LIMIT:
			case BEYOND_LOWER_THETA3_LIMIT:
			case BEYOND_LOWER_THETA4_LIMIT:
			case BEYOND_LOWER_THETA5_LIMIT:
			case BEYOND_LOWER_THETA6_LIMIT:
			case BEYOND_LOWER_THETA7_LIMIT:
			break;
			default:
				throw ecp_robot::ECP_error (NON_FATAL_ERROR, EDP_ERROR);
			break;

		} /* end: switch */


	}
}

