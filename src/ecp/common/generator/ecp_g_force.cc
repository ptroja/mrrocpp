/**
 * @file generator/ecp_g_force.cc
 * @brief ECP force generators
 * - class declaration
 * @author yoyek
 * @date 01.01.2002
 *
 * $URL$
 * $LastChangedRevision$
 * $LastChangedDate$
 * $LastChangedBy$
 */




// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - force methods
// Funkcje do tworzenia procesow ECP z wykorzystaniem sily
//
// Ostatnia modyfikacja: 2004r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <math.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/generator/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			weight_meassure_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


weight_meassure::weight_meassure(common::task::task& _ecp_task,
		double _weight_difference, double _catch_time) :
	generator(_ecp_task), weight_difference(_weight_difference),
			current_buffer_pointer(0), initial_weight(0.0),
			initial_weight_counted(false),
			catch_time(_catch_time), terminate_state_recognized(false)
{
	clear_buffer();
}

void weight_meassure::insert_in_buffer(double fx)
{

	weight_in_cyclic_buffer[current_buffer_pointer] = fx;

	if ((++current_buffer_pointer)==WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE)
	{
		current_buffer_pointer=0;
	}

}

void weight_meassure::clear_buffer()
{
	for (int i=0; i<WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE; i++)
	{
		weight_in_cyclic_buffer[current_buffer_pointer] = 0.0;
	}
	current_buffer_pointer=0;
	initial_weight_counted = false;
	terminate_state_recognized = false;

	catch_lag = initial_catch_lag = (int) (1000000*catch_time/(USLEEP_TIME));
	// std::cout << "weight_meassure_generator" << initial_catch_lag << std::endl;

}

double weight_meassure::check_average_weight_in_buffer(void) const
{
	double returned_value=0.0;

	for (int i=0; i<WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE; i++)
	{
		returned_value += weight_in_cyclic_buffer[current_buffer_pointer];
	}
	returned_value/=10;
	return returned_value;
}

void weight_meassure::set_weight_difference(double _weight_difference)
{
	weight_difference = _weight_difference;
}

bool weight_meassure::first_step()
{
	clear_buffer();

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type
			= lib::TCIM;

	return true;
}

bool weight_meassure::next_step()
{
	usleep(USLEEP_TIME);

	if (check_and_null_trigger())
	{
		return false;
	}

	// transformacja ciezaru do osi z ukladu bazowego
	lib::Homog_matrix current_frame_wo_offset(the_robot->reply_package.arm.pf_def.arm_frame);
	current_frame_wo_offset.remove_translation();

	//	std::cout << 	current_frame_wo_offset << std::endl;

	lib::Ft_v_vector force_torque(lib::Ft_tr(current_frame_wo_offset)
			* lib::Ft_v_vector(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz));

	insert_in_buffer(-force_torque[2]);

	//std::cout << 	-force_torque[2] << std::endl;

	// nie wyznaczono jeszcze wagi poczatkowej
	if (!initial_weight_counted)
	{
		if (current_buffer_pointer==0)
		{
			initial_weight_counted = true;
			initial_weight = check_average_weight_in_buffer();
		}

		return true;
	}
	else
	//  wyznaczono wage poczatkowa
	{

		if (((weight_difference>0)&&(check_average_weight_in_buffer()
				- initial_weight) > weight_difference)|| ((weight_difference<0)
				&&(check_average_weight_in_buffer() - initial_weight)
						< weight_difference))

		{
			// wszytkie potweridzenia warunku koncowego musza wystapic pod rzad
			if (!terminate_state_recognized)
			{
				catch_lag = initial_catch_lag;
			}

			terminate_state_recognized = true;
			//    	printf("check_average_weight_in_buffer: %f, %f\n", check_average_weight_in_buffer(), initial_weight );
			if ((--catch_lag) <= 0)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		else
		{
			terminate_state_recognized = false;
			return true;
		}
	}

	return true;
}


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			bias_edp_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


bias_edp_force::bias_edp_force(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
}

bool bias_edp_force::first_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.set_type = RMODEL_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::FORCE_BIAS;

	return true;
}

// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool bias_edp_force::next_step()
{
	return false;
}


tff_nose_run::tff_nose_run(common::task::task& _ecp_task, int step) :
			generator(_ecp_task), step_no(step)
{
	// domyslnie wszytkie osie podatne a pulse_check nieaktywne
	configure_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT);
	configure_pulse_check (false);
	configure_velocity (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_force (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_reciprocal_damping (FORCE_RECIPROCAL_DAMPING, FORCE_RECIPROCAL_DAMPING, FORCE_RECIPROCAL_DAMPING,
		 TORQUE_RECIPROCAL_DAMPING, TORQUE_RECIPROCAL_DAMPING, TORQUE_RECIPROCAL_DAMPING);
	configure_inertia (FORCE_INERTIA, FORCE_INERTIA, FORCE_INERTIA, TORQUE_INERTIA, TORQUE_INERTIA, TORQUE_INERTIA);

	set_force_meassure (false);
}



void tff_nose_run::set_force_meassure(bool fm)
{
	force_meassure = fm;
}


void tff_nose_run::configure_pulse_check(bool pulse_check_activated_l)
{
	pulse_check_activated = pulse_check_activated_l;
}


void tff_nose_run::configure_behaviour(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z,
	 lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az)
 {
	generator_edp_data.next_behaviour[0] = x;
	generator_edp_data.next_behaviour[1] = y;
	generator_edp_data.next_behaviour[2] = z;
	generator_edp_data.next_behaviour[3] = ax;
	generator_edp_data.next_behaviour[4] = ay;
	generator_edp_data.next_behaviour[5] = az;
 }


void tff_nose_run::configure_velocity(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_velocity[0] = x;
	generator_edp_data.next_velocity[1] = y;
	generator_edp_data.next_velocity[2] = z;
	generator_edp_data.next_velocity[3] = ax;
	generator_edp_data.next_velocity[4] = ay;
	generator_edp_data.next_velocity[5] = az;
}


void tff_nose_run::configure_force(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_force_xyz_torque_xyz[0] = x;
	generator_edp_data.next_force_xyz_torque_xyz[1] = y;
	generator_edp_data.next_force_xyz_torque_xyz[2] = z;
	generator_edp_data.next_force_xyz_torque_xyz[3] = ax;
	generator_edp_data.next_force_xyz_torque_xyz[4] = ay;
	generator_edp_data.next_force_xyz_torque_xyz[5] = az;
}


void tff_nose_run::configure_reciprocal_damping(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_reciprocal_damping[0] = x;
	generator_edp_data.next_reciprocal_damping[1] = y;
	generator_edp_data.next_reciprocal_damping[2] = z;
	generator_edp_data.next_reciprocal_damping[3] = ax;
	generator_edp_data.next_reciprocal_damping[4] = ay;
	generator_edp_data.next_reciprocal_damping[5] = az;
}


void tff_nose_run::configure_inertia(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_inertia[0] = x;
	generator_edp_data.next_inertia[1] = y;
	generator_edp_data.next_inertia[2] = z;
	generator_edp_data.next_inertia[3] = ax;
	generator_edp_data.next_inertia[4] = ay;
	generator_edp_data.next_inertia[5] = az;
}




// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_nose_run::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		 the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = generator_edp_data.next_behaviour[i];
		 the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = generator_edp_data.next_velocity[i];
		 the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = generator_edp_data.next_force_xyz_torque_xyz[i];
		 the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[i] = generator_edp_data.next_reciprocal_damping[i];
		 the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = generator_edp_data.next_inertia[i];
	}


	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_nose_run::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	// cout << "next_step" << endl;

	if (pulse_check_activated && check_and_null_trigger())
	{ // Koniec odcinka
		//	ecp_t.set_ecp_reply (lib::TASK_TERMINATED);

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter==1)
	{
		the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate=0;
	}

	// wyrzucanie odczytu sil

	if(force_meassure)
	{
		lib::Homog_matrix current_frame_wo_offset(the_robot->reply_package.arm.pf_def.arm_frame);
		current_frame_wo_offset.remove_translation();

		lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

		std::cout<<"force: "<<force_torque<<std::endl;
	}
	return true;

}


// metoda przeciazona bo nie chcemy rzucac wyjatku wyjscia poza zakres ruchu - UWAGA napisany szkielet skorygowac cialo funkcji


void tff_nose_run::execute_motion(void)
{
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP

	// komunikacja wlasciwa
	the_robot->send();
	if (the_robot->reply_package.reply_type == lib::ERROR) {

		the_robot->query();
		throw ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);

	}
	the_robot->query();

	if (the_robot->reply_package.reply_type == lib::ERROR) {
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
				throw ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
			break;

		} /* end: switch */
	}
}

eih_nose_run::eih_nose_run(common::task::task& _ecp_task,
		int step) : tff_nose_run(_ecp_task, step)
{
	count = 0;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool eih_nose_run::next_step()
{
	++count;

	if (count > 25)
	{// co jakis czas generator sie zatrzymuje
		count = 0;
		return false;
	}else if (pulse_check_activated && check_and_null_trigger())
	{ // Koniec odcinka
		//	ecp_t.set_ecp_reply (lib::TASK_TERMINATED);

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter==1)
	{
		the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate=0;
	}

	// wyrzucanie odczytu sil

	if(force_meassure)
	{
		lib::Homog_matrix current_frame_wo_offset(the_robot->reply_package.arm.pf_def.arm_frame);
		current_frame_wo_offset.remove_translation();

		lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

		std::cout<<"force: "<<force_torque<<std::endl;
	}
	return true;

}

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// pcbird_nose_run generator
//
//////////////////////////////////////////////////////////////////////////////////////////////////

pcbird_nose_run::pcbird_nose_run(common::task::task& _ecp_task,
		int step) : tff_nose_run(_ecp_task, step)
{
	count = 0;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool pcbird_nose_run::first_step()
{
	//bedziemy pobierali dane o robocie
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;

	return tff_nose_run::first_step();
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step ---------------------------------------
// ----------------------------------------------------------------------------------------------

bool pcbird_nose_run::next_step()
{
	++count;

	//co jakis czas generator sie zatrzymuje
	if (count > 10){
		count = 0;
		return false;
	}

	return tff_nose_run::next_step();
}

//////////////////////////////////////////////////////////////////////////////////////////////////

sr_nose_run::sr_nose_run(common::task::task& _ecp_task,
		int step) :
	generator(_ecp_task),
	step_no(step)
{

	// domyslnie wszytkie osie podatne a pulse_check nieaktywne
	configure_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT);
	configure_pulse_check (false);
	configure_velocity (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_force (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_reciprocal_damping (FORCE_RECIPROCAL_DAMPING, FORCE_RECIPROCAL_DAMPING, FORCE_RECIPROCAL_DAMPING,
		 TORQUE_RECIPROCAL_DAMPING, TORQUE_RECIPROCAL_DAMPING, TORQUE_RECIPROCAL_DAMPING);
	configure_inertia (FORCE_INERTIA, FORCE_INERTIA, FORCE_INERTIA, TORQUE_INERTIA, TORQUE_INERTIA, TORQUE_INERTIA);

	set_force_meassure (false);
	state = 0;
}



void sr_nose_run::set_force_meassure(bool fm)
{
	force_meassure = fm;
}


void sr_nose_run::configure_pulse_check(bool pulse_check_activated_l)
{
	pulse_check_activated = pulse_check_activated_l;
}


void sr_nose_run::configure_behaviour(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z,
	 lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az)
 {
	generator_edp_data.next_behaviour[0] = x;
	generator_edp_data.next_behaviour[1] = y;
	generator_edp_data.next_behaviour[2] = z;
	generator_edp_data.next_behaviour[3] = ax;
	generator_edp_data.next_behaviour[4] = ay;
	generator_edp_data.next_behaviour[5] = az;
 }


void sr_nose_run::configure_velocity(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_velocity[0] = x;
	generator_edp_data.next_velocity[1] = y;
	generator_edp_data.next_velocity[2] = z;
	generator_edp_data.next_velocity[3] = ax;
	generator_edp_data.next_velocity[4] = ay;
	generator_edp_data.next_velocity[5] = az;
}


void sr_nose_run::configure_force(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_force_xyz_torque_xyz[0] = x;
	generator_edp_data.next_force_xyz_torque_xyz[1] = y;
	generator_edp_data.next_force_xyz_torque_xyz[2] = z;
	generator_edp_data.next_force_xyz_torque_xyz[3] = ax;
	generator_edp_data.next_force_xyz_torque_xyz[4] = ay;
	generator_edp_data.next_force_xyz_torque_xyz[5] = az;
}


void sr_nose_run::configure_reciprocal_damping(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_reciprocal_damping[0] = x;
	generator_edp_data.next_reciprocal_damping[1] = y;
	generator_edp_data.next_reciprocal_damping[2] = z;
	generator_edp_data.next_reciprocal_damping[3] = ax;
	generator_edp_data.next_reciprocal_damping[4] = ay;
	generator_edp_data.next_reciprocal_damping[5] = az;
}


void sr_nose_run::configure_inertia(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_inertia[0] = x;
	generator_edp_data.next_inertia[1] = y;
	generator_edp_data.next_inertia[2] = z;
	generator_edp_data.next_inertia[3] = ax;
	generator_edp_data.next_inertia[4] = ay;
	generator_edp_data.next_inertia[5] = az;
}




// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool sr_nose_run::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		 the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = generator_edp_data.next_behaviour[i];
		 the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = generator_edp_data.next_velocity[i];
		 the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = generator_edp_data.next_force_xyz_torque_xyz[i];
		 the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[i] = generator_edp_data.next_reciprocal_damping[i];
		 the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = generator_edp_data.next_inertia[i];
	}


	iter_while_idle = 0;

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool sr_nose_run::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	// cout << "next_step" << endl;

	if (pulse_check_activated && check_and_null_trigger())
	{ // Koniec odcinka
		//	ecp_t.set_ecp_reply (lib::TASK_TERMINATED);
		next_state();
//std::cerr << iter_while_idle << std::endl;

		return false;
	}

	if(state == 0)
		return true;

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	//Kwantowanie sily
	for (int i=0; i<12; i++)
	{
		int temp = (int)the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[i];
		the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[i] = (double)temp;
	}


	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter==1)
	{
		the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate=0;
	}

	// wyrzucanie odczytu sil

	if(force_meassure)
	{
		lib::Homog_matrix current_frame_wo_offset(the_robot->reply_package.arm.pf_def.arm_frame);
		current_frame_wo_offset.remove_translation();

		lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

		std::cout<<"force: "<<force_torque<<std::endl;
	}


	return check_and_decide();




//	for (int i=0; i<6; i++)
//		std::cerr << the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[i] << "   ";
//	std::cerr << std::endl;





//	return true;

}

bool sr_nose_run::check_and_decide()
{
	bool cond = true;

	for (int i=0; i<6; i++)
	{
		cond = cond && (fabs(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[i])<2);
//		if (fabs(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[i])>=0.8)
//		fprintf(stderr, "%d -> %.2f\n", i, the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[i]);
	}
	//std::cerr << std::endl;

	if(cond)
		iter_while_idle++;
	else
		iter_while_idle = 0;
//std::cerr << iter_while_idle << std::endl;
	if(iter_while_idle>150)
	{
		iter_while_idle = 0;
		return false;
	}

	return true;
}

short sr_nose_run::get_state()
{
	return state;
}

void sr_nose_run::next_state()
{
	state++;

	if(state>2)
		state = 0;
}

// metoda przeciazona bo nie chcemy rzucac wyjatku wyjscia poza zakres ruchu - UWAGA napisany szkielet skorygowac cialo funkcji
// TODO: this should be rather:
// try {
//   ecp_robot::execute_motion();
// } catch (ECP_error & e) ...

void sr_nose_run::execute_motion(void)
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
	if (the_robot->reply_package.reply_type == lib::ERROR) {

		the_robot->query();
		throw ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);

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
	if (the_robot->reply_package.reply_type == lib::ERROR) {


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
				throw ecp_robot::ECP_error (lib::NON_FATAL_ERROR, EDP_ERROR);
			break;

		} /* end: switch */


	}
}
// ----------- END OF SPOTS_RECOGNITION -------------------------




tff_rubik_grab::tff_rubik_grab(common::task::task& _ecp_task, int step) :
	generator(_ecp_task),
	step_no(step)
{
}

void tff_rubik_grab::configure(double l_goal_position,
		double l_position_increment, unsigned int l_min_node_counter,
		bool l_both_axes_running)
{
	goal_position = l_goal_position;
	position_increment = l_position_increment;
	min_node_counter = l_min_node_counter;
	both_axes_running = l_both_axes_running;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_grab::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.interpolation_type
			= lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
	}

	for (int i=0; i<3; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	if (both_axes_running)
		for (int i=0; i<2; i++)
		{
			the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[i]
					= FORCE_RECIPROCAL_DAMPING;
			the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		}
	else
	{
		the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[1]
				= FORCE_RECIPROCAL_DAMPING;
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[1] = lib::CONTACT;
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::UNGUARDED_MOTION;
	}

	for (int i=2; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_grab::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	//	  cout << "next_step" << endl;


	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter==1)
	{
		the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate=0;
		desired_absolute_gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;
	}

	if ((desired_absolute_gripper_coordinate > goal_position)
			|| (node_counter < min_node_counter))
	{
		desired_absolute_gripper_coordinate -= position_increment;
		the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate =- position_increment;
	}
	else
	{
		return false;
	}

	return true;
}

tff_rubik_face_rotate::tff_rubik_face_rotate(common::task::task& _ecp_task, int step)
	: generator(_ecp_task),
	step_no(step)
{
}

void tff_rubik_face_rotate::configure(double l_turn_angle)
{
	turn_angle = l_turn_angle;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_face_rotate::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.interpolation_type	= lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
	}

	for (int i=0; i<3; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[5] = TORQUE_RECIPROCAL_DAMPING/4;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[5] = lib::CONTACT;

	if (-0.1 < turn_angle && turn_angle < 0.1)
	{
		for (int i=0; i<6; i++)
			the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}
	else
	{
		for (int i=0; i<3; i++)
		{
			the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[i]
					= FORCE_RECIPROCAL_DAMPING;
			the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		}
		for (int i=3; i<5; i++)
			the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;

		the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[5] = TORQUE_RECIPROCAL_DAMPING;
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[5] = lib::CONTACT;
		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[5] = copysign(5.0, turn_angle);
	}

	return true;
}


// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_face_rotate::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	// cout << "next_step" << endl;

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter==1)
	{
		the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate=0;
		if (turn_angle < -0.1 || 0.1 < turn_angle)
		{
			lib::Homog_matrix frame(the_robot->reply_package.arm.pf_def.arm_frame);
			double xyz_eul_zyz[6];
			frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double angle_to_move = (turn_angle / 180.0) * M_PI;
			if (xyz_eul_zyz[5] + angle_to_move < -M_PI)
			{
				stored_gamma = 2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				range_change = true;
			}
			else if (xyz_eul_zyz[5] + angle_to_move > M_PI)
			{
				stored_gamma = -2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				range_change = true;
			}
			else
			{
				stored_gamma = xyz_eul_zyz[5] + angle_to_move;
				range_change = false;
			}
		}
	}
	else
	{

		if (turn_angle < -0.1 || 0.1 < turn_angle)
		{
			lib::Homog_matrix current_frame(the_robot->reply_package.arm.pf_def.arm_frame);
			double xyz_eul_zyz[6];
			current_frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double current_gamma = xyz_eul_zyz[5];
			if (!range_change)
			{
				if(( turn_angle < 0.0 && stored_gamma> current_gamma)
						|| (
								turn_angle> 0.0 && stored_gamma < current_gamma))
						{
							return false;
						}
					}
			else
			{
				if ((turn_angle < 0.0 && stored_gamma < current_gamma)
						|| (turn_angle > 0.0 && stored_gamma > current_gamma))
				{
					range_change = false;
				}
			}
		}

	}

	return true;
}

tff_gripper_approach::tff_gripper_approach(common::task::task& _ecp_task, int step)
	: generator(_ecp_task),
	step_no(step)
{
}

void tff_gripper_approach::configure(double l_speed,
		unsigned int l_motion_time)
{
	speed = l_speed;
	motion_time = l_motion_time;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_gripper_approach::first_step()
{
	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.interpolation_type
			= lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
	}

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = 0;
	}

	the_robot->ecp_command.instruction.arm.pf_def.inertia[2] = FORCE_INERTIA / 4;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = speed;

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		//		the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0;
	}

	the_robot->ecp_command.instruction.arm.pf_def.behaviour[2] = lib::GUARDED_MOTION;
	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[2]
			= FORCE_RECIPROCAL_DAMPING / 2;

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_gripper_approach::next_step()
{
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter==1)
	{
		the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate=0;
	}
	else if (node_counter > motion_time)
	{
		return false;
	}

	return true;
}

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			ecp_force_tool_change_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

force_tool_change::force_tool_change (common::task::task& _ecp_task)
	: generator (_ecp_task)
{
    set_tool_parameters(-0.18, 0.0, 0.25, 0);
}

bool force_tool_change::first_step ()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.set_type = RMODEL_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::FORCE_TOOL;

	for(int i = 0 ; i < 3 ; i++)
		the_robot->ecp_command.instruction.rmodel.force_tool.position[i] = tool_parameters[i];
	the_robot->ecp_command.instruction.rmodel.force_tool.weight = weight;

	return true;
}

bool force_tool_change::next_step ()
{
    return false;
}

void force_tool_change::set_tool_parameters(double x, double y, double z, double v)
{
    tool_parameters[0] = x;
    tool_parameters[1] = y;
    tool_parameters[2] = z;
    weight = v;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
