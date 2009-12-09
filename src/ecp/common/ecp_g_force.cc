/**
 * @file ecp_g_force.cc
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
#include "ecp/common/ecp_g_force.h"

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

	lib::Ft_v_vector force_torque(lib::Ft_v_tr(current_frame_wo_offset, lib::Ft_v_tr::FT)
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
// 			y_nose_run_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_nose_run_force::y_nose_run_force(common::task::task& _ecp_task, int step) :
	generator(_ecp_task),
	step_no(step)
{
}

bool y_nose_run_force::first_step()
{
	for (int i=0; i<6; i++)
	{
		delta[i]=0.0;
	}

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV;

	//		the_robot->EDP_data.force_move_mode=2; // z regulacja silowa po query
	//		the_robot->EDP_data.position_set_mode=1; // przyrostowo

	//		the_robot->EDP_data.force_axis_quantity=3; // DOBRZE

	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.interpolation_type
			= lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
	/*
	 the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0.0;
	 the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0.0;
	 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=0.0;

	 for (int j=0; j<6 ; j++)	{
	 the_robot->EDP_data.position_increment[j]=0.0;
	 }

	 the_robot->EDP_data.dyslocation_matrix[0][0]=1;
	 the_robot->EDP_data.dyslocation_matrix[1][1]=1;
	 the_robot->EDP_data.dyslocation_matrix[2][2]=1;
	 the_robot->EDP_data.dyslocation_matrix[3][3]=0;
	 the_robot->EDP_data.dyslocation_matrix[4][4]=0;
	 the_robot->EDP_data.dyslocation_matrix[5][5]=0;
	 */

	return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_nose_run_force::next_step()
{
	struct timespec start[9];

	if (check_and_null_trigger())
	{
		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	// (okreslenie kolejnego wezla interpolacji)
	if (((node_counter)%1000)==1)
	{
		if (clock_gettime( CLOCK_REALTIME , &start[0]) == -1)
		{
			printf("blad pomiaru czasu");
		}
		printf("ECP pomiarow: %d,  czas: %ld\n", node_counter, start[0].tv_sec
				%100);
	}

	return true;
}


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_egg_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_egg_force::y_egg_force(common::task::task& _ecp_task, int step,
		int mode) :
			generator(_ecp_task),
			step_no(step),
			int_mode(mode)
{
}

bool y_egg_force::first_step()
{

	for (int i=0; i<6; i++)
		delta[i]=0.0;

	gen_state = next_gen_state = 1; // jazda w powietrzu
	prev_gen_state = 0;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	// 		the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.get_type = ARM_DV+OUTPUTS_DV; // arm_inputs DEBUG
	the_robot->ecp_command.instruction.set_type = ARM_DV;
	/*
	 the_robot->EDP_data.force_move_mode=2; // z regulacja silowa po query
	 the_robot->EDP_data.position_set_mode=1; // przyrostowo

	 the_robot->EDP_data.force_axis_quantity=3; // DOBRZE

	 the_robot->ecp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
	 the_robot->ecp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
	 the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	 the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0.0;
	 the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0.0;
	 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=0.0;

	 for (int j=0; j<6 ; j++)	{
	 the_robot->EDP_data.position_increment[j]=0.0;
	 }

	 the_robot->EDP_data.dyslocation_matrix[0][0]=1;
	 the_robot->EDP_data.dyslocation_matrix[1][1]=1;
	 the_robot->EDP_data.dyslocation_matrix[2][2]=1;
	 the_robot->EDP_data.dyslocation_matrix[3][3]=0;
	 the_robot->EDP_data.dyslocation_matrix[4][4]=0;
	 the_robot->EDP_data.dyslocation_matrix[5][5]=0;
	 */

	return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_egg_force::next_step()
{

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	// (okreslenie kolejnego wezla interpolacji)
	// 	printf("odczyt: %d, sila: %f\n", the_robot->reply_package.analog_input[1], (sensor_m.begin())->second->image.sensor_union.force.rez[2]);

	gen_state=next_gen_state;

	switch (gen_state)
	{
	case 0:
		break;
	case 1: // wodzenie za nos

		if (prev_gen_state != gen_state)
		{
			sr_ecp_msg.message("Wodzenie za nos - aby zakonczyc nacisnij ECP_TRIGGER");
		}

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;
		/*
		 the_robot->EDP_data.force_axis_quantity=3; // DOBRZE
		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0.0;
		 the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0.0;
		 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=0.0;

		 for (int j=0; j<6 ; j++)	{
		 the_robot->EDP_data.position_increment[j]=0.0;
		 }
		 */
		// inkrementacja numeru iteracji dla biezacego stanu
		in_state_iteration++;

		if (check_and_null_trigger())
		{
			in_state_iteration=0;
			next_gen_state = 2;
		}

		break;
	case 2: // oczekiwanie na jajko (brak ruchu)

		if (prev_gen_state != gen_state)
		{
			sr_ecp_msg.message("Wloz jajko i nacisnij ECP_TRIGGER");
		}

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;
		/*
		 the_robot->EDP_data.force_axis_quantity=0; // ruch czysto pozycyjny

		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 // inkrementacja numeru iteracji dla biezacego stanu
		 in_state_iteration++;

		 // bezruch
		 for (int j=0; j<6 ; j++)	{
		 the_robot->EDP_data.position_increment[j]=0.0;
		 }
		 */

		if (check_and_null_trigger())
		{
			in_state_iteration=0;
			next_gen_state = 3;
		}

		break;

	case 3: // uwzglednianie ciezaru jajka - zerowanie odczytow czujnika (bias)

		// zerowanie odczytow
		(sensor_m.begin())->second->to_vsp.parameters=1;
		(sensor_m.begin())->second->configure_sensor();

		next_gen_state = 4;

		break;

	case 4: // jazda w dol (pozycyjna)

		if (prev_gen_state != gen_state)
		{
			sr_ecp_msg.message("Pozycyjny ruch w dol");
		}

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;
		/*
		 the_robot->EDP_data.force_axis_quantity = 0;

		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 the_robot->EDP_data.position_increment[0]=0.0;
		 the_robot->EDP_data.position_increment[1]=0.0;
		 the_robot->EDP_data.position_increment[2]=-POS_Z_AXIS_MACROSTEP_INC;
		 the_robot->EDP_data.position_increment[3]=0.0;
		 the_robot->EDP_data.position_increment[4]=0.0;
		 the_robot->EDP_data.position_increment[5]=0.0;
		 */
		in_state_iteration++;

		if ( (in_state_iteration>INIT_ITER_NUMBER)&&(
		/*
		 ((the_robot->reply_package.analog_input[1]<PROG_ODLEGLOSCI_PODCZERWIEN_EGG)
		 &&(the_robot->reply_package.analog_input[1]>ODLEGLOSCI_PODCZERWIEN_MIN_VALUE))|| // sieje zerami bugami
		 */
		((sensor_m.begin())->second->image.sensor_union.force.rez[2]>SILA_KONTAKTU_EGG) ))
		{
			in_state_iteration=0;
			if (int_mode==0) // rozbijamy
			{
				next_gen_state = 5;
			}
			else // nierozbijamy
			{
				next_gen_state = 6;
			}

		}

		break;

	case 5: // jazda w dol (pozycyjna, po zetknieciu z jajkiem) - stan do rozbijania

		if (prev_gen_state != gen_state)
		{
			sr_ecp_msg.message("Pozycyjny ruch w dol, po zetknieciu z jajkiem");
		}

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;
		/*
		 the_robot->EDP_data.force_axis_quantity = 0;

		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 the_robot->EDP_data.position_increment[0]=0.0;
		 the_robot->EDP_data.position_increment[1]=0.0;
		 the_robot->EDP_data.position_increment[2]=-POS_Z_AXIS_MACROSTEP_INC;
		 the_robot->EDP_data.position_increment[3]=0.0;
		 the_robot->EDP_data.position_increment[4]=0.0;
		 the_robot->EDP_data.position_increment[5]=0.0;
		 */
		in_state_iteration++;

		if (in_state_iteration>IMPACT_ITERATIONS_NUMBER)
		{
			in_state_iteration=0;
			next_gen_state = 6;
		}

		break;

	case 6: // silowy ruch w dol

		if (prev_gen_state != gen_state)
		{
			sr_ecp_msg.message("Silowy ruch w dol - aby zakonczyc nacisnij ECP_TRIGGER");
		}

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;

		/*
		 the_robot->EDP_data.force_axis_quantity=1;

		 the_robot->EDP_data.relative_force_vector[0]=0.0;
		 the_robot->EDP_data.relative_force_vector[1]=0.0;
		 the_robot->EDP_data.relative_force_vector[2]=1.0;

		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
		 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP_OPADANIE_EGG;

		 in_state_iteration++;

		 for (int j=0; j<6 ; j++)	{
		 the_robot->EDP_data.position_increment[j]=0.0;
		 }
		 */
		if (check_and_null_trigger())
		{
			in_state_iteration=0;
			next_gen_state = 7;
		}

		break;
	case 7: // krotki ruch w gore

		if (prev_gen_state != gen_state)
		{
			sr_ecp_msg.message("Unoszenie");
		}

		td.interpolation_node_no = 1;
		td.internode_step_no = 1000;
		td.value_in_step_no = td.internode_step_no - 2;
		/*
		 the_robot->EDP_data.force_axis_quantity=0;

		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 the_robot->EDP_data.position_increment[0]=0.0;
		 the_robot->EDP_data.position_increment[1]=0.0;
		 the_robot->EDP_data.position_increment[2]=0.05;
		 the_robot->EDP_data.position_increment[3]=0.0;
		 the_robot->EDP_data.position_increment[4]=0.0;
		 the_robot->EDP_data.position_increment[5]=0.0;
		 */
		next_gen_state = 1;

		break;
	default:

		break;
	}

	prev_gen_state=gen_state;

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

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_edge_follow_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_edge_follow_force::y_edge_follow_force(
		common::task::task& _ecp_task, int step) :
	ecp_teach_in_generator(_ecp_task),
	step_no(step),
	tool_frame(0.0, 0.0, 0.25)
{
}


bool y_edge_follow_force::first_step()
{
	for (int i=0; i<6; i++)
		delta[i]=0.0;

	create_pose_list_head(emptyps, 0.0, delta, 2);

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		//	the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0.0;
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[0] = FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::CONTACT;
	// Sila dosciku do rawedzi
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[0] = 4;

	return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_edge_follow_force::next_step()
{
	// static int count;
	// struct timespec start[9];
	if (check_and_null_trigger())
	{
		return false;
	}

	// 	wstawienie nowego przyrostu pozyji do przyrostowej trajektorii ruchu do zapisu do pliku
	lib::Homog_matrix tmp_matrix(the_robot->reply_package.arm.pf_def.arm_frame);

	// tablice pomocnicze do utworzenia przyrostowej trajektorii ruchu do zapisu do pliku
	double inc_delta[6], tmp_delta[6];

	tmp_matrix.get_xyz_euler_zyz(inc_delta);

	for (int i=0; i<6; i++)
		inc_delta[i] = -inc_delta[i];

	tmp_matrix.set_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
	tmp_matrix.get_xyz_euler_zyz(tmp_delta);

	for (int i=0; i<6; i++)
		inc_delta[i]+=tmp_delta[i];

	insert_pose_list_element(emptyps, 0.0, inc_delta, 2);

	// wyznaczenie nowej macierzy referencyjnej i predkosci ruchu

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
			= the_robot->reply_package.arm.pf_def.gripper_coordinate;

	for (int i=0; i<MAX_SERVOS_NR; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=0.0;
	}






	// sprowadzenie sil do ukladu kisci
	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

	double wx = force_torque[0];
	double wy = force_torque[1];

	double v = hypot(wx, wy);

	if (v != 0.0)
	{
		double s_alfa = wy / v;
		double c_alfa = wx / v;

		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = 0.002*v;
		//     the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = -0.00;
		//	the_robot->EDP_data.ECPtoEDP_position_velocity[1] = 0.0;

		// basic_rot_frame = lib::Homog_matrix(c_alfa, s_alfa, 0.0,	-s_alfa, c_alfa, 0.0,	0.0, 0.0, 1,	0.0, 0.0, 0.0);
		basic_rot_frame = lib::Homog_matrix(
			c_alfa, -s_alfa, 0.0, 0.0,
			s_alfa, c_alfa, 0.0, 0.0,
			0.0, 0.0, 1, 0.0
		);

		// dodatkowa macierz obracajaca kierunek wywieranej sily tak aby stabilizowac jej wartosc
		double alfa_r = 0.2*(v-4);
		double s_alfa_r = sin(alfa_r);
		double c_alfa_r = cos(alfa_r);

		// ex_rot_frame = lib::Homog_matrix(c_alfa_r, s_alfa_r, 0.0,	-s_alfa_r, c_alfa_r, 0.0,	0.0, 0.0, 1,	0.0, 0.0, 0.0);
		ex_rot_frame = lib::Homog_matrix(c_alfa_r, -s_alfa_r, 0.0, 0.0,
			s_alfa_r, c_alfa_r, 0.0, 0.0,
			 0.0, 0.0, 1, 0.0);

		// obrocenie pierwotnej macierzy
		basic_rot_frame = basic_rot_frame * ex_rot_frame;

//		basic_rot_frame = !basic_rot_frame;

		tool_frame = tool_frame * basic_rot_frame;
		// basic_rot_frame.set_translation_vector(0, 0, 0.25);

		tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

		//	ECPtoEDP_ref_frame.get_frame_tab(the_robot->EDP_data.ECPtoEDP_reference_frame);

		/*
		 the_robot->EDP_data.ECPtoEDP_reference_frame[0][0] = c_alfa;
		 the_robot->EDP_data.ECPtoEDP_reference_frame[0][1] = s_alfa;

		 the_robot->EDP_data.ECPtoEDP_reference_frame[1][0] = -s_alfa;
		 the_robot->EDP_data.ECPtoEDP_reference_frame[1][1] = c_alfa;
		 */

		printf("sensor: x: %+ld, y: %+ld, v:%+ld, %f\n",
				lround(wx), lround(wy), lround(v),
				atan2(s_alfa, c_alfa)*DEGREES_TO_RADIANS);
	}

	return true;

}

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// legobrick_attach_force_generator
//
//////////////////////////////////////////////////////////////////////////////////////////////////
legobrick_attach_force::legobrick_attach_force(
		common::task::task& _ecp_task, int step) :
	ecp_teach_in_generator(_ecp_task),
	step_no(step)//, tool_frame(0.026551, -0.011313, 0.25 + 0.028)
{
	//macierz jednorodna przejscia na uklad narzedzia do przemieszczania klockow
	lib::frame_tab tmp_tool_frame = {
			{cos(-M_PI/4), -1 * sin (-M_PI/4), 0, 0.02655},
			{sin(-M_PI/4), cos(-M_PI/4), 0, -0.011313},
			{0, 0, 1, 0.25 + 0.028}
	};

	tool_frame = lib::Homog_matrix(tmp_tool_frame);
}
//--------------------------------------------------------------------------------------
bool legobrick_attach_force::first_step()
{
	for (int i=0; i<6; i++)
		delta[i]=0.0;

	create_pose_list_head(emptyps, 0.0, delta, 2);

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	//os x
	//the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[0] = FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[0] = 0.0;
	//the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::CONTACT;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::UNGUARDED_MOTION;

	//the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[3] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[3] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[3] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[3] = lib::UNGUARDED_MOTION;

	//os y (obrotu)
	//the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[1] = FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[1] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[1] = lib::UNGUARDED_MOTION;
	//the_robot->ecp_command.instruction.arm.pf_def.behaviour[1] = lib::CONTACT;

	//the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[4] = TORQUE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[4] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[4] = 0.0;
	//the_robot->ecp_command.instruction.arm.pf_def.behaviour[4] = lib::GUARDED_MOTION;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[4] = lib::UNGUARDED_MOTION;

	//os z
	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[2] = FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[2] = 5.0;
	//the_robot->ecp_command.instruction.arm.pf_def.behaviour[2] = lib::UNGUARDED_MOTION;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[2] = lib::CONTACT;

	//the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[5] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[5] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[5] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[5] = lib::UNGUARDED_MOTION;

	return true;
}
//--------------------------------------------------------------------------------------
bool legobrick_attach_force::next_step()
{
	if (check_and_null_trigger())
	{
		return false;
	}

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
			= the_robot->reply_package.arm.pf_def.gripper_coordinate;

	for (int i=0; i<MAX_SERVOS_NR; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=0.0;
	}

	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

	double wz = force_torque[2];

	//warunek stopu
	if(wz >= 5.0) return false;

	return true;

}
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// legobrick_detach_force_generator
//
//////////////////////////////////////////////////////////////////////////////////////////////////
legobrick_detach_force::legobrick_detach_force(
		common::task::task& _ecp_task, int step) :
	ecp_teach_in_generator(_ecp_task),
	step_no(step)//, tool_frame(0.026551, -0.011313, 0.25 + 0.028)
{
	//macierz jednorodna przejscia na uklad narzedzia do przemieszczania klockow
	lib::frame_tab tmp_tool_frame = {{cos(-M_PI/4), -1 * sin (-M_PI/4), 0, 0.02655}
				, {sin(-M_PI/4), cos(-M_PI/4), 0, -0.011313}
				, {0, 0, 1, 0.25 + 0.028}};

	tool_frame = lib::Homog_matrix(tmp_tool_frame);
	isStart = true;
}
//--------------------------------------------------------------------------------------
bool legobrick_detach_force::first_step()
{
	for (int i=0; i<6; i++)
		delta[i]=0.0;

	create_pose_list_head(emptyps, 0.0, delta, 2);

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.rmodel.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA/4;
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	//os x
	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[0] = FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[0] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::CONTACT;
	//the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::UNGUARDED_MOTION;

	//the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[3] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[3] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[3] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[3] = lib::UNGUARDED_MOTION;

	//os y (obrotu)
	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[1] = FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[1] = 0.0;
	//the_robot->ecp_command.instruction.arm.pf_def.behaviour[1] = lib::UNGUARDED_MOTION;//CONTACT;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[1] = lib::CONTACT;

	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[4] = TORQUE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[4] = -0.05;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[4] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[4] = lib::GUARDED_MOTION;

	//os z
	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[2] = FORCE_RECIPROCAL_DAMPING/4;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[2] = 20.0;
	//the_robot->ecp_command.instruction.arm.pf_def.behaviour[2] = lib::UNGUARDED_MOTION;//CONTACT;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[2] = lib::CONTACT;

	//the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[5] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[5] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[5] = 0;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[5] = lib::UNGUARDED_MOTION;

	return true;
}
//--------------------------------------------------------------------------------------
bool legobrick_detach_force::next_step()
{
	if (check_and_null_trigger())
	{
		return false;
	}

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
			= the_robot->reply_package.arm.pf_def.gripper_coordinate;

	for (int i=0; i<MAX_SERVOS_NR; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]=0.0;
	}

	if(isStart){
		start_position_w3 = 3 * the_robot->reply_package.arm.pf_def.arm_coordinates[5];
		isStart = false;
	}

	//warunek stopu
	double stop_position_w3 = 3 * the_robot->reply_package.arm.pf_def.arm_coordinates[5];

	printf("Start: %f , Stop: %f\n", start_position_w3, stop_position_w3);
	printf("Roznica %f\n", start_position_w3 - stop_position_w3);

	if(fabs(start_position_w3 - stop_position_w3) > 0.8) return false;

	//printf("Start: %f , Stop: %f\n", start_position_w3, stop_position_w3);

	return true;

}
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_drawing_teach_in_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_drawing_teach_in_force::y_drawing_teach_in_force(
		common::task::task& _ecp_task, int step) :
	ecp_teach_in_generator(_ecp_task),
	step_no(step)
{
}

bool y_drawing_teach_in_force::first_step()
{

	if (teach_or_move == YG_MOVE)
	{

		for (int i=0; i<6; i++)
			delta[i]=0.0;

		initiate_pose_list();

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;

		the_robot->ecp_command.instruction.instruction_type = lib::GET;
		the_robot->ecp_command.instruction.get_type = ARM_DV;
		the_robot->ecp_command.instruction.set_type = ARM_DV;

		/*
		 the_robot->EDP_data.force_axis_quantity=1;

		 the_robot->EDP_data.relative_force_vector[0]=0.0;
		 the_robot->EDP_data.relative_force_vector[1]=0.0;
		 the_robot->EDP_data.relative_force_vector[2]=1.0;

		 normalize_vector(the_robot->EDP_data.relative_force_vector,the_robot->EDP_data.relative_force_vector,3);

		 the_robot->ecp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
		 the_robot->ecp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
		 the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;

		 the_robot->EDP_data.dyslocation_matrix[0][0]=1;
		 the_robot->EDP_data.dyslocation_matrix[1][1]=1;
		 the_robot->EDP_data.dyslocation_matrix[2][2]=1;
		 the_robot->EDP_data.dyslocation_matrix[3][3]=0;
		 the_robot->EDP_data.dyslocation_matrix[4][4]=0;
		 the_robot->EDP_data.dyslocation_matrix[5][5]=0;
		 */

		return true;

		// UCZENIE

	}
	else if (teach_or_move == YG_TEACH)
	{

		for (int i=0; i<6; i++)
			delta[i]=0.0;

		create_pose_list_head(emptyps, 0.0, delta);

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;

		the_robot->ecp_command.instruction.instruction_type = lib::GET;
		the_robot->ecp_command.instruction.get_type = ARM_DV;
		the_robot->ecp_command.instruction.set_type = ARM_DV;
		/*
		 the_robot->EDP_data.force_move_mode=2;
		 the_robot->EDP_data.position_set_mode=1; // przyrostowo

		 the_robot->EDP_data.force_axis_quantity=3;

		 the_robot->ecp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
		 the_robot->ecp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
		 the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0;
		 the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0;
		 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;

		 for (int j=0; j<6 ; j++)	{
		 the_robot->EDP_data.position_increment[j]=0.0;
		 }
		 // zerowy przyrost pozycji

		 the_robot->EDP_data.dyslocation_matrix[0][0]=1;
		 the_robot->EDP_data.dyslocation_matrix[1][1]=1;
		 the_robot->EDP_data.dyslocation_matrix[2][2]=1;
		 the_robot->EDP_data.dyslocation_matrix[3][3]=0;
		 the_robot->EDP_data.dyslocation_matrix[4][4]=0;
		 the_robot->EDP_data.dyslocation_matrix[5][5]=0;
		 */

		return true;

	}
	return true;

}
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
bool y_drawing_teach_in_force::next_step()
{

	if (teach_or_move == YG_MOVE)
	{

		ecp_taught_in_pose tip; // Nauczona pozycja

		if (!(is_pose_list_element()))
		{ // Koniec odcinka
			return false;
			// flush_pose_list();

		}

		// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
		the_robot->ecp_command.instruction.instruction_type = lib::SET;

		// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
		// (okreslenie kolejnego wezla interpolacji)

		get_pose(tip);

		delta[0]=tip.coordinates[0];
		delta[1]=tip.coordinates[1];
		/*
		 the_robot->EDP_data.position_increment[0]=delta[0];
		 the_robot->EDP_data.position_increment[1]=delta[1];
		 the_robot->EDP_data.position_increment[2]=0.0;
		 the_robot->EDP_data.position_increment[3]=0.0;
		 the_robot->EDP_data.position_increment[4]=0.0;
		 the_robot->EDP_data.position_increment[5]=0.0;
		 */
		next_pose_list_ptr();

		return true;

		// UCZENIE

	}
	else if (teach_or_move == YG_TEACH)
	{

		int i; // licznik kolejnych wspolrzednych wektora [0..6]

		double inc_delta[6]=
		{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

		if (check_and_null_trigger())
		{ // Koniec odcinka
			initiate_pose_list();
			return false;
		}

		for (i=0; i<6; i++)
		{
			inc_delta[i]
					=-the_robot->reply_package.arm.pf_def.arm_coordinates[i];
		}

		for (i=0; i<6; i++)
		{
			inc_delta[i]
					+=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
		}

		// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
		the_robot->ecp_command.instruction.instruction_type = lib::SET;

		insert_pose_list_element(emptyps, 0.0, inc_delta);

		return true;

	}

	return true;

}

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_advanced_drawing_teach_in_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_advanced_drawing_teach_in_force::y_advanced_drawing_teach_in_force(
		common::task::task& _ecp_task, int step) :
	y_drawing_teach_in_force(_ecp_task, step)
{
}

bool y_advanced_drawing_teach_in_force::first_step()
{

	if (teach_or_move == YG_MOVE)
	{

		for (int i=0; i<6; i++)
			delta[i]=0.0;

		initiate_pose_list();

		gen_state = next_gen_state = 4; // jazda w powietrzu
		prev_gen_state = 0;

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;

		the_robot->ecp_command.instruction.instruction_type = lib::GET;
		the_robot->ecp_command.instruction.get_type = ARM_DV;
		the_robot->ecp_command.instruction.set_type = ARM_DV;

		/*
		 the_robot->EDP_data.force_axis_quantity=1;

		 the_robot->EDP_data.relative_force_vector[0]=0.0;
		 the_robot->EDP_data.relative_force_vector[1]=0.0;
		 the_robot->EDP_data.relative_force_vector[2]=1.0;

		 normalize_vector(the_robot->EDP_data.relative_force_vector,the_robot->EDP_data.relative_force_vector,3);

		 the_robot->ecp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
		 the_robot->ecp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
		 the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;

		 the_robot->EDP_data.dyslocation_matrix[0][0]=1;
		 the_robot->EDP_data.dyslocation_matrix[1][1]=1;
		 the_robot->EDP_data.dyslocation_matrix[2][2]=1;
		 the_robot->EDP_data.dyslocation_matrix[3][3]=0;
		 the_robot->EDP_data.dyslocation_matrix[4][4]=0;
		 the_robot->EDP_data.dyslocation_matrix[5][5]=0;
		 */

		return true;

		// UCZENIE

	}
	else if (teach_or_move == YG_TEACH)
	{

		// zerowanie odczytow
		(sensor_m.begin())->second->to_vsp.parameters=1;
		(sensor_m.begin())->second->configure_sensor();

		(sensor_m.begin())->second->to_vsp.parameters=4;
		(sensor_m.begin())->second->configure_sensor();

		gen_state = 0; // nie zapisuje trajektorii dopoki nie osiagnie chociaz raz podloza

		for (int i=0; i<6; i++)
			delta[i]=0.0;

		create_pose_list_head(emptyps, 0.0, delta, 2);

		td.interpolation_node_no = 1;
		td.internode_step_no = step_no;
		td.value_in_step_no = td.internode_step_no - 2;

		the_robot->ecp_command.instruction.instruction_type = lib::GET;
		the_robot->ecp_command.instruction.get_type = ARM_DV;
		the_robot->ecp_command.instruction.set_type = ARM_DV;
		/*
		 the_robot->EDP_data.force_move_mode=2;
		 the_robot->EDP_data.position_set_mode=1; // przyrostowo

		 the_robot->EDP_data.force_axis_quantity=3;

		 the_robot->ecp_command.instruction.set_arm_type = POSE_FORCE_LINEAR;
		 the_robot->ecp_command.instruction.get_arm_type = POSE_FORCE_LINEAR;
		 the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
		 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

		 the_robot->EDP_data.ECPtoEDP_force_coordinates[0]=0;
		 the_robot->EDP_data.ECPtoEDP_force_coordinates[1]=0;
		 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;

		 for (int j=0; j<6 ; j++)	{
		 the_robot->EDP_data.position_increment[j]=0.0;}
		 // zerowy przyrost pozycji

		 the_robot->EDP_data.dyslocation_matrix[0][0]=1;
		 the_robot->EDP_data.dyslocation_matrix[1][1]=1;
		 the_robot->EDP_data.dyslocation_matrix[2][2]=1;
		 the_robot->EDP_data.dyslocation_matrix[3][3]=0;
		 the_robot->EDP_data.dyslocation_matrix[4][4]=0;
		 the_robot->EDP_data.dyslocation_matrix[5][5]=0;
		 */

		return true;

	}

	return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_advanced_drawing_teach_in_force::next_step()
{

	if (teach_or_move == YG_MOVE)
	{

		ecp_taught_in_pose tip; // Nauczona pozycja

		if (!(is_pose_list_element()))
		{ // Koniec odcinka
			(sensor_m.begin())->second->to_vsp.parameters = 6;
			(sensor_m.begin())->second->configure_sensor();
			return false;

			// flush_pose_list();
			//		return false;
		}

		// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
		the_robot->ecp_command.instruction.instruction_type = lib::SET;

		get_pose(tip);

		gen_state=next_gen_state;

		switch (gen_state)
		{
		case 0:
			break;
		case 1:
			break;
		case 2: // powierzchnia

			if (prev_gen_state != gen_state)
			{
				sr_ecp_msg.message("ECP Powierzchnia");
			}

			td.interpolation_node_no = 1;
			td.internode_step_no = step_no;
			td.value_in_step_no = td.internode_step_no - 2;
			/*
			 the_robot->EDP_data.force_axis_quantity=1;

			 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

			 // inkrementacja numeru iteracji dla biezacego stanu
			 in_state_iteration++;

			 #define ST2_LOW_SEGMENT 20

			 if (in_state_iteration < ST2_LOW_SEGMENT)
			 {
			 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]= (short) (MIN_SILA_DOCISKUEDP +
			 (in_state_iteration)*(SILA_DOCISKUEDP - MIN_SILA_DOCISKUEDP) /	(ST2_LOW_SEGMENT));
			 } else
			 {
			 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP;
			 }

			 the_robot->EDP_data.position_increment[0]=tip.coordinates[0];
			 the_robot->EDP_data.position_increment[1]=tip.coordinates[1];
			 the_robot->EDP_data.position_increment[2]=0.0;
			 the_robot->EDP_data.position_increment[3]=0.0;
			 the_robot->EDP_data.position_increment[4]=0.0;
			 the_robot->EDP_data.position_increment[5]=0.0;
			 */
			if ((tip.extra_info == 3)||(tip.extra_info == 4)||(tip.extra_info
					== 5))
			{
				// nauczona trajektoria przeszla w unoszenie
				next_gen_state = 3;
				in_state_iteration = 0;
			}
			else
			{
				next_pose_list_ptr();
			}

			break;

		case 3: // unoszenie
			if (prev_gen_state != gen_state)
			{
				sr_ecp_msg.message("ECP Unoszenie");
			}

			td.interpolation_node_no = 1;
			td.internode_step_no = 500;
			td.value_in_step_no = td.internode_step_no - 2;
			/*
			 the_robot->EDP_data.force_axis_quantity=0;

			 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

			 the_robot->EDP_data.position_increment[0]=0.0;
			 the_robot->EDP_data.position_increment[1]=0.0;
			 the_robot->EDP_data.position_increment[2]=0.01;
			 the_robot->EDP_data.position_increment[3]=0.0;
			 the_robot->EDP_data.position_increment[4]=0.0;
			 the_robot->EDP_data.position_increment[5]=0.0;
			 */
			next_gen_state = 4;

			break;

		case 4: // uniesienie

			if (prev_gen_state != gen_state)
			{
				sr_ecp_msg.message("ECP Uniesienie");
			}

			td.interpolation_node_no = 1;
			td.internode_step_no = step_no;
			td.value_in_step_no = td.internode_step_no - 2;
			/*
			 the_robot->EDP_data.force_axis_quantity = 0;

			 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

			 the_robot->EDP_data.position_increment[0]=tip.coordinates[0];
			 the_robot->EDP_data.position_increment[1]=tip.coordinates[1];
			 the_robot->EDP_data.position_increment[2]=0.0;
			 the_robot->EDP_data.position_increment[3]=0.0;
			 the_robot->EDP_data.position_increment[4]=0.0;
			 the_robot->EDP_data.position_increment[5]=0.0;
			 */
			if (tip.extra_info == 2)
			{ // nauczona trajektoria osiagnela powierzchnie
				next_gen_state = 5;
				(sensor_m.begin())->second->to_vsp.parameters = 5;
				(sensor_m.begin())->second->configure_sensor();
			}
			else
			{
				next_pose_list_ptr();
			}
			break;

		case 5: // opuszczanie

			if (prev_gen_state != gen_state)
			{
				sr_ecp_msg.message("ECP Opuszczanie");
			}

			if ((sensor_m.begin())->second->image.sensor_union.force.event_type == 2)
			{
				// czujnik wyczul powierzchnie
				next_gen_state = 6;
				in_state_iteration=0;
			}
			else
			{

				td.interpolation_node_no = 1;
				td.internode_step_no = step_no;
				td.value_in_step_no = td.internode_step_no - 2;
				/*
				 the_robot->EDP_data.force_axis_quantity=1;
				 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
				 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
				 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=SILA_DOCISKUEDP_OPADANIE;

				 the_robot->EDP_data.position_increment[0]=0.0;
				 the_robot->EDP_data.position_increment[1]=0.0;
				 the_robot->EDP_data.position_increment[2]=0.0;
				 the_robot->EDP_data.position_increment[3]=0.0;
				 the_robot->EDP_data.position_increment[4]=0.0;
				 the_robot->EDP_data.position_increment[5]=0.0;
				 */
			}

			break;
		case 6: // zetkniecie z kartka

			if (prev_gen_state != gen_state)
			{
				sr_ecp_msg.message("ECP Zetkniecie");
			}

			in_state_iteration++;

			td.interpolation_node_no = 1;
			td.internode_step_no = step_no;
			td.value_in_step_no = td.internode_step_no - 2;
			/*
			 the_robot->EDP_data.force_axis_quantity=1;
			 the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			 the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

			 the_robot->EDP_data.position_increment[0]=0.0;
			 the_robot->EDP_data.position_increment[1]=0.0;
			 the_robot->EDP_data.position_increment[2]=0.0;
			 the_robot->EDP_data.position_increment[3]=0.0;
			 the_robot->EDP_data.position_increment[4]=0.0;
			 the_robot->EDP_data.position_increment[5]=0.0;

			 #define ST6_LOW_SEGMENT 100

			 // dobor sily docisku - chcemy ladnie uderzyc w powierzchnie
			 if (in_state_iteration<ST6_LOW_SEGMENT)
			 {
			 the_robot->EDP_data.ECPtoEDP_force_coordinates[2]=MIN_SILA_DOCISKUEDP;
			 } else
			 {
			 next_gen_state = 2;
			 in_state_iteration=0;
			 }
			 */
			break;
		default:

			break;
		}

		prev_gen_state=gen_state;

		return true;

		// UCZENIE

	}
	else if (teach_or_move == YG_TEACH)
	{

		int i; // licznik kolejnych wspolrzednych wektora [0..6]

		double inc_delta[6]=
		{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

		if (check_and_null_trigger())
		{ // Koniec odcinka
			initiate_pose_list();
			return false;
		}

		for (i=0; i<6; i++)
		{
			inc_delta[i]
					=-the_robot->reply_package.arm.pf_def.arm_coordinates[i];
		}

		for (i=0; i<6; i++)
		{
			inc_delta[i]
					+=the_robot->reply_package.arm.pf_def.arm_coordinates[i];
		}

		// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
		the_robot->ecp_command.instruction.instruction_type = lib::SET;

		if ((sensor_m.begin())->second->image.sensor_union.force.event_type==2)
		{
			gen_state = 1;
		}

		if (gen_state == 1)
		{
			insert_pose_list_element(emptyps, 0.0, inc_delta,
					(sensor_m.begin())->second->image.sensor_union.force.event_type);
		}

		return true;

	}

	return true;

}
// --------------------------------------------------------------------------


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

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool eih_nose_run::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	// cout << "next_step" << endl;

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
