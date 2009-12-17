// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6s.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody wspolne dla robotow IRp-6
// 				- definicja metod klasy edp_irp6s_postument_track_effector
//
// Autor:		tkornuta
// Data:		14.02.2007
// -------------------------------------------------------------------------

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "edp/common/reader.h"
#include "edp/common/edp_irp6s_postument_track.h"
#include "edp/common/servo_gr.h"
#include "lib/mathtr/mrmath.h"
#include "lib/srlib.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/edp_vsp_t.h"


namespace mrrocpp {
namespace edp {
namespace common {

/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::set_rmodel(lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;

	//printf(" SET RMODEL: ");
	switch (instruction.set_rmodel_type)
	{
	case lib::TOOL_FRAME:
		//printf("TOOL_FRAME\n");
		// przepisa specyfikacj do TRANSFORMATORa
		tool_frame_2_frame(instruction);
		break;
	case lib::ARM_KINEMATIC_MODEL:
		//printf("ARM_KINEMATIC_MODEL\n");
		// Ustawienie modelu kinematyki.
		set_kinematic_model(instruction.rmodel.kinematic_model.kinematic_model_no);
		break;
	case lib::SERVO_ALGORITHM:
		// ustawienie algorytmw serworegulacji oraz ich parametrow
		// zmiana algorytmu regulacji
		/* Uformowanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow dla procesu SERVO_GROUP */
		sb->servo_command.instruction_code = lib::SERVO_ALGORITHM_AND_PARAMETERS;
		for (int i = 0; i<number_of_servos; i++)
		{
			sb->servo_command.parameters.servo_alg_par.servo_algorithm_no[i] = servo_algorithm_ecp[i]
			                                                                                   = instruction.rmodel.servo_algorithm.servo_algorithm_no[i];
			sb->servo_command.parameters.servo_alg_par.servo_parameters_no[i] = servo_parameters_ecp[i]
			                                                                                     = instruction.rmodel.servo_algorithm.servo_parameters_no[i];
		}
		/* Wyslanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow procesowi SERVO_GROUP */
		sb->send_to_SERVO_GROUP(); //
		break;
	case lib::FORCE_TOOL:
		if(vs==NULL){
			printf("Nie w�aczono force_tryb=2 w pliku ini\n");
			break;
		}
		vs->force_sensor_set_tool = true;
		for (int i = 0; i<3; i++)
		{
			vs->next_force_tool_position[i] = instruction.rmodel.force_tool.position[i];
		}
		vs->next_force_tool_weight = instruction.rmodel.force_tool.weight;
		vs->check_for_command_execution_finish();
		break;
	case lib::FORCE_BIAS:
		if(vs==NULL){
			printf("Nie w�aczono force_tryb=2 w pliku ini\n");
			break;
		}
		vs->force_sensor_do_configure = true;
		vs->check_for_command_execution_finish();
		break;
	default: // blad: nie istniejca specyfikacja modelu robota
		// ustawi numer bledu
		throw NonFatal_error_2(INVALID_SET_RMODEL_TYPE);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::get_rmodel(lib::c_buffer &instruction)
{
	//printf(" GET RMODEL: ");
	switch (instruction.get_rmodel_type)
	{
	case lib::TOOL_FRAME:
		//printf("TOOL_FRAME\n");
		// przepisac specyfikacje z TRANSFORMATORa do bufora wysylkowego
		tool_frame_2_frame_rep();
		break;
	case lib::ARM_KINEMATIC_MODEL:
		reply.rmodel_type = lib::ARM_KINEMATIC_MODEL;
		// okreslenie numeru zestawu parametrow przelicznika kinematycznego oraz jego korektora
		reply.rmodel.kinematic_model.kinematic_model_no = get_current_kinematic_model_no();
		break;
	case lib::SERVO_ALGORITHM:
		reply.rmodel_type = lib::SERVO_ALGORITHM;
		// ustawienie numeru algorytmu serworegulatora oraz numeru jego zestawu parametrow
		for (int i = 0; i<number_of_servos; i++)
		{
			reply.rmodel.servo_algorithm.servo_algorithm_no[i] = servo_algorithm_sg[i];
			reply.rmodel.servo_algorithm.servo_parameters_no[i] = servo_parameters_sg[i];
		}
		break;

	case lib::FORCE_TOOL:
		for (int i = 0; i<3; i++)
		{
			reply.rmodel.force_tool.position[i] = vs->current_force_tool_position[i];
		}
		reply.rmodel.force_tool.weight = vs->current_force_tool_weight;
		break;
	default: // blad: nie istniejaca specyfikacja modelu robota
		// ustawie numer bledu
		throw NonFatal_error_2(INVALID_GET_RMODEL_TYPE);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
irp6s_postument_track_effector::irp6s_postument_track_effector(lib::configurator &_config, lib::robot_name_t l_robot_name) :
	manip_effector(_config, l_robot_name)
	{

	pthread_mutex_init(&force_mutex, NULL);

	// czujnik sil nie zostal jeszcze skonfigurowany po synchronizacji robota


	if (config.exists("force_tryb"))
		force_tryb = config.return_int_value("force_tryb");
	else
		force_tryb = 0;

	// ustalenie ilosci stopni swobody dla funkcji obslugi przerwania i synchronizacji w zaleznosci od aktywnosci chwytaka
	if (config.exists("is_gripper_active"))
		is_gripper_active = config.return_int_value("is_gripper_active");
	else
		is_gripper_active = 1;

	sem_init( &force_master_sem, 0, 0);

	}


/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::create_threads()
{
#ifdef __QNXNTO__
	// jesli wlaczono obsluge sily
	if (force_tryb > 0)
	{

		vs = sensor::return_created_edp_force_sensor(*this); //!< czujnik wirtualny

		edp_vsp_obj = new edp_vsp(*this); //!< czujnik wirtualny

		// byY - utworzenie watku pomiarow sily
		vs->create_thread();

		sem_wait(&force_master_sem);

		// by Y - utworzenie watku komunikacji miedzy EDP a VSP
		edp_vsp_obj->create_thread();
	}
#endif
	manip_and_conv_effector::create_threads();
}



/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::pose_force_torque_at_frame_move(lib::c_buffer &instruction)
{
	//	static int debugi=0;
	//   debugi++;

	double desired_motor_pos_new_tmp[MAX_SERVOS_NR];

	motion_type = instruction.motion_type;

	// zmienne z bufora wejsciowego
	const uint16_t &ECP_motion_steps = instruction.motion_steps; // liczba krokow w makrokroku
	const uint16_t &ECP_value_in_step_no = instruction.value_in_step_no; // liczba krokow po ktorych bedzie wyslana odpowiedz do ECP o przewidywanym zakonczeniu ruchu
	const lib::POSE_SPECIFICATION &set_arm_type = instruction.set_arm_type;

	double (&force_xyz_torque_xyz)[6] = instruction.arm.pf_def.force_xyz_torque_xyz; // wartosci zadana sily
	double (&inertia)[6] = instruction.arm.pf_def.inertia;
	double (&reciprocal_damping)[6] = instruction.arm.pf_def.reciprocal_damping;
	const lib::BEHAVIOUR_SPECIFICATION (&behaviour)[6] = instruction.arm.pf_def.behaviour;
	const double &desired_gripper_coordinate = instruction.arm.pf_def.gripper_coordinate;
	const double (&arm_coordinates)[MAX_SERVOS_NR] = instruction.arm.pf_def.arm_coordinates;
	const lib::frame_tab &arm_frame = instruction.arm.pf_def.arm_frame;

	// w trybie TCIM interpolujemy w edp_trans stad zadajemy pojedynczy krok do serwo
	motion_steps = 1;
	value_in_step_no = 0;

	// MODYFIKACJA PARAMETROW W ZALEZNOSCI OD ZALOZONEGO ZACHOWANIA DLA DANEGO KIERUNKU
	for (int i = 0; i < 6; i++)
	{
		switch (behaviour[i])
		{
		case lib::UNGUARDED_MOTION:
			reciprocal_damping[i]= 0.0; // the force influence is eliminated
			// inertia is not eleliminated
			break;
		case lib::GUARDED_MOTION:
			force_xyz_torque_xyz[i] = 0.0; // the desired force is set to zero
			break;
		default:
			break;
		}
	}

	double current_force[6], previous_force[6];

	double beginning_gripper_coordinate;
	static double ending_gripper_coordinate;
	static lib::frame_tab local_force_end_effector_frame;
	const unsigned long PREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE = 10;
	lib::Ft_v_vector base_pos_xyz_rot_xyz_vector; // wartosci ruchu pozycyjnego


	static unsigned long last_force_step_counter = step_counter;

	lib::Ft_v_vector move_rot_vector;
	lib::Ft_v_vector pos_xyz_rot_xyz_vector;
	static lib::Ft_v_vector previous_move_rot_vector;

	// WYLICZENIE POZYCJI POCZATKOWEJ
	double begining_joints[MAX_SERVOS_NR], tmp_joints[MAX_SERVOS_NR], tmp_motor_pos[MAX_SERVOS_NR];
	lib::frame_tab begining_frame;

	get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, begining_joints);
	get_current_kinematic_model()->i2e_transform(begining_joints, &begining_frame);
	lib::Homog_matrix begining_end_effector_frame(begining_frame);
	lib::Homog_matrix next_frame = begining_end_effector_frame;

	// WYZNACZENIE goal_frame
	lib::frame_tab goal_frame_tab;
	lib::Homog_matrix goal_frame;

	lib::Homog_matrix goal_frame_increment_in_end_effector;
	lib::Ft_v_vector goal_xyz_angle_axis_increment_in_end_effector;


	switch (motion_type)
	{
	case lib::ABSOLUTE:
		switch (set_arm_type)
		{
		case lib::FRAME:
			goal_frame.set_frame_tab(arm_frame);
			break;
		case lib::JOINT:
			get_current_kinematic_model()->i2e_transform(arm_coordinates, &goal_frame_tab);
			goal_frame.set_frame_tab(goal_frame_tab);
			break;
		case lib::MOTOR:
			get_current_kinematic_model()->mp2i_transform(arm_coordinates, tmp_joints);
			get_current_kinematic_model()->i2e_transform(tmp_joints, &goal_frame_tab);
			goal_frame.set_frame_tab(goal_frame_tab);
			break;
		default:
			break;
		}
		break;
		case lib::RELATIVE:
			switch (set_arm_type)
			{
			case lib::FRAME:
				goal_frame.set_frame_tab(arm_frame);
				goal_frame = begining_end_effector_frame * goal_frame;
				break;
			case lib::JOINT:
				for (int i = 0; i < MAX_SERVOS_NR; i++)
				{
					tmp_joints[i] = begining_joints[i] + arm_coordinates[i];
				}
				get_current_kinematic_model()->i2e_transform(tmp_joints, &goal_frame_tab);
				goal_frame.set_frame_tab(goal_frame_tab);
				break;
			case lib::MOTOR:
				for (int i = 0; i < MAX_SERVOS_NR; i++)
				{
					tmp_motor_pos[i] = desired_motor_pos_new[i] + arm_coordinates[i];
				}
				get_current_kinematic_model()->mp2i_transform(tmp_motor_pos, tmp_joints);
				get_current_kinematic_model()->i2e_transform(tmp_joints, &goal_frame_tab);
				goal_frame.set_frame_tab(goal_frame_tab);
				break;
			default:
				break;
			}
			break;
			default:
				break;

	}

	// WYZNACZENIE PREDKOSCI RUCHU

	switch (set_arm_type)
	{
	case lib::FRAME:
	case lib::JOINT:
	case lib::MOTOR:
		goal_frame_increment_in_end_effector = ((!begining_end_effector_frame)*goal_frame);
		goal_frame_increment_in_end_effector.get_xyz_angle_axis(goal_xyz_angle_axis_increment_in_end_effector);
		for (int i = 0; i < 6; i++)
		{
			base_pos_xyz_rot_xyz_vector[i] = goal_xyz_angle_axis_increment_in_end_effector[i] * (double) (1/ ( ((double)STEP)
					*((double)ECP_motion_steps) ) );
		}
		break;
	case lib::PF_VELOCITY:
		for (int i = 0; i < 6; i++)
		{
			pos_xyz_rot_xyz_vector[i] = arm_coordinates[i];
		}
		break;
	default:
		throw System_error();
	}


	//   lib::copy_frame (reply.arm.pf_def.beggining_arm_frame, begining_frame);

	beginning_gripper_coordinate = begining_joints[gripper_servo_nr];

	lib::Homog_matrix current_tool(get_current_kinematic_model()->tool);

	//	std::cout << current_tool << std::endl;

	lib::Ft_v_tr ft_tr_tool_matrix(current_tool, lib::Ft_v_tr::FT);
	lib::Ft_v_tr ft_tr_inv_tool_matrix = !ft_tr_tool_matrix;
	lib::Ft_v_tr v_tr_tool_matrix(current_tool, lib::Ft_v_tr::V);
	lib::Ft_v_tr v_tr_inv_tool_matrix = !v_tr_tool_matrix;

	// poczatek generacji makrokroku
	for (int step = 1; step <= ECP_motion_steps; step++)
	{

		lib::Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);

		lib::Ft_v_tr ft_tr_current_frame_matrix(current_frame_wo_offset, lib::Ft_v_tr::FT);
		lib::Ft_v_tr ft_tr_inv_current_frame_matrix = !ft_tr_current_frame_matrix;
		lib::Ft_v_tr v_tr_current_frame_matrix(current_frame_wo_offset, lib::Ft_v_tr::V);
		lib::Ft_v_tr v_tr_inv_current_frame_matrix = !v_tr_current_frame_matrix;

		force_msr_download(current_force, previous_force);
		// sprowadzenie sil z ukladu bazowego do ukladu kisci
		// modyfikacja pobranych sil w ukladzie czujnika - do ukladu wyznaczonego przez force_tool_frame i reference_frame


		lib::Homog_matrix begining_end_effector_frame_with_current_translation = begining_end_effector_frame;
		begining_end_effector_frame_with_current_translation.set_translation_vector(next_frame);

		lib::Homog_matrix modified_beginning_to_desired_end_effector_frame =
			!begining_end_effector_frame_with_current_translation * next_frame;

		lib::Ft_v_tr
		v_tr_modified_beginning_to_desired_end_effector_frame(modified_beginning_to_desired_end_effector_frame, lib::Ft_v_tr::V);
		lib::Ft_v_tr v_tr_inv_modified_beginning_to_desired_end_effector_frame =
			!v_tr_modified_beginning_to_desired_end_effector_frame;


		lib::Ft_v_vector current_force_torque(ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix
				* lib::Ft_v_vector(current_force));
		//		lib::Ft_v_vector tmp_force_torque (lib::Ft_v_tr((!current_tool) * (!current_frame_wo_offset), lib::Ft_v_tr::FT) * lib::Ft_v_vector (current_force));
		lib::Ft_v_vector previous_force_torque(ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix
				* lib::Ft_v_vector(previous_force));


		//wyzerowanie historii dla dlugiej przerwy w sterowaniu silowym

		if (step_counter - last_force_step_counter > PREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE)
		{
			//			printf("\n\nPREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE\n\n");
			previous_move_rot_vector.set_values(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		}
		else
		{
			//	printf("\n\nPREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE NOT\n\n");
		}

		previous_move_rot_vector = v_tr_inv_tool_matrix * v_tr_inv_current_frame_matrix * previous_move_rot_vector;

		switch (set_arm_type)
		{
		case lib::FRAME:
		case lib::JOINT:
		case lib::MOTOR:
			pos_xyz_rot_xyz_vector = v_tr_inv_modified_beginning_to_desired_end_effector_frame
			* base_pos_xyz_rot_xyz_vector;
			break;
		default:
			break;
		}

		// wyznaczenie predkosci z uwzglednieniem wirtualnej inercji i wirtualnego tarcia wiskotycznego
		for (int i = 0; i < 6; i++)
		{

			// MODYFIKACJA PARAMETROW W ZALEZNOSCI OD ZALOZONEGO ZACHOWANIA DLA DANEGO KIERUNKU
			switch (behaviour[i])
			{
			case lib::CONTACT:
				pos_xyz_rot_xyz_vector[i] = 0.0; // the desired velocity is set to zero
				break;
			default:
				break;
			}

			// PRAWO STEROWANIA
			move_rot_vector[i] = ((reciprocal_damping[i] * (force_xyz_torque_xyz[i] - current_force_torque[i])
					+ pos_xyz_rot_xyz_vector[i] ) * STEP * STEP + reciprocal_damping[i] * inertia[i]
					                                                                              * previous_move_rot_vector[i]) / (STEP + reciprocal_damping[i] * inertia[i]);
		}


		previous_move_rot_vector = v_tr_current_frame_matrix * v_tr_tool_matrix * move_rot_vector;
		// 	end: sprowadzenie predkosci ruchu do orientacji  ukladu bazowego lub ukladu koncowki

		//		if (debugi%10==0) printf("aaa: %f\n", force_xyz_torque_xyz[0] + force_torque[0]);

		lib::Homog_matrix rot_frame(lib::Homog_matrix::MTR_XYZ_ANGLE_AXIS, move_rot_vector);

		// wyliczenie nowej pozycji zadanej
		next_frame = next_frame * rot_frame;

		// scope-locked reader data update
		{
			boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			next_frame.get_xyz_euler_zyz(rb_obj->step_data.current_cartesian_position);
		}

		next_frame.get_frame_tab(desired_end_effector_frame);



		switch (motion_type)
		{
			case lib::ABSOLUTE:
				desired_joints_tmp[gripper_servo_nr] = beginning_gripper_coordinate + (((desired_gripper_coordinate
								- beginning_gripper_coordinate) / ECP_motion_steps) * step);
			break;
			case lib::RELATIVE:
				desired_joints_tmp[gripper_servo_nr] = beginning_gripper_coordinate +
					((desired_gripper_coordinate / ECP_motion_steps) * step);
			break;
			default:
			break;
		}

		// Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
		get_current_kinematic_model()->e2i_transform(desired_joints_tmp, current_joints, &desired_end_effector_frame);
		// Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
		get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);
		// kinematyka nie stwierdzila bledow, przepisanie wartosci


		for (int i=0; i< number_of_servos; i++)
		{
			desired_joints[i] = desired_joints_tmp[i];
			desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
		}
		move_servos();
		if (step == ECP_value_in_step_no)
		{ // przygotowanie predicted frame dla ECP
			//  next_frame.get_frame_tab(reply.arm.pf_def.arm_frame);
			lib::Homog_matrix predicted_frame = next_frame;
			for (int i=0; i< ECP_motion_steps - ECP_value_in_step_no; i++)
			{
				predicted_frame = predicted_frame * rot_frame;
			}
			//            predicted_frame.get_frame_tab(reply.arm.pf_def.predicted_arm_frame);
			/*
             for (int i=0; i<6; i++)
             {
             reply.arm.pf_def.force_xyz_torque_xyz[i] = current_force_torque[i];
             }
			 */
			mt_tt_obj->trans_t_to_master_order_status_ready();
		}

		last_force_step_counter = step_counter;

	}

	next_frame.get_frame_tab(local_force_end_effector_frame);
	ending_gripper_coordinate = desired_gripper_coordinate;

}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::move_arm(lib::c_buffer &instruction)
{ // przemieszczenie ramienia
	// Wypenienie struktury danych transformera na podstawie parametrow polecenia
	// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych

	if (instruction.interpolation_type == lib::MIM)
	{
		switch (instruction.set_arm_type)
		{
		case lib::MOTOR:
			compute_motors(instruction);
			move_servos();
			mt_tt_obj->trans_t_to_master_order_status_ready();
			break;
		case lib::JOINT:
			compute_joints(instruction);
			move_servos();
			mt_tt_obj->trans_t_to_master_order_status_ready();
			break;
		case lib::FRAME:
			compute_frame(instruction);
			move_servos();
			mt_tt_obj->trans_t_to_master_order_status_ready();
			break;
		default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
		throw NonFatal_error_2(INVALID_SET_END_EFFECTOR_TYPE);
		}
	}
	else if (instruction.interpolation_type == lib::TCIM)
	{
		pose_force_torque_at_frame_move(instruction);
	}

	// by Y - uwaga na wyjatki, po rzuceniu wyjatku nie zostanie zaktualizowany previous_set_arm_type
	previous_set_arm_type = instruction.set_arm_type;

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

	//   printf(" GET ARM\n");

	double current_force[6];

	if (read_hardware)
	{
		// Uformowanie rozkazu odczytu dla SERVO_GROUP
		sb->servo_command.instruction_code = lib::READ;
		// Wyslanie rozkazu do SERVO_GROUP
		// Pobranie z SERVO_GROUP aktualnej pozycji silnikow
		//		printf("get_arm_position read_hardware\n");

		sb->send_to_SERVO_GROUP();

		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for (int i = 0; i < number_of_servos; i++)
		{
			desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
		}

		if (is_synchronised())
		{
			//  check_motor_position(desired_motor_pos_new);
			// dla sprawdzenia ograncizen w joints i motors

			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints_tmp);

			for (int i=0; i< number_of_servos; i++)
			{
				desired_joints[i] = current_joints[i] = desired_joints_tmp[i];
			}

		}

	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi

	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
	get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);

	switch (instruction.get_arm_type)
	{
	case lib::FRAME:
		// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
		arm_frame_2_frame();
		break;

	case lib::JOINT:
		// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
		arm_joints_2_joints();
		break;
	case lib::MOTOR:
		arm_motors_2_motors();
		break;
	default: // blad: nieznany sposob zapisu wspolrzednych koncowki
		printf("EFF_TYPE: %d\n", instruction.get_arm_type);
		throw NonFatal_error_2(INVALID_GET_END_EFFECTOR_TYPE);
	}

	if (instruction.interpolation_type == lib::TCIM)
	{
		lib::Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);
		lib::Ft_v_tr ft_tr_inv_current_frame_matrix(!current_frame_wo_offset, lib::Ft_v_tr::FT);

		lib::Homog_matrix current_tool(get_current_kinematic_model()->tool);
		lib::Ft_v_tr ft_tr_inv_tool_matrix(!current_tool, lib::Ft_v_tr::FT);

		force_msr_download(current_force, NULL);
		// sprowadzenie sil z ukladu bazowego do ukladu kisci
		// modyfikacja pobranych sil w ukladzie czujnika - do ukladu wyznaczonego przez force_tool_frame i reference_frame

		lib::Ft_v_vector current_force_torque(ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix
				* lib::Ft_v_vector(current_force));
		current_force_torque.to_table(reply.arm.pf_def.force_xyz_torque_xyz);

		if ((robot_name == lib::ROBOT_IRP6_ON_TRACK) || (robot_name == lib::ROBOT_IRP6_POSTUMENT))
		{
			reply.arm.pf_def.gripper_coordinate = current_joints[gripper_servo_nr];
			reply.arm.pf_def.gripper_reg_state = servo_gripper_reg_state;
		}
	}

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

		reply.servo_step=rb_obj->step_data.step;
	}
}
/*--------------------------------------------------------------------------*/

// sprawdza stan EDP zaraz po jego uruchomieniu

void irp6s_postument_track_effector::servo_joints_and_frame_actualization_and_upload(void)
{
	static int catch_nr=0;
	// Wyznaczenie nowych wartosci joints and frame dla obliczen w servo.
	try
	{
		{
			boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);
			get_current_kinematic_model()->mp2i_transform(servo_current_motor_pos, servo_current_joints);
		}


		// scope-locked reader data update
		{
			boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			for (int j = 0; j < number_of_servos; j++)
			{
				rb_obj->step_data.current_joints[j] = servo_current_joints[j];
			}
		}

		// Obliczenie lokalnej macierzy oraz obliczenie położenia robota we wsp. zewnętrznych.
		lib::frame_tab local_frame;
		get_current_kinematic_model()->i2e_transform(servo_current_joints, &local_frame);
		// Pobranie wsp. zewnętrznych w układzie
		lib::Homog_matrix local_matrix(local_frame);
		local_matrix.get_xyz_euler_zyz(servo_real_kartez_pos);

		// Zapisanie wartosci rzeczywistej dla readera
		// scope-locked reader data update
		{
			boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			for (int i=0; i<6; i++)
			{
				rb_obj->step_data.real_cartesian_position[i] = servo_real_kartez_pos[i];
				rb_obj->step_data.real_cartesian_vel[i] = servo_real_kartez_vel[i];
				rb_obj->step_data.real_cartesian_acc[i] = servo_real_kartez_acc[i];
			}
		}

		// Obliczenie polozenia robota we wsp. zewnetrznych bez narzedzia.
		get_current_kinematic_model()->i2e_wo_tool_transform(servo_current_joints, &servo_current_frame_wo_tool);

		if ( (force_tryb> 0)&&(is_synchronised())&&(!(vs->first_configure_done))&&(!(vs->force_sensor_do_first_configure)))
		{
			vs->force_sensor_do_first_configure = true;
		}

		catch_nr=0;
	}

	catch (...)
	{
		if ((++catch_nr) == 1)
			printf("servo thread servo_joints_and_frame_actualization_and_upload throw catch exception\n");
	}

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// przepisnie danych na zestaw globalny
		for (int i=0; i < number_of_servos; i++)
		{
			global_current_motor_pos[i]=servo_current_motor_pos[i];
			global_current_joints[i]=servo_current_joints[i];
		}

		lib::copy_frame(global_current_frame_wo_tool, servo_current_frame_wo_tool);
	}
}

lib::Homog_matrix irp6s_postument_track_effector::return_current_frame(TRANSLATION_ENUM translation_mode)
{// by Y
	boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);
	// przepisanie danych na zestaw lokalny dla edp_force
	// lib::copy_frame(force_current_end_effector_frame, global_current_end_effector_frame);
	lib::Homog_matrix return_frame(global_current_frame_wo_tool);

	if (translation_mode == WITHOUT_TRANSLATION)
		return_frame.remove_translation();
	return return_frame;
}

void irp6s_postument_track_effector::force_msr_upload(const double *new_value)
{// by Y wgranie globalnego zestawu danych
	pthread_mutex_lock( &force_mutex);
	for (int i=0; i<=5; i++)
	{
		prevoius_global_kartez_force_msr[i]=global_kartez_force_msr[i];
		global_kartez_force_msr[i]=new_value[i];
		// 		printf("ALARM\n");
	}
	pthread_mutex_unlock( &force_mutex);
}

// by Y odczytanie globalnego zestawu danych
void irp6s_postument_track_effector::force_msr_download(double *new_value, double *old_value)
{
	pthread_mutex_lock( &force_mutex);
	for (int i=0; i<=5; i++)
	{
		if (new_value)
			new_value[i]=global_kartez_force_msr[i];
		if (old_value)
			old_value[i]=prevoius_global_kartez_force_msr[i];
	}
	pthread_mutex_unlock( &force_mutex);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
