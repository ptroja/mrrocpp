// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6s.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody wspolne dla robotow IRp-6
// 				- definicja metod klasy edp_irp6s_effector
//
// Autor:		tkornuta
// Data:		14.02.2007
// -------------------------------------------------------------------------

#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <cerrno>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/edp/servo_gr.h"
#include "base/edp/reader.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/edp_e_manip.h"
#include "base/edp/edp_force_sensor.h"
#include "base/kinematics/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace edp {
namespace common {

bool manip_effector::compute_servo_joints_and_frame(void)
{
	static bool force_sensor_post_synchro_configuration = false;

	if (!(motor_driven_effector::compute_servo_joints_and_frame())) {
		return false;
	}

	static int catch_nr = 0;

	// wyznaczenie nowych wartosci joints and frame dla obliczen w servo
	try {

		lib::Homog_matrix local_matrix;

		// Obliczenie lokalnej macierzy oraz obliczenie polozenia robota we wsp. zewnetrznych.
		get_current_kinematic_model()->i2e_transform(servo_current_joints, local_matrix);
		// Pobranie wsp. zewnetrznych w ukladzie

		lib::Xyz_Euler_Zyz_vector servo_real_kartez_pos; // by Y polozenie we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX
		local_matrix.get_xyz_euler_zyz(servo_real_kartez_pos);

		//obliczanie zadanej pozycji koncowki wedlug aktualnego rozkazu przetwarzanego w servo

		// @bug race condition (issue #69)
		while (!sb) {
			std::cerr << "Race condition detected! (" << __FILE__ << ":" << __LINE__ << ")" << std::endl;
			usleep(1000);
		}

		lib::MotorArray servo_desired_motor_pos(sb->command.parameters.move.abs_position, number_of_servos);

		lib::JointArray servo_desired_joints(number_of_servos);

		get_current_kinematic_model()->mp2i_transform(servo_desired_motor_pos, servo_desired_joints);
		get_current_kinematic_model()->i2e_transform(servo_desired_joints, local_matrix);
		// Pobranie wsp. zewnetrznych w ukladzie

		lib::Xyz_Euler_Zyz_vector servo_desired_kartez_pos; // by Y polozenie we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX
		local_matrix.get_xyz_euler_zyz(servo_desired_kartez_pos);

		// scope-locked reader data update
		{
			boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			servo_real_kartez_pos.to_table(rb_obj->step_data.real_cartesian_position);
			servo_desired_kartez_pos.to_table(rb_obj->step_data.desired_cartesian_position);
		}

		// Obliczenie polozenia robota we wsp. zewnetrznych bez narzedzia.
		((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->i2e_wo_tool_transform(servo_current_joints, servo_current_frame_wo_tool);

		if (vs != NULL) {
			boost::mutex::scoped_lock lock(vs->mtx);
			if ((is_synchronised()) && (!(force_sensor_post_synchro_configuration))) {
				force_sensor_post_synchro_configuration = true;
				vs->new_edp_command = true;
				vs->command = FORCE_CONFIGURE;
			}
		}

		catch_nr = 0;

	} //: try
	catch (...) {
		if ((++catch_nr) == 1) {
			std::cout << "servo thread compute_servo_joints_and_frame throw catch exception" << std::endl;
		}
		return false;
	} //: catch

	return true;
}

/*--------------------------------------------------------------------------*/
manip_effector::manip_effector(shell &_shell, const lib::robot_name_t & l_robot_name, lib::c_buffer & c_buffer_ref, lib::r_buffer & r_buffer_ref) :
		motor_driven_effector(_shell, l_robot_name, c_buffer_ref, r_buffer_ref)
{
}

/*--------------------------------------------------------------------------*/
void manip_effector::set_robot_model_with_sb(const lib::c_buffer &instruction)
{
	switch (instruction.robot_model.type)
	{
		case lib::SERVO_ALGORITHM:
			sb->set_robot_model_servo_algorithm(instruction);
			break;
		default: // blad: nie istniejca specyfikacja modelu robota
			// ustawi numer bledu
			manip_effector::set_robot_model(instruction);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::get_arm_position_with_force_and_sb(bool read_hardware, lib::c_buffer &instruction)
{
	if (read_hardware) {
		motor_driven_effector::get_arm_position_read_hardware_sb();

		if (is_synchronised()) {
			//  check_motor_position(desired_motor_pos_new);
			// dla sprawdzenia ograncizen w joints i motors

			// Wspolrzedne wewnetrzne
			lib::JointArray desired_joints_tmp(number_of_servos);

			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints_tmp);

			for (int i = 0; i < number_of_servos; i++) {
				desired_joints[i] = current_joints[i] = desired_joints_tmp[i];
			}
		}
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	if (is_synchronised()) {
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
		get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
	}

	// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
	// Przepisanie definicji koncowki danej w postaci
	// TRANS z wewntrznych struktur danych TRANSFORMATORa
	// do wewntrznych struktur danych REPLY_BUFFER

	reply.arm.pf_def.arm_frame = current_end_effector_frame;

	motor_driven_effector::get_arm_position_get_arm_type_switch(instruction);

	reply.servo_step = step_counter;

	lib::Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);
	lib::Ft_tr ft_tr_inv_current_frame_matrix(!current_frame_wo_offset);

	lib::Homog_matrix current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool);
	lib::Ft_tr ft_tr_inv_tool_matrix(!current_tool);

	lib::Ft_vector current_force;
	force_msr_download(current_force);

	// sprowadzenie sil z ukladu bazowego do ukladu kisci
	// modyfikacja pobranych sil w ukladzie czujnika - do ukladu wyznaczonego przez force_tool_frame i reference_frame

	reply.arm.pf_def.force_xyz_torque_xyz = ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix * current_force;
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::get_arm_position_get_arm_type_switch(lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

	// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
	get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
	// TRANS z wewntrznych struktur danych TRANSFORMATORa
	// do wewntrznych struktur danych REPLY_BUFFER

	reply.arm.pf_def.arm_frame = current_end_effector_frame;

	motor_driven_effector::get_arm_position_get_arm_type_switch(instruction);

}

/*--------------------------------------------------------------------------*/
void manip_effector::compute_base_pos_xyz_rot_xyz_vector(const lib::JointArray & begining_joints, const lib::Homog_matrix & begining_end_effector_frame, const lib::c_buffer & instruction, lib::Xyz_Angle_Axis_vector & base_pos_xyz_rot_xyz_vector)
{
	const lib::MOTION_TYPE &motion_type = instruction.motion_type;
	const lib::POSE_SPECIFICATION &set_arm_type = instruction.set_arm_type;
	lib::Homog_matrix arm_frame(instruction.arm.pf_def.arm_frame);
	lib::JointArray joint_arm_coordinates(instruction.arm.pf_def.arm_coordinates, number_of_servos);
	lib::MotorArray motor_arm_coordinates(instruction.arm.pf_def.arm_coordinates, number_of_servos);
	const uint16_t &ECP_motion_steps = instruction.motion_steps; // liczba krokow w makrokroku

	// WYLICZENIE POZYCJI POCZATKOWEJ
	lib::JointArray tmp_joints(number_of_servos);
	lib::MotorArray tmp_motor_pos(number_of_servos);

	lib::Homog_matrix goal_frame;

	lib::Homog_matrix goal_frame_increment_in_end_effector;
	lib::Xyz_Angle_Axis_vector goal_xyz_angle_axis_increment_in_end_effector;

	switch (motion_type)
	{
		case lib::ABSOLUTE:
			switch (set_arm_type)
			{
				case lib::FRAME:
					goal_frame = arm_frame;
					break;
				case lib::JOINT:
					get_current_kinematic_model()->i2e_transform(joint_arm_coordinates, goal_frame);

					break;
				case lib::MOTOR:
					get_current_kinematic_model()->mp2i_transform(motor_arm_coordinates, tmp_joints);
					get_current_kinematic_model()->i2e_transform(tmp_joints, goal_frame);

					break;
				default:
					break;
			}
			break;
		case lib::RELATIVE:
			switch (set_arm_type)
			{
				case lib::FRAME:
					goal_frame = arm_frame;
					goal_frame = begining_end_effector_frame * goal_frame;
					break;
				case lib::JOINT:
					for (int i = 0; i < number_of_servos; i++) {
						tmp_joints[i] = begining_joints[i] + joint_arm_coordinates[i];
					}
					get_current_kinematic_model()->i2e_transform(tmp_joints, goal_frame);
					break;
				case lib::MOTOR:
					for (int i = 0; i < number_of_servos; i++) {
						tmp_motor_pos[i] = desired_motor_pos_new[i] + motor_arm_coordinates[i];
					}
					get_current_kinematic_model()->mp2i_transform(tmp_motor_pos, tmp_joints);
					get_current_kinematic_model()->i2e_transform(tmp_joints, goal_frame);
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
			goal_frame_increment_in_end_effector = ((!begining_end_effector_frame) * goal_frame);
			goal_frame_increment_in_end_effector.get_xyz_angle_axis(goal_xyz_angle_axis_increment_in_end_effector);
			for (int i = 0; i < 6; i++) {
				base_pos_xyz_rot_xyz_vector[i] = goal_xyz_angle_axis_increment_in_end_effector[i]
						* (double) (1 / (((double) lib::EDP_STEP) * ((double) ECP_motion_steps)));
			}
			break;
		case lib::PF_VELOCITY:
			base_pos_xyz_rot_xyz_vector = lib::Xyz_Angle_Axis_vector(instruction.arm.pf_def.arm_coordinates);
			break;
		default:
			throw exception::se();
	}
}

/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::iterate_macrostep(const lib::JointArray & begining_joints, const lib::Homog_matrix & begining_end_effector_frame, const lib::c_buffer &instruction, const lib::Xyz_Angle_Axis_vector & base_pos_xyz_rot_xyz_vector)
{
	desired_end_effector_frame = begining_end_effector_frame;

	//	static int debugi=0;
	//   debugi++;

	lib::MotorArray desired_motor_pos_new_tmp(number_of_servos);
	lib::JointArray desired_joints_tmp(number_of_servos); // Wspolrzedne wewnetrzne -

	// zmienne z bufora wejsciowego
	const uint16_t &ECP_motion_steps = instruction.motion_steps; // liczba krokow w makrokroku
	const uint16_t &ECP_value_in_step_no = instruction.value_in_step_no; // liczba krokow po ktorych bedzie wyslana odpowiedz do ECP o przewidywanym zakonczeniu ruchu
	const lib::POSE_SPECIFICATION &set_arm_type = instruction.set_arm_type;

	double force_xyz_torque_xyz[6]; // wartosci zadana sily
	double inertia[6];
	double reciprocal_damping[6];

	const lib::BEHAVIOUR_SPECIFICATION (&behaviour)[6] = instruction.arm.pf_def.behaviour;
	//	const double &desired_gripper_coordinate = instruction.arm.pf_def.gripper_coordinate;

	// w trybie TCIM interpolujemy w edp_trans stad zadajemy pojedynczy krok do serwo
	motion_steps = 1;
	value_in_step_no = 0;

	// MODYFIKACJA PARAMETROW W ZALEZNOSCI OD ZALOZONEGO ZACHOWANIA DLA DANEGO KIERUNKU
	for (int i = 0; i < 6; i++) {
		switch (behaviour[i])
		{
			case lib::UNGUARDED_MOTION:
				inertia[i] = 0.0;
				reciprocal_damping[i] = 0.0; // the force influence is eliminated
				force_xyz_torque_xyz[i] = instruction.arm.pf_def.force_xyz_torque_xyz[i];
				// inertia is not eleliminated
				break;
			case lib::GUARDED_MOTION:
				inertia[i] = instruction.arm.pf_def.inertia[i];
				reciprocal_damping[i] = instruction.arm.pf_def.reciprocal_damping[i];
				force_xyz_torque_xyz[i] = 0.0; // the desired force is set to zero
				break;
			case lib::CONTACT:
				inertia[i] = instruction.arm.pf_def.inertia[i];
				reciprocal_damping[i] = instruction.arm.pf_def.reciprocal_damping[i];
				force_xyz_torque_xyz[i] = instruction.arm.pf_def.force_xyz_torque_xyz[i];
				break;
			default:
				break;
		}
	}

	const unsigned long PREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE = 10;

	static unsigned long last_force_step_counter = step_counter;

	lib::Xyz_Angle_Axis_vector move_rot_vector;
	lib::Xyz_Angle_Axis_vector pos_xyz_rot_xyz_vector;
	static lib::Xyz_Angle_Axis_vector previous_move_rot_vector;

	lib::Homog_matrix current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool);

	//	std::cout << current_tool << std::endl;

	lib::Ft_tr ft_tr_inv_tool_matrix = !(lib::Ft_tr(current_tool));
	lib::V_tr v_tr_tool_matrix(current_tool);
	lib::V_tr v_tr_inv_tool_matrix = !v_tr_tool_matrix;

	// poczatek generacji makrokrokubase_pos_xyz_rot_xyz_vector
	for (int step = 1; step <= ECP_motion_steps; ++step) {

		lib::Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);

		lib::V_tr v_tr_current_frame_matrix(current_frame_wo_offset);

		lib::Ft_vector current_force;
		force_msr_download(current_force);
		// sprowadzenie sil z ukladu bazowego do ukladu kisci
		// modyfikacja pobranych sil w ukladzie czujnika - do ukladu wyznaczonego przez force_tool_frame i reference_frame

		lib::Homog_matrix begining_end_effector_frame_with_current_translation = begining_end_effector_frame;
		begining_end_effector_frame_with_current_translation.set_translation_vector(desired_end_effector_frame);

		lib::Ft_v_vector current_force_torque(ft_tr_inv_tool_matrix * !(lib::Ft_tr(current_frame_wo_offset))
				* current_force);
		//		lib::Ft_v_vector tmp_force_torque (lib::Ft_v_tr((!current_tool) * (!current_frame_wo_offset), lib::Ft_v_tr::FT) * lib::Ft_v_vector (current_force));

		//wyzerowanie historii dla dlugiej przerwy w sterowaniu silowym

		if (step_counter - last_force_step_counter > PREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE) {
			//			printf("\n\nPREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE\n\n");
			previous_move_rot_vector = lib::Xyz_Angle_Axis_vector::Zero();
		} else {
			//	printf("\n\nPREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE NOT\n\n");
		}

		previous_move_rot_vector = v_tr_inv_tool_matrix * (!v_tr_current_frame_matrix) * previous_move_rot_vector;

		switch (set_arm_type)
		{
			case lib::FRAME:
			case lib::JOINT:
			case lib::MOTOR:
				pos_xyz_rot_xyz_vector = lib::V_tr(!(lib::V_tr(!begining_end_effector_frame_with_current_translation
						* desired_end_effector_frame))) * base_pos_xyz_rot_xyz_vector;
				break;
			case lib::PF_VELOCITY:
				pos_xyz_rot_xyz_vector = base_pos_xyz_rot_xyz_vector;
				break;
			default:
				break;
		}

		// wyznaczenie predkosci z uwzglednieniem wirtualnej inercji i wirtualnego tarcia wiskotycznego
		for (int i = 0; i < 6; i++) {

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
					+ pos_xyz_rot_xyz_vector[i]) * lib::EDP_STEP * lib::EDP_STEP
					+ reciprocal_damping[i] * inertia[i] * previous_move_rot_vector[i])
					/ (lib::EDP_STEP + reciprocal_damping[i] * inertia[i]);
		}

		previous_move_rot_vector = v_tr_current_frame_matrix * v_tr_tool_matrix * move_rot_vector;
		// 	end: sprowadzenie predkosci ruchu do orientacji  ukladu bazowego lub ukladu koncowki

		//		if (debugi%10==0) printf("aaa: %f\n", force_xyz_torque_xyz[0] + force_torque[0]);

		lib::Homog_matrix rot_frame(move_rot_vector);

		// wyliczenie nowej pozycji zadanej
		desired_end_effector_frame = desired_end_effector_frame * rot_frame;

		/*// przeniesione do manip_effector::compute_servo_joints_and_frame(void)
		 lib::Xyz_Euler_Zyz_vector tmp_vector;
		 //	next_frame.get_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector.to_table(rb_obj->step_data.desired_cartesian_position));
		 //	next_frame.get_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector aa);
		 next_frame.get_xyz_euler_zyz(tmp_vector);

		 // scope-locked reader data update
		 {
		 boost::mutex::scoped_lock lock(rb_obj->reader_mutex);
		 tmp_vector.to_table(rb_obj->step_data.desired_cartesian_position);
		 }
		 */

		// Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
		get_current_kinematic_model()->e2i_transform(desired_joints_tmp, desired_joints, desired_end_effector_frame);
		// Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow

		get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);
		// kinematyka nie stwierdzila bledow, przepisanie wartosci

		for (int i = 0; i < number_of_servos; i++) {
			desired_joints[i] = desired_joints_tmp[i];
			desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
		}

		move_servos();

		if (step == ECP_value_in_step_no) { // przygotowanie predicted frame dla ECP
			//  next_frame.get_frame_tab(reply.arm.pf_def.arm_frame);
			lib::Homog_matrix predicted_frame = desired_end_effector_frame;
			for (int i = 0; i < ECP_motion_steps - ECP_value_in_step_no; i++) {
				predicted_frame = predicted_frame * rot_frame;
			}
			//            predicted_frame.get_frame_tab(reply.arm.pf_def.predicted_arm_frame);
			/*
			 for (int i=0; i<6; i++)
			 {
			 reply.arm.pf_def.force_xyz_torque_xyz[i] = current_force_torque[i];
			 }
			 */
			mt_tt_obj->trans_t_to_master_synchroniser.command();
			move_arm_second_phase = true;
		}

		last_force_step_counter = step_counter;

	}
}

/*--------------------------------------------------------------------------*/
void manip_effector::pose_force_torque_at_frame_move(const lib::c_buffer &instruction)
{
	// WYLICZENIE POZYCJI POCZATKOWEJ
	lib::JointArray begining_joints(number_of_servos);
	lib::Homog_matrix begining_end_effector_frame;
	get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, begining_joints);
	get_current_kinematic_model()->i2e_transform(begining_joints, begining_end_effector_frame);

	lib::Xyz_Angle_Axis_vector base_pos_xyz_rot_xyz_vector; // wartosci ruchu pozycyjnego

	// WYZNACZENIE base_pos_xyz_rot_xyz_vector
	compute_base_pos_xyz_rot_xyz_vector(begining_joints, begining_end_effector_frame, instruction, base_pos_xyz_rot_xyz_vector);

	// macrostep iteration
	iterate_macrostep(begining_joints, begining_end_effector_frame, instruction, base_pos_xyz_rot_xyz_vector);
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::set_robot_model(const lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;
	//printf(" SET ROBOT_MODEL: ");
	switch (instruction.robot_model.type)
	{
		case lib::FORCE_TOOL:
			if (vs == NULL) {
				printf("vs == NULL\n");
				break;
			}
			{
				boost::mutex::scoped_lock lock(vs->mtx);
				vs->new_command_synchroniser.null_command();
				vs->command = FORCE_SET_TOOL;
				for (int i = 0; i < 3; i++) {
					vs->next_force_tool_position[i] = instruction.robot_model.force_tool.position[i];
				}
				vs->next_force_tool_weight = instruction.robot_model.force_tool.weight;
				vs->new_edp_command = true;
			}
			vs->new_command_synchroniser.wait();
			break;
		case lib::FORCE_BIAS:
			if (vs == NULL) {
				printf("vs == NULL\n");
				break;
			}
			{
				boost::mutex::scoped_lock lock(vs->mtx);
				vs->new_command_synchroniser.null_command();
				vs->command = FORCE_CONFIGURE;
				vs->new_edp_command = true;
			}
			vs->new_command_synchroniser.wait();
			break;
		case lib::TOOL_FRAME:
			//printf("TOOL_FRAME\n");
			// przepisa specyfikacj do TRANSFORMATORa
			// Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
			// do wewntrznych struktur danych TRANSFORMATORa
			// Sprawdzenie czy przepisana macierz jest jednorodna
			// Jezeli nie, to wyzwalany jest wyjatek.

			// Przyslano dane dotyczace narzedzia i koncowki.
			// Sprawdzenie poprawnosci macierzy
			//	set_tool_frame_in_kinematic_model(lib::Homog_matrix(instruction.robot_model.tool_frame_def.tool_frame));
		{
			const lib::Homog_matrix & hm = instruction.robot_model.tool_frame_def.tool_frame;

			if (!(hm.is_valid())) {
				BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(INVALID_HOMOGENEOUS_MATRIX));
			}
			// Ustawienie macierzy reprezentujacej narzedzie.
			// TODO: dynamic_cast<>
			((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool = hm;

			/*
			 // odswierzanie
			 get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
			 get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
			 */
		}

			break;
		default: // blad: nie istniejaca specyfikacja modelu robota
			motor_driven_effector::set_robot_model(instruction);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::get_robot_model(lib::c_buffer &instruction)
{
	//printf(" GET ROBOT_MODEL: ");
	switch (instruction.get_robot_model_type)
	{
		case lib::FORCE_TOOL:
			if (vs == NULL) {
				printf("vs == NULL\n");
				break;
			}
			for (int i = 0; i < 3; i++) {
				reply.robot_model.force_tool.position[i] = vs->current_force_tool_position[i];
			}
			reply.robot_model.force_tool.weight = vs->current_force_tool_weight;
			break;
		case lib::TOOL_FRAME:
			//printf("TOOL_FRAME\n");
			// przepisac specyfikacje z TRANSFORMATORa do bufora wysylkowego
			// Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
			// z wewntrznych struktur danych TRANSFORMATORa
			// do wewntrznych struktur danych REPLY_BUFFER

			reply.robot_model.type = lib::TOOL_FRAME;

			reply.robot_model.tool_frame_def.tool_frame =
					((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool;

			break;
		default: // blad: nie istniejaca specyfikacja modelu robota
			motor_driven_effector::get_robot_model(instruction);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::compute_frame(const lib::c_buffer &instruction)
{
	lib::MotorArray desired_motor_pos_new_tmp(number_of_servos);
	lib::JointArray desired_joints_tmp(number_of_servos); // Wspolrzedne wewnetrzne -
	const lib::MOTION_TYPE &motion_type = instruction.motion_type;
	// obliczenia dla ruchu ramienia (kocwk: FRAME)
	/* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
	/* Zlecenie transformerowi przeliczenie wspolrzednych */

	motion_steps = instruction.motion_steps;
	value_in_step_no = instruction.value_in_step_no;
	lib::Homog_matrix p_m(instruction.arm.pf_def.arm_frame);

	if ((value_in_step_no <= 0) || (motion_steps <= 0) || (value_in_step_no > motion_steps + 1)) {
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(INVALID_MOTION_PARAMETERS));

	}
	switch (motion_type)
	{
		case lib::ABSOLUTE: // ruch bezwzgledny
			desired_end_effector_frame = p_m;
			//      fprintf(stderr, "debug@%s:%d\n", __FILE__, __LINE__);
			break;
		case lib::RELATIVE: // ruch wzgledny
			//      fprintf(stderr, "debug@%s:%d\n", __FILE__, __LINE__);
			desired_end_effector_frame = current_end_effector_frame * p_m;
			break;
		default: {
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(INVALID_MOTION_TYPE));

		}
	}
	// Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
	get_current_kinematic_model()->e2i_transform(desired_joints_tmp, desired_joints, desired_end_effector_frame);
	// Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
	get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);

	// kinematyka nie stwierdzila bledow, przepisanie wartosci
	for (int i = 0; i < number_of_servos; i++) {
		desired_joints[i] = desired_joints_tmp[i];
		desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/

lib::Homog_matrix manip_effector::return_current_frame(TRANSLATION_ENUM translation_mode)
{ // by Y
	boost::mutex::scoped_lock lock(effector_mutex);
	// przepisanie danych na zestaw lokalny dla edp_force
	// lib::copy_frame(force_current_end_effector_frame, global_current_end_effector_frame);
	lib::Homog_matrix return_frame(servo_current_frame_wo_tool);

	if (translation_mode == WITHOUT_TRANSLATION)
		return_frame.remove_translation();
	return return_frame;
}

void manip_effector::force_msr_upload(const lib::Ft_vector & l_vector)
{ // by Y wgranie globalnego zestawu danych
	boost::mutex::scoped_lock lock(force_mutex);
	global_force_msr = l_vector;
}

// by Y odczytanie globalnego zestawu danych
void manip_effector::force_msr_download(lib::Ft_vector & l_vector)
{
	boost::mutex::scoped_lock lock(force_mutex);
	l_vector = global_force_msr;
}

/*--------------------------------------------------------------------------*/

// Synchronizacja robota.
void manip_effector::synchronise()
{
	motor_driven_effector::synchronise();
	get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
}

//   sprawdza stan robota
void manip_effector::get_controller_state(lib::c_buffer &instruction)
{
	motor_driven_effector::get_controller_state(instruction);
	if (is_synchronised()) {
		get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
	}
}

/*--------------------------------------------------------------------------*/
void manip_effector::single_thread_move_arm(const lib::c_buffer &instruction)
{ // przemieszczenie ramienia
// Wypenienie struktury danych transformera na podstawie parametrow polecenia
// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych

	switch (instruction.set_arm_type)
	{
		case lib::FRAME:
			compute_frame(instruction);
			move_servos();
			break;
		default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
			motor_driven_effector::single_thread_move_arm(instruction);
	}

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::multi_thread_move_arm(const lib::c_buffer &instruction)
{ // przemieszczenie ramienia
// Wypenienie struktury danych transformera na podstawie parametrow polecenia
// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych

	switch (instruction.interpolation_type)
	{
		case lib::MIM:
			switch (instruction.set_arm_type)
			{
				case lib::FRAME:
					compute_frame(instruction);
					move_servos();
					mt_tt_obj->trans_t_to_master_synchroniser.command();

					break;
				default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
					motor_driven_effector::multi_thread_move_arm(instruction);
			}
			break;
		case lib::TCIM:
			pose_force_torque_at_frame_move(instruction);

			break;
		default:
			break;
	}
}
/*--------------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

