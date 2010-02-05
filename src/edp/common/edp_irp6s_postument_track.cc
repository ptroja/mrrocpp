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
#include <semaphore.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "edp/common/reader.h"
#include "edp/common/edp_irp6s_postument_track.h"
#include "edp/common/servo_gr.h"
#include "lib/mrmath/mrmath.h"
#include "lib/srlib.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/edp_vsp_t.h"
#include "kinematics/common/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace edp {
namespace common {

void irp6s_postument_track_effector::master_order(MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::multi_thread_master_order(nm_task, nm_tryb);
}

/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::compute_frame(const lib::c_buffer &instruction)
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
		throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
	}
	switch (motion_type)
	{
		case lib::ABSOLUTE: // ruch bezwzgledny
			// dla robotow track i postument - oblicz chwytak

			desired_joints_tmp[gripper_servo_nr] = instruction.arm.pf_def.gripper_coordinate;

			desired_end_effector_frame = p_m;
			//      fprintf(stderr, "debug@%s:%d\n", __FILE__, __LINE__);
			break;
		case lib::RELATIVE: // ruch wzgledny
			// dla robotow track i postument - oblicz chwytak

			desired_joints_tmp[gripper_servo_nr] = instruction.arm.pf_def.gripper_coordinate
					+ current_joints[gripper_servo_nr];

			//      fprintf(stderr, "debug@%s:%d\n", __FILE__, __LINE__);
			desired_end_effector_frame = current_end_effector_frame * p_m;
			break;
		default:
			throw NonFatal_error_2(INVALID_MOTION_TYPE);
	}
	// Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
	get_current_kinematic_model()->e2i_transform(desired_joints_tmp, current_joints, desired_end_effector_frame);
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
void irp6s_postument_track_effector::set_rmodel(lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;

	//printf(" SET RMODEL: ");
	switch (instruction.set_rmodel_type)
	{
		case lib::SERVO_ALGORITHM:
			sb->set_rmodel_servo_algorithm(instruction);
			break;
		default: // blad: nie istniejca specyfikacja modelu robota
			// ustawi numer bledu
			manip_effector::set_rmodel(instruction);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
irp6s_postument_track_effector::irp6s_postument_track_effector(lib::configurator &_config, lib::robot_name_t l_robot_name) :
	manip_effector(_config, l_robot_name)
{
	// czujnik sil nie zostal jeszcze skonfigurowany po synchronizacji robota

	if (config.exists("force_tryb"))
		force_tryb = config.value <int> ("force_tryb");
	else
		force_tryb = 0;

	// ustalenie ilosci stopni swobody dla funkcji obslugi przerwania i synchronizacji w zaleznosci od aktywnosci chwytaka
	if (config.exists("is_gripper_active"))
		is_gripper_active = config.value <int> ("is_gripper_active");
	else
		is_gripper_active = 1;

}

/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::create_threads()
{
#ifdef __QNXNTO__
	// jesli wlaczono obsluge sily
	if (force_tryb > 0) {

		vs = sensor::return_created_edp_force_sensor(*this); //!< czujnik wirtualny

		edp_vsp_obj = new edp_vsp(*this); //!< czujnik wirtualny

		// byY - utworzenie watku pomiarow sily
		new boost::thread(boost::bind(&sensor::force::operator(), vs));

		vs->thread_started.wait();

		// by Y - utworzenie watku komunikacji miedzy EDP a VSP
		new boost::thread(*edp_vsp_obj);
	}
#endif
	motor_driven_effector::hi_create_threads();
}

/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::iterate_macrostep(const lib::JointArray begining_joints, const lib::Homog_matrix begining_end_effector_frame, const lib::c_buffer &instruction, const lib::Xyz_Angle_Axis_vector base_pos_xyz_rot_xyz_vector)
{
	desired_end_effector_frame = begining_end_effector_frame;

	//	static int debugi=0;
	//   debugi++;

	lib::MotorArray desired_motor_pos_new_tmp(number_of_servos);
	lib::JointArray desired_joints_tmp(number_of_servos); // Wspolrzedne wewnetrzne -
	const lib::MOTION_TYPE &motion_type = instruction.motion_type;

	// zmienne z bufora wejsciowego
	const uint16_t &ECP_motion_steps = instruction.motion_steps; // liczba krokow w makrokroku
	const uint16_t &ECP_value_in_step_no = instruction.value_in_step_no; // liczba krokow po ktorych bedzie wyslana odpowiedz do ECP o przewidywanym zakonczeniu ruchu
	const lib::POSE_SPECIFICATION &set_arm_type = instruction.set_arm_type;

	double force_xyz_torque_xyz[6]; // wartosci zadana sily
	double inertia[6];
	double reciprocal_damping[6];

	const lib::BEHAVIOUR_SPECIFICATION (&behaviour)[6] = instruction.arm.pf_def.behaviour;
	const double &desired_gripper_coordinate = instruction.arm.pf_def.gripper_coordinate;

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

	lib::Ft_vector current_force;

	const unsigned long PREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE = 10;

	static unsigned long last_force_step_counter = step_counter;

	lib::Xyz_Angle_Axis_vector move_rot_vector;
	lib::Xyz_Angle_Axis_vector pos_xyz_rot_xyz_vector;
	static lib::Xyz_Angle_Axis_vector previous_move_rot_vector;

	double beginning_gripper_coordinate = begining_joints[gripper_servo_nr];

	lib::Homog_matrix
			current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool);

	//	std::cout << current_tool << std::endl;


	lib::Ft_tr ft_tr_inv_tool_matrix = !(lib::Ft_tr(current_tool));
	lib::V_tr v_tr_tool_matrix(current_tool);
	lib::V_tr v_tr_inv_tool_matrix = !v_tr_tool_matrix;

	// poczatek generacji makrokrokubase_pos_xyz_rot_xyz_vector
	for (int step = 1; step <= ECP_motion_steps; ++step) {

		lib::Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);

		lib::V_tr v_tr_current_frame_matrix(current_frame_wo_offset);

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
			previous_move_rot_vector.set_values(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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
					+ pos_xyz_rot_xyz_vector[i]) * STEP * STEP + reciprocal_damping[i] * inertia[i]
					* previous_move_rot_vector[i]) / (STEP + reciprocal_damping[i] * inertia[i]);
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

		switch (motion_type)
		{
			case lib::ABSOLUTE:
				desired_joints_tmp[gripper_servo_nr] = beginning_gripper_coordinate + (((desired_gripper_coordinate
						- beginning_gripper_coordinate) / ECP_motion_steps) * step);
				break;
			case lib::RELATIVE:
				desired_joints_tmp[gripper_servo_nr] = beginning_gripper_coordinate + ((desired_gripper_coordinate
						/ ECP_motion_steps) * step);
				break;
			default:
				break;
		}

		// Przeliczenie wspolrzednych zewnetrznych na wspolrzedne wewnetrzne
		get_current_kinematic_model()->e2i_transform(desired_joints_tmp, current_joints, desired_end_effector_frame);
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
		}

		last_force_step_counter = step_counter;

	}

}

/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::move_arm(lib::c_buffer &instruction)
{ // przemieszczenie ramienia
	// Wypenienie struktury danych transformera na podstawie parametrow polecenia
	// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych

	manip_effector::multi_thread_move_arm(instruction);

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void irp6s_postument_track_effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

	//   printf(" GET ARM\n");
	//	lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	lib::Ft_vector current_force;
	lib::JointArray desired_joints_tmp(number_of_servos); // Wspolrzedne wewnetrzne -

	if (read_hardware) {
		motor_driven_effector::get_arm_position_read_hardware_sb();

		desired_motor_pos_new[gripper_servo_nr] = desired_motor_pos_old[gripper_servo_nr]
				= current_motor_pos[gripper_servo_nr];

		if (is_synchronised()) {
			//  check_motor_position(desired_motor_pos_new);
			// dla sprawdzenia ograncizen w joints i motors

			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints_tmp);

			for (int i = 0; i < number_of_servos; i++) {
				desired_joints[i] = current_joints[i] = desired_joints_tmp[i];
			}

		}
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi

	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
	get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);

	switch (instruction.get_arm_type)
	{
		case lib::FRAME:
			// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
			// Przepisanie definicji koncowki danej w postaci
			// TRANS z wewntrznych struktur danych TRANSFORMATORa
			// do wewntrznych struktur danych REPLY_BUFFER

			current_end_effector_frame.get_frame_tab(reply.arm.pf_def.arm_frame);

			// dla robotow track i postument - oblicz chwytak

			reply.arm.pf_def.gripper_coordinate = current_joints[gripper_servo_nr];

			break;
		default: // blad: nieznany sposob zapisu wspolrzednych koncowki
			motor_driven_effector::get_arm_position_get_arm_type_switch(instruction);
	}

	reply.servo_step = step_counter;

	if (instruction.interpolation_type == lib::TCIM) {
		lib::Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);
		lib::Ft_tr ft_tr_inv_current_frame_matrix(!current_frame_wo_offset);

		lib::Homog_matrix
				current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool);
		lib::Ft_tr ft_tr_inv_tool_matrix(!current_tool);

		force_msr_download(current_force);
		// sprowadzenie sil z ukladu bazowego do ukladu kisci
		// modyfikacja pobranych sil w ukladzie czujnika - do ukladu wyznaczonego przez force_tool_frame i reference_frame

		lib::Ft_v_vector current_force_torque(ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix * current_force);
		current_force_torque.to_table(reply.arm.pf_def.force_xyz_torque_xyz);

		reply.arm.pf_def.gripper_coordinate = current_joints[gripper_servo_nr];

	}
}
/*--------------------------------------------------------------------------*/

// sprawdza stan EDP zaraz po jego uruchomieniu

bool irp6s_postument_track_effector::compute_servo_joints_and_frame(void)
{
	bool ret_val = true;

	if (!(manip_effector::compute_servo_joints_and_frame())) {
		ret_val = false;
	} else if (vs != NULL) {

		boost::mutex::scoped_lock lock(vs->mtx);
		if ((force_tryb > 0) && (is_synchronised()) && (!(vs->is_sensor_configured))) {
			vs->new_edp_command = true;
			vs->command = FORCE_CONFIGURE;
		}

	}
	return ret_val;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
