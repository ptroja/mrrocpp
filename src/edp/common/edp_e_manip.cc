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

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <errno.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

#include "edp/common/reader.h"
#include "edp/common/edp_e_manip.h"

#include "kinematics/common/kinematic_model_with_tool.h"

using std::cout;

namespace mrrocpp {
namespace edp {
namespace common {

void manip_effector::master_order(MT_ORDER nm_task, int nm_tryb)
{
	manip_and_conv_effector::master_order(nm_task, nm_tryb);
}

/*--------------------------------------------------------------------------*/
manip_effector::manip_effector(lib::configurator &_config, lib::robot_name_t l_robot_name) :
	manip_and_conv_effector(_config, l_robot_name)
{
}

/*--------------------------------------------------------------------------*/
void manip_effector::set_rmodel(lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;
	//printf(" SET RMODEL: ");
	switch (instruction.set_rmodel_type)
	{
		case lib::TOOL_FRAME:
			//printf("TOOL_FRAME\n");
			// przepisa specyfikacj do TRANSFORMATORa
			// Przepisanie definicji narzedzia danej w postaci TOOL_FRAME
			// do wewntrznych struktur danych TRANSFORMATORa
			// Sprawdzenie czy przepisana macierz jest jednorodna
			// Jezeli nie, to wyzwalany jest wyjatek.


			// Przyslano dane dotyczace narzedzia i koncowki.
			// Sprawdzenie poprawnosci macierzy
			set_tool_frame_in_kinematic_model(lib::Homog_matrix(instruction.rmodel.tool_frame_def.tool_frame));
			break;
		default: // blad: nie istniejaca specyfikacja modelu robota
			manip_and_conv_effector::set_rmodel(instruction);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_effector::compute_frame(const lib::c_buffer &instruction)
{
	lib::MotorArray desired_motor_pos_new_tmp(MAX_SERVOS_NR);
	lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	lib::MOTION_TYPE motion_type;
	// obliczenia dla ruchu ramienia (kocwk: FRAME)
	/* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
	/* Zlecenie transformerowi przeliczenie wspolrzednych */

	motion_type = instruction.motion_type;
	motion_steps = instruction.motion_steps;
	value_in_step_no = instruction.value_in_step_no;
	lib::Homog_matrix p_m(instruction.arm.pf_def.arm_frame);

	if ((value_in_step_no <= 0) || (motion_steps <= 0) || (value_in_step_no > motion_steps + 1)) {
		throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
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

void manip_effector::set_tool_frame_in_kinematic_model(const lib::Homog_matrix& hm)
{
	if (!(hm.is_valid())) {
		throw NonFatal_error_2(INVALID_HOMOGENEOUS_MATRIX);
	}
	// Ustawienie macierzy reprezentujacej narzedzie.
	((mrrocpp::kinematics::common::kinematic_model_with_tool*) get_current_kinematic_model())->tool = hm;
	/*
	 // odswierzanie
	 get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
	 get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
	 */
}

/*--------------------------------------------------------------------------*/

void manip_effector::master_joints_and_frame_download(void)
{ // by Y
	boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

	// przepisanie danych na zestaw lokalny dla edp_master
	for (int i = 0; i < number_of_servos; i++) {
		current_motor_pos[i] = global_current_motor_pos[i];
		current_joints[i] = global_current_joints[i];
	}
	servo_current_frame_wo_tool = global_current_frame_wo_tool;
}

/*--------------------------------------------------------------------------*/

// Synchronizacja robota.
void manip_effector::synchronise()
{
	manip_and_conv_effector::synchronise();
	get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
}

//   sprawdza stan robota
void manip_effector::get_controller_state(lib::c_buffer &instruction)
{
	manip_and_conv_effector::get_controller_state(instruction);
	if (is_synchronised()) {
		get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
	}
}

void manip_effector::create_threads()
{
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

