// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6s_and_conv.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody wspolne dla robotow IRp-6 oraz tasmociagu
// 				- definicja metod klasy edp_irp6s_and_conv_effector
//
// Autor:		tkornuta
// Data:		14.01.2007
// -------------------------------------------------------------------------

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <signal.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <errno.h>
#include <pthread.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "edp/common/servo_gr.h"
#include "edp/common/reader.h"
#include "edp/common/edp_e_manip_and_conv.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/vis_server.h"

#include "lib/mrmath/mrmath.h"

//#include "kinematics/common/kinematic_model.h"
#include "edp/common/in_out.h"

namespace mrrocpp {
namespace edp {
namespace common {

servo_buffer * manip_and_conv_effector::return_created_servo_buffer()
{
	return NULL;
}
void manip_and_conv_effector::get_arm_position_set_reply_step()
{
	// scope-locked reader data update
	boost::mutex::scoped_lock lock(rb_obj->reader_mutex);
	reply.servo_step = rb_obj->step_data.step;
}

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::get_arm_position_read_hardware_sb()
{ // odczytanie pozycji ramienia

	//   printf(" GET ARM\n");
	lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -

	// Uformowanie rozkazu odczytu dla SERVO_GROUP
	sb->servo_command.instruction_code = lib::READ;
	// Wyslanie rozkazu do SERVO_GROUP
	// Pobranie z SERVO_GROUP aktualnej pozycji silnikow
	//		printf("get_arm_position read_hardware\n");

	sb->send_to_SERVO_GROUP();

	// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
	for (int i = 0; i < number_of_servos; i++) {
		desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
	}

	if (is_synchronised()) {
		//  check_motor_position(desired_motor_pos_new);
		// dla sprawdzenia ograncizen w joints i motors

		get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints_tmp);

		for (int i = 0; i < number_of_servos; i++) {
			desired_joints[i] = current_joints[i] = desired_joints_tmp[i];
		}

	}

}

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::get_arm_position_get_arm_type_switch(lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

	switch (instruction.get_arm_type)
	{
		case lib::JOINT:
			// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
			get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
			// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
			// Przepisanie definicji koncowki danej w postaci
			// JOINTS z wewntrznych struktur danych TRANSFORMATORa
			// do wewntrznych struktur danych REPLY_BUFFER
			reply.arm_type = lib::JOINT;
			for (int i = 0; i < number_of_servos; i++) {
				reply.arm.pf_def.arm_coordinates[i] = current_joints[i];
			}
			break;
		case lib::MOTOR:
			reply.arm_type = lib::MOTOR;
			for (int i = 0; i < number_of_servos; i++) {
				reply.arm.pf_def.arm_coordinates[i] = current_motor_pos[i];
			}
			break;
		default: // blad: nieznany sposob zapisu wspolrzednych koncowki
			printf("EFF_TYPE: %d\n", instruction.get_arm_type);
			throw NonFatal_error_2(INVALID_GET_END_EFFECTOR_TYPE);
	}

}

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::multi_thread_move_arm(lib::c_buffer &instruction)
{ // przemieszczenie ramienia
	// Wypenienie struktury danych transformera na podstawie parametrow polecenia
	// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych


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
		default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
			throw NonFatal_error_2(INVALID_SET_END_EFFECTOR_TYPE);
	}

	// by Y - uwaga na wyjatki, po rzuceniu wyjatku nie zostanie zaktualizowany previous_set_arm_type
	previous_set_arm_type = instruction.set_arm_type;

}
/*--------------------------------------------------------------------------*/

void manip_and_conv_effector::multi_thread_master_order(MT_ORDER nm_task, int nm_tryb)
{
	mt_tt_obj->master_to_trans_t_order(nm_task, nm_tryb);
}

/*--------------------------------------------------------------------------*/
manip_and_conv_effector::manip_and_conv_effector(lib::configurator &_config, lib::robot_name_t l_robot_name) :
	effector(_config, l_robot_name), kinematics_manager(), step_counter(0), number_of_servos(-1),
			current_joints(MAX_SERVOS_NR), desired_joints(MAX_SERVOS_NR), desired_motor_pos_old(MAX_SERVOS_NR),
			desired_motor_pos_new(MAX_SERVOS_NR), current_motor_pos(MAX_SERVOS_NR),
			global_current_motor_pos(MAX_SERVOS_NR), global_current_joints(MAX_SERVOS_NR),
			servo_current_motor_pos(MAX_SERVOS_NR), servo_current_joints(MAX_SERVOS_NR)
{

	controller_state_edp_buf.is_synchronised = false;
	controller_state_edp_buf.is_power_on = true;
	controller_state_edp_buf.is_wardrobe_on = true;
	controller_state_edp_buf.is_robot_blocked = false;

	real_reply_type = lib::ACKNOWLEDGE;

	// is_get_arm_read_hardware=false;
	previous_set_arm_type = lib::MOTOR;

#ifdef DOCENT_SENSOR
	startedCallbackRegistered_ = false;
	stoppedCallbackRegistered_ = false;
#endif

#ifdef __QNXNTO__
	ThreadCtl(_NTO_TCTL_IO, NULL);
#endif

	if (test_mode) {
		msg->message("Test mode activated");
	}
}

manip_and_conv_effector::~manip_and_conv_effector()
{
}

void manip_and_conv_effector::master_joints_read(double* output)
{
	boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

	// przepisanie danych na zestaw lokalny dla edp_master
	for (int i = 0; i < number_of_servos; i++) {
		output[i] = global_current_joints[i];
	}
}

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::hi_create_threads()
{
	rb_obj = new reader_buffer(*this);
	mt_tt_obj = new manip_trans_t(*this);
	in_out_obj = new in_out_buffer();
	vis_obj = new vis_server(*this);
	sb = return_created_servo_buffer();

	// wait for initialization of servo thread
	{
		boost::unique_lock <boost::mutex> lock(thread_started_mutex);

		while (!thread_started) {
			thread_started_cond.wait(thread_started_mutex);
		}
	}
}

// kasuje zmienne - uwaga najpierw nalezy ustawic number_of_servos
void manip_and_conv_effector::reset_variables()
{
	for (int i = 0; i < number_of_servos; i++) {

		desired_joints[i] = 0.0;

		current_joints[i] = 0.0; // ??? wspolrzedne q2 i q3 nie mog by zerowe
		servo_current_joints[i] = 0.0;

		desired_motor_pos_new[i] = 0.0;
		desired_motor_pos_old[i] = 0.0;
		current_motor_pos[i] = 0.0;

		servo_current_motor_pos[i] = 0.0; // Polozenia walow silnikow -// dla watku edp_servo
	}
	// wspolrzedne q2 i q3 mog by zerowe ale powinny byc jak nizej
	// desired_joints[1] = LOWER_LEFT_LIMIT;
	// desired_joints[2] = LOWER_RIGHT_LIMIT;
}

bool manip_and_conv_effector::servo_joints_and_frame_actualization_and_upload(void)
{
	bool ret_val = true;
	static int catch_nr = 0;
	// wyznaczenie nowych wartosci joints and frame dla obliczen w servo
	try {
		{
			boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);
			get_current_kinematic_model()->mp2i_transform(servo_current_motor_pos, servo_current_joints);
		}

		// scope-locked reader data update
		{
			boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			for (int j = 0; j < number_of_servos; j++) {
				rb_obj->step_data.current_joints[j] = servo_current_joints[j];
			}
		}
		catch_nr = 0;
	}//: try
	catch (...) {
		if ((++catch_nr) == 1)
			printf("servo thread servo_joints_and_frame_actualization_and_upload throw catch exception\n");
		ret_val = false;
	}//: catch

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// przepisnie danych na zestaw globalny
		for (int i = 0; i < number_of_servos; i++) {
			global_current_motor_pos[i] = servo_current_motor_pos[i];
			global_current_joints[i] = servo_current_joints[i];
		}//: for

	}

	return ret_val;
}

bool manip_and_conv_effector::is_power_on() const
{
	return controller_state_edp_buf.is_power_on;
}

bool manip_and_conv_effector::pre_synchro_motion(lib::c_buffer &instruction) const
// sprawdzenie czy jest to dopuszczalny rozkaz ruchu
// przed wykonaniem synchronizacji robota
{
	if ((instruction.instruction_type == lib::SET) && (instruction.set_type == ARM_DV) && (instruction.set_arm_type
			== lib::MOTOR) && (instruction.motion_type == lib::RELATIVE))
		return true;
	else
		return false;
}

bool manip_and_conv_effector::is_synchronised(void) const
{
	return controller_state_edp_buf.is_synchronised;
}

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::interpret_instruction(lib::c_buffer &instruction)
{
	//	fprintf(stderr, "instruction:\n");
	//	fprintf(stderr, "\tinstruction_type: %d\n", instruction.instruction_type);
	//	fprintf(stderr, "\tget_type: %d\n", instruction.get_type);
	//	fprintf(stderr, "\tget_rmodel_type: %d\n", instruction.get_rmodel_type);
	//	fprintf(stderr, "\tset_type: %d\n", instruction.set_type);
	//	fprintf(stderr, "\tset_rmodel_type: %d\n", instruction.set_rmodel_type);

	// interpretuje otrzymana z ECP instrukcje;
	// wypelnaia struktury danych TRANSFORMATORa;
	// przygotowuje odpowiedz dla ECP
	// 	printf("interpret instruction poczatek\n");
	// wstepne przygotowanie bufora odpowiedzi
	rep_type(instruction); // okreslenie typu odpowiedzi
	reply.error_no.error0 = OK;
	reply.error_no.error1 = OK;
	// Wykonanie instrukcji
	switch (instruction.instruction_type)
	{
		case lib::SET:
			// tu wykonanie instrukcji SET

			if (instruction.is_set_outputs())
				// ustawienie wyjsc
				set_outputs(instruction);
			if (instruction.is_set_rmodel())
				// zmiana modelu robota
				// set_rmodel();
				master_order(MT_SET_RMODEL, 0);
			if (instruction.is_set_arm()) {
				// przemieszczenie koncowki
				// move_arm();
				master_order(MT_MOVE_ARM, 0);
				instruction.get_arm_type = instruction.set_arm_type;
				get_arm_position(false, instruction); // Aktualizacja transformera
				instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
				//    	printf("interpret instruction, set koniec\n");
			}

			break;
		case lib::GET:
			// tu wykonanie instrukcji GET
			// ustalenie formatu odpowiedzi
			//switch (rep_type(instruction))
			switch (reply.reply_type)
			{
				case lib::CONTROLLER_STATE:
					// odczytanie TCP i orientacji koncowki
					// get_arm_position(true);
					master_order(MT_GET_CONTROLLER_STATE, 0);
					break;
				case lib::ARM:
				case lib::RMODEL:
				case lib::INPUTS:
				case lib::ARM_RMODEL:
				case lib::ARM_INPUTS:
				case lib::RMODEL_INPUTS:
				case lib::ARM_RMODEL_INPUTS:
					if (instruction.is_get_inputs()) {
						get_inputs(&reply);
					}

					if ((instruction.is_get_arm()) || (instruction.is_set_arm())) {
						master_order(MT_GET_ARM_POSITION, true);
					}

					if (instruction.is_get_rmodel()) {
						if (!((instruction.is_get_arm()) || (instruction.is_set_arm()))) {
							if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM) {
								// get_algorithms();
								master_order(MT_GET_ALGORITHMS, 0);
							}
						}
						get_rmodel(instruction);
					}
					break;
				default: // blad
					// ustawi numer bledu
					throw NonFatal_error_2(INVALID_REPLY_TYPE);
			}
			break;
		case lib::SET_GET:
			// tu wykonanie instrukcji SET i GET
			// Cz SET
			if (instruction.is_set_outputs())
				// ustawienie wyj
				set_outputs(instruction);
			if (instruction.is_set_rmodel())
				// zmiana aktualnie uzywanego modelu robota (narzedzie, kinematic_model_with_tool kinematyczny,
				// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
				//        set_rmodel();
				master_order(MT_SET_RMODEL, 0);
			if (instruction.is_set_arm())
				// przemieszczenie koncowki
				// move_arm();
				master_order(MT_MOVE_ARM, 0);
			// Cz GET
			// ustalenie formatu odpowiedzi
			switch (reply.reply_type)
			{
				case lib::CONTROLLER_STATE:
					// odczytanie TCP i orientacji koncowki
					// get_arm_position(true);
					master_order(MT_GET_CONTROLLER_STATE, 0);
					break;
				case lib::ARM:
				case lib::RMODEL:
				case lib::INPUTS:
				case lib::ARM_RMODEL:
				case lib::ARM_INPUTS:
				case lib::RMODEL_INPUTS:
				case lib::ARM_RMODEL_INPUTS:
					if (instruction.is_get_inputs()) {
						get_inputs(&reply);
					}

					if (instruction.is_set_arm()) {
						get_arm_position(false, instruction);
					} else if (instruction.is_get_arm()) {
						master_order(MT_GET_ARM_POSITION, true);
					}

					if (instruction.is_get_rmodel()) {
						if (!instruction.is_set_arm()) {
							// ewentualna aktualizacja numerow algorytmow i ich zestawow parametrow
							if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM)
								master_order(MT_GET_ALGORITHMS, 0);
						}
						// odczytanie aktualnie uzywanego modelu robota (narzedzie, kinematic_model_with_tool kinematyczny,
						// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
						get_rmodel(instruction);
					}

					break;
				default: // blad
					// ustawi numer bledu
					throw NonFatal_error_2(INVALID_REPLY_TYPE);
			}
			break;
		default: // blad
			// ustawi numer bledu
			throw NonFatal_error_2(INVALID_INSTRUCTION_TYPE);
	}

	// printf("interpret instruction koniec\n");

}
/*--------------------------------------------------------------------------*/

// Synchronizacja robota.
void manip_and_conv_effector::synchronise()
{

#ifdef __QNXNTO__
	flushall();
#endif
	/* Uformowanie rozkazu synchronizacji dla procesu SERVO_GROUP */
	sb->servo_command.instruction_code = lib::SYNCHRONISE;
	/* Wyslanie rozkazu synchronizacji do realizacji procesowi SERVO_GROUP */
	sb->send_to_SERVO_GROUP();
	controller_state_edp_buf.is_synchronised = true; // Ustawienie flagi zsynchronizowania robota

	// aktualizacja pozycji robota
	// Uformowanie rozkazu odczytu dla SERVO_GROUP
	sb->servo_command.instruction_code = lib::READ;
	// Wyslanie rozkazu do SERVO_GROUP
	// Pobranie z SERVO_GROUP aktualnej pozycji silnikow
	//	printf("get_arm_position read_hardware\n");

	sb->send_to_SERVO_GROUP();

	// dla pierwszego wypelnienia current_joints i current_end_effector_frame
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

	// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
	for (int i = 0; i < number_of_servos; i++) {
		servo_current_motor_pos[i] = desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
		desired_joints[i] = current_joints[i];
	}
}

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::set_outputs(const lib::c_buffer &instruction)
{
	// ustawienie wyjsc binarnych
	in_out_obj->set_output(&instruction.output_values);
	// throw NonFatal_error_2(NOT_IMPLEMENTED_YET);
	// printf(" OUTPUTS SET\n");
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::get_inputs(lib::r_buffer *local_reply)
{
	// odczytanie wejsc binarnych
	in_out_obj->get_input(&((*local_reply).input_values), ((*local_reply).analog_input));
	// throw NonFatal_error_2(NOT_IMPLEMENTED_YET);
	// printf(" INPUTS GET\n");
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::get_algorithms()
{
	// odczytanie numerow algorytmow i ich numerow zestawow parametrow

	// Uformowanie rozkazu odczytu dla SERVO_GROUP
	sb->servo_command.instruction_code = lib::READ;
	// Wyslanie rozkazu do SERVO_GROUP
	// Pobranie z SERVO_GROUP aktualnej pozycji silnikow i numerow algorytmow etc.
	sb->send_to_SERVO_GROUP();

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
lib::REPLY_TYPE manip_and_conv_effector::rep_type(const lib::c_buffer &instruction)
{
	// ustalenie formatu odpowiedzi
	reply.reply_type = lib::ACKNOWLEDGE;
	if (instruction.is_get_inputs()) {
		reply.reply_type = lib::INPUTS;
	}
	if (instruction.is_get_rmodel()) {
		if (reply.reply_type == lib::ACKNOWLEDGE)
			reply.reply_type = lib::RMODEL;
		else
			reply.reply_type = lib::RMODEL_INPUTS;
	}

	real_reply_type = reply.reply_type;

	if ((instruction.is_get_arm()) || (instruction.is_set_arm())) {
		switch (reply.reply_type)
		{
			case lib::ACKNOWLEDGE:
				reply.reply_type = lib::ARM;
				break;
			case lib::INPUTS:
				reply.reply_type = lib::ARM_INPUTS;
				break;
			case lib::RMODEL:
				reply.reply_type = lib::ARM_RMODEL;
				break;
			case lib::RMODEL_INPUTS:
				reply.reply_type = lib::ARM_RMODEL_INPUTS;
				break;
			default:
				break;
		}
		// is_set_arm jest obslugiwane w szczegolny sposob
		if (instruction.is_get_arm()) {
			real_reply_type = reply.reply_type;
		}
	}

	// by Y
	if (instruction.is_get_controller_state()) {
		reply.reply_type = lib::CONTROLLER_STATE;
	}

	return reply.reply_type;
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::compute_motors(const lib::c_buffer &instruction)
{

	lib::MotorArray desired_motor_pos_new_tmp(MAX_SERVOS_NR);
	lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -


	// obliczenia dla ruchu ramienia (silnikami)
	/* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
	/* Zlecenie transformerowi przeliczenie wspolrzednych */
	const double* p; // wskanik miejsca w strukturze przesanej z ECP, w ktorym znajduj sie wspolrzedne

	const lib::MOTION_TYPE &motion_type = instruction.motion_type;
	motion_steps = instruction.motion_steps;
	value_in_step_no = instruction.value_in_step_no;
	p = &instruction.arm.pf_def.arm_coordinates[0];
	if ((motion_steps <= 0) /* || (value_in_step_no < 0) */)
		throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
	switch (motion_type)
	{
		case lib::ABSOLUTE: // ruch bezwzgledny
			for (int i = 0; i < number_of_servos; i++) {
				desired_motor_pos_new_tmp[i] = p[i];
				//          printf("i: %d, d: %f, p: %f\n",i, desired_motor_pos_new[i], p[i]); // DEBUG
			}
			// sprawdzi przekroczenie dopuszczalnego zakresu
			// check_motor_position(desired_motor_pos_new);

			// dla sprawdzenia ograniczen w joints i motos
			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new_tmp, desired_joints_tmp);
			break;
		case lib::RELATIVE: // ruch wzgledny
			for (int i = 0; i < number_of_servos; i++) {
				desired_motor_pos_new_tmp[i] = p[i] + desired_motor_pos_new[i];
			}
			// Jesli robot zsynchronizowany sprawdzi przekroczenie dopuszczalnego zakresu
			if (is_synchronised()) {
				//  check_motor_position(desired_motor_pos_new);
				// dla sprawdzenia ograncizen w joints i motors

				get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new_tmp, desired_joints_tmp);
			}
			break;
		default:
			throw NonFatal_error_2(INVALID_MOTION_TYPE);
	}

	// kinematyka nie stwierdzila bledow, przepisanie wartosci
	for (int i = 0; i < number_of_servos; i++) {
		desired_joints[i] = desired_joints_tmp[i];
		desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
	}

	// printf("P=%lf\n",desired_motor_pos_new[0]);
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::set_rmodel(lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;
	//printf(" SET RMODEL: ");
	switch (instruction.set_rmodel_type)
	{
		case lib::ARM_KINEMATIC_MODEL:
			//printf("ARM_KINEMATIC_MODEL\n");
			// Ustawienie modelu kinematyki.
			set_kinematic_model(instruction.rmodel.kinematic_model.kinematic_model_no);
			break;
		default: // blad: nie istniejaca specyfikacja modelu robota
			// ustawia numer bledu
			throw NonFatal_error_2(INVALID_SET_RMODEL_TYPE);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::get_rmodel(lib::c_buffer &instruction)
{
	//printf(" GET RMODEL: ");
	switch (instruction.get_rmodel_type)
	{
		case lib::ARM_KINEMATIC_MODEL:
			reply.rmodel_type = lib::ARM_KINEMATIC_MODEL;
			// okreslenie numeru zestawu parametrow przelicznika kinematycznego oraz jego korektora
			reply.rmodel.kinematic_model.kinematic_model_no = get_current_kinematic_model_no();
			break;
		case lib::SERVO_ALGORITHM:
			reply.rmodel_type = lib::SERVO_ALGORITHM;
			// ustawienie numeru algorytmu serworegulatora oraz numeru jego zestawu parametrow

			break;
		default: // blad: nie istniejaca specyfikacja modelu robota
			// ustawie numer bledu
			throw NonFatal_error_2(INVALID_GET_RMODEL_TYPE);
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::compute_joints(const lib::c_buffer &instruction)
{
	lib::MotorArray desired_motor_pos_new_tmp(MAX_SERVOS_NR);
	lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -

	// obliczenia dla ruchu ramienia (stawami)
	/* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
	/* Zlecenie transformerowi przeliczenie wspolrzednych */
	const double* p; // wskanik miejsca w strukturze przeslanej z ECP, w ktorym znajduje sie wspolrzedne

	const lib::MOTION_TYPE &motion_type = instruction.motion_type;
	motion_steps = instruction.motion_steps;
	value_in_step_no = instruction.value_in_step_no;
	p = &instruction.arm.pf_def.arm_coordinates[0];
	if ((value_in_step_no <= 0) || (motion_steps <= 0) || (value_in_step_no > motion_steps + 1))
		throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
	switch (motion_type)
	{
		case lib::ABSOLUTE: // ruch bezwzgledny
			for (int i = 0; i < number_of_servos; i++)
				desired_joints_tmp[i] = p[i];
			break;
		case lib::RELATIVE: // ruch wzgledny
			for (int i = 0; i < number_of_servos; i++)
				desired_joints_tmp[i] = desired_joints[i] + p[i];
			break;
		default:
			throw NonFatal_error_2(INVALID_MOTION_TYPE);
	}
	// check_joints(desired_joints);
	get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);

	// kinematyka nie stwierdzila bledow, przepisanie wartosci
	for (int i = 0; i < number_of_servos; i++) {
		desired_joints[i] = desired_joints_tmp[i];
		desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
	}

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::move_servos()
{
	/* Wyslanie polecenia ruchu do procesu SERVO_GROUP oraz odebranie wyniku
	 realizacji pierwszej fazy ruchu */
	int i;

	/* Uformowanie rozkazu ruchu dla SERVO_GROUP */
	sb->servo_command.instruction_code = lib::MOVE;
	sb->servo_command.parameters.move.number_of_steps = motion_steps;
	sb->servo_command.parameters.move.return_value_in_step_no = value_in_step_no;

	//		printf("edp_irp6s_and_conv_effector::move_servos: %f, %f\n", desired_motor_pos_new[1], desired_motor_pos_old[1]);

	for (i = 0; i < number_of_servos; i++) {
		sb->servo_command.parameters.move.macro_step[i] = desired_motor_pos_new[i] - desired_motor_pos_old[i];
		sb->servo_command.parameters.move.abs_position[i] = desired_motor_pos_new[i]; // by Y
		//    nowa wartosc zadana staje sie stara

		desired_motor_pos_old[i] = desired_motor_pos_new[i];
	}

	/*
	 printf("move_servos_aa: %f, %f, %f, %f, %f, %f, %f, %d\n", sb->servo_command.parameters.move.abs_position[0], sb->servo_command.parameters.move.abs_position[1], sb->servo_command.parameters.move.abs_position[2], sb->servo_command.parameters.move.abs_position[3],
	 sb->servo_command.parameters.move.abs_position[4], sb->servo_command.parameters.move.abs_position[5], sb->servo_command.parameters.move.abs_position[6], sb->servo_command.parameters.move.number_of_steps);
	 */
	/* Wyslanie makrokroku do realizacji procesowi SERVO_GROUP */
	/* Odebranie od procesu SERVO_GROUP informacji o realizacji pierwszej fazy ruchu */
	sb->send_to_SERVO_GROUP();

}
/*--------------------------------------------------------------------------*/

void manip_and_conv_effector::update_servo_current_motor_pos(double motor_position_increment, int i)
{
	servo_current_motor_pos[i] += motor_position_increment;
}

void manip_and_conv_effector::update_servo_current_motor_pos_abs(double abs_motor_position, int i)
{
	servo_current_motor_pos[i] = abs_motor_position;
}

// sprawdza stan EDP zaraz po jego uruchomieniu


void manip_and_conv_effector::get_controller_state(lib::c_buffer &instruction)
{

	//printf("get_controller_state: %d\n", controller_state_edp_buf.is_synchronised); fflush(stdout);
	reply.controller_state = controller_state_edp_buf;

	// aktualizacja pozycji robota
	// Uformowanie rozkazu odczytu dla SERVO_GROUP
	sb->servo_command.instruction_code = lib::READ;
	// Wyslanie rozkazu do SERVO_GROUP
	// Pobranie z SERVO_GROUP aktualnej pozycji silnikow
	//	printf("get_arm_position read_hardware\n");

	sb->send_to_SERVO_GROUP();

	// dla pierwszego wypelnienia current_joints
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for (int i = 0; i < number_of_servos; i++) {
			servo_current_motor_pos[i] = desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
			desired_joints[i] = current_joints[i];
		}
	}
}

void manip_and_conv_effector::main_loop()
{
	// by Y pierwsza petla while do odpytania o stan EDP przez UI zaraz po starcie EDP
	STATE next_state = GET_STATE;
	; // stan nastepny, do ktorego przejdzie EDP_MASTER

	while ((next_state != GET_INSTRUCTION) && (next_state != GET_SYNCHRO)) {

		try { // w tym bloku beda wylapywane wyjatki (bledy)
			switch (next_state)
			{
				case GET_STATE:
					// wstepna interpretacja nadeslanego polecenia w celu wykrycia nieprawidlowosci
					switch (receive_instruction())
					{
						case lib::GET:
							// potwierdzenie przyjecia polecenia (dla ECP)
							//            printf("SET_GET\n");
							insert_reply_type(lib::ACKNOWLEDGE);
							reply_to_instruction();

							if ((rep_type(new_instruction)) == lib::CONTROLLER_STATE) {
								// master_order(MT_GET_CONTROLLER_STATE, 0);
								interpret_instruction(new_instruction);
							} else {
								throw NonFatal_error_1(INVALID_INSTRUCTION_TYPE);
							}

							break;
						case lib::QUERY: // blad: nie ma o co pytac - zadne polecenie uprzednio nie zostalo wydane
							// okreslenie numeru bledu
							throw NonFatal_error_1(QUERY_NOT_EXPECTED);
						default: // blad: nieznana instrukcja
							// okreslenie numeru bledu
							throw NonFatal_error_1(INVALID_INSTRUCTION_TYPE);
					}
					next_state = WAIT;
					break;
				case WAIT:
					if (receive_instruction() == lib::QUERY) { // instrukcja wlasciwa =>
						// zle jej wykonanie, czyli wyslij odpowiedz
						reply_to_instruction();
					} else { // blad: powinna byla nadejsc instrukcja QUERY
						throw NonFatal_error_3(QUERY_EXPECTED);
					}

					/*
					 if (test_mode == 0)
					 {
					 if (!is_power_on()) // jesli wzmacniacz mocy jest wylaczony
					 {
					 exit(EXIT_FAILURE);
					 }
					 }
					 */

					if (!is_synchronised()) // jesli ma zostac przeprowadzona synchronizacja
					{
						next_state = GET_SYNCHRO;
					} else // jesli robot jest juz zsynchronizowany
					{
						next_state = GET_INSTRUCTION;
						msg->message("Robot is initially synchronised");
					}

					break;
				default:
					break;
			}
		}

		catch (NonFatal_error_1 nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			establish_error(nfe.error, OK);
			//  printf("catch NonFatal_error_1\n");
			// informacja dla ECP o bledzie
			reply_to_instruction();
			msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
			// powrot do stanu: GET_INSTRUCTION
			next_state = GET_STATE;
		} // end: catch(transformer::NonFatal_error_1 nfe)

		catch (NonFatal_error_2 nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			//  printf ("catch master thread NonFatal_error_2\n");
			establish_error(nfe.error, OK);
			msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
			// powrot do stanu: WAIT
			next_state = WAIT;
		} // end: catch(transformer::NonFatal_error_2 nfe)

		catch (NonFatal_error_3 nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesowa
			// zapamietanie poprzedniej odpowiedzi
			// Oczekiwano na QUERY a otrzymano co innego, wiec sygnalizacja bledu i
			// dalsze oczekiwanie na QUERY
			lib::REPLY_TYPE rep_type = is_reply_type();
			uint64_t err_no_0 = is_error_no_0();
			uint64_t err_no_1 = is_error_no_1();

			establish_error(nfe.error, OK);
			// informacja dla ECP o bledzie
			reply_to_instruction();
			// przywrocenie poprzedniej odpowiedzi
			insert_reply_type(rep_type);
			establish_error(err_no_0, err_no_1);
			//     printf("ERROR w EDP 3\n");
			msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
			// msg->message(lib::NON_FATAL_ERROR, err_no_0, err_no_1); // by Y - oryginalnie
			// powrot do stanu: GET_INSTRUCTION
			next_state = GET_STATE;
		} // end: catch(transformer::NonFatal_error_3 nfe)

		catch (Fatal_error fe) {
			//     printf("ERROR w EDP transformer fe\n");
			// Obsluga bledow fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu fe
			// Sa to bledy dotyczace sprzetu oraz QNXa (komunikacji)
			establish_error(fe.error0, fe.error1);
			msg->message(lib::FATAL_ERROR, fe.error0, fe.error1);
			// Powrot do stanu: WAIT
			next_state = WAIT;
		} // end: catch(transformer::Fatal_error fe)

	} // end while


	while (next_state != GET_INSTRUCTION) {

		try { // w tym bloku beda wylapywane wyjatki (bledy)
			switch (next_state)
			{
				case GET_SYNCHRO:
					/* Oczekiwanie na zlecenie synchronizacji robota */
					switch (receive_instruction())
					{
						case lib::SYNCHRO:
							// instrukcja wlasciwa => zle jej wykonanie
							/* Potwierdzenie przyjecia instrukcji synchronizacji do wykonania */
							insert_reply_type(lib::ACKNOWLEDGE);
							reply_to_instruction();
							/* Zlecenie wykonania synchronizacji */
							master_order(MT_SYNCHRONISE, 0); // by Y przejscie przez watek transfor w celu ujednolicenia
							// synchronise();
							// Jezeli synchronizacja okae sie niemoliwa, to zostanie zgloszony wyjatek:
							/* Oczekiwanie na poprawne zakoczenie synchronizacji */
							next_state = SYNCHRO_TERMINATED;
							break;
						case lib::SET:
							// instrukcja wlasciwa => zle jej wykonanie
							if (pre_synchro_motion(new_instruction)) {
								/* Potwierdzenie przyjecia instrukcji ruchow presynchronizacyjnych do wykonania */
								insert_reply_type(lib::ACKNOWLEDGE);
								reply_to_instruction();
								/* Zlecenie wykonania ruchow presynchronizacyjnych */
								interpret_instruction(new_instruction);
								// Jezeli wystapil blad w trakcie realizacji ruchow presynchronizacyjnych,
								// to zostanie zgloszony wyjatek:

								/* Oczekiwanie na poprawne zakoczenie synchronizacji */
								next_state = WAIT_Q;
							} else
								// blad: jedyna instrukcja w tym stanie moze by polecenie
								// synchronizacji lub ruchow presynchronizacyjnych
								// Bez synchronizacji adna inna instrukcja nie moze by wykonana przez EDP
								/* Informacja o bedzie polegajcym na braku polecenia synchronizacji */
								throw NonFatal_error_1(NOT_YET_SYNCHRONISED);
							break;
						default: // blad: jedyna instrukcja w tym stanie moze by polecenie
							// synchronizacji lub ruchow presynchronizacyjnych
							// Bez synchronizacji adna inna instrukcja nie moze by wykonana przez EDP
							/* Informacja o bedzie polegajcym na braku polecenia synchronizacji */
							throw NonFatal_error_1(INVALID_INSTRUCTION_TYPE);
					}
					break;
				case SYNCHRO_TERMINATED:
					/* Oczekiwanie na zapytanie od ECP o status zakonczenia synchronizacji (QUERY) */
					if (receive_instruction() == lib::QUERY) { // instrukcja wlasciwa => zle jej wykonanie
						// Budowa adekwatnej odpowiedzi
						insert_reply_type(lib::SYNCHRO_OK);
						reply_to_instruction();
						next_state = GET_INSTRUCTION;
						if (msg->message("Robot is synchronised"))
							printf(" Nie znaleziono SR\n");
					} else { // blad: powinna byla nadejsc instrukcja QUERY
						throw NonFatal_error_4(QUERY_EXPECTED);
					}
					break;
				case WAIT_Q:
					/* Oczekiwanie na zapytanie od ECP o status zakonczenia synchronizacji (QUERY) */
					if (receive_instruction() == lib::QUERY) { // instrukcja wlasciwa => zle jej wykonanie
						// Budowa adekwatnej odpowiedzi
						reply_to_instruction();
						next_state = GET_SYNCHRO;
					} else { // blad: powinna byla nadejsc instrukcja QUERY
						throw NonFatal_error_3(QUERY_EXPECTED);
					}
					break;
				default:
					break;
			}
		}

		// printf("debug edp po while\n");		// by Y&W

		catch (NonFatal_error_1 nfe1) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			establish_error(nfe1.error, OK);
			reply_to_instruction();
			msg->message(lib::NON_FATAL_ERROR, nfe1.error, (uint64_t) 0);
			// powrot do stanu: GET_SYNCHRO
			next_state = GET_SYNCHRO;
		} // end: catch(transformer::NonFatal_error_1 nfe1)

		catch (NonFatal_error_2 nfe2) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			// zapamietanie poprzedniej odpowiedzi
			lib::REPLY_TYPE rep_type = is_reply_type();
			establish_error(nfe2.error, OK);
			reply_to_instruction();
			msg->message(lib::NON_FATAL_ERROR, nfe2.error, (uint64_t) 0);
			// przywrocenie poprzedniej odpowiedzi
			insert_reply_type(rep_type); // powrot do stanu: WAIT_Q
			next_state = WAIT_Q;
		} // end: catch(transformer::NonFatal_error nfe2)

		catch (NonFatal_error_3 nfe3) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			// zapamietanie poprzedniej odpowiedzi
			lib::REPLY_TYPE rep_type = is_reply_type();
			uint64_t err_no_0 = is_error_no_0();
			uint64_t err_no_1 = is_error_no_1();
			establish_error(nfe3.error, OK);
			reply_to_instruction();
			msg->message(lib::NON_FATAL_ERROR, nfe3.error, (uint64_t) 0);
			// przywrocenie poprzedniej odpowiedzi
			insert_reply_type(rep_type);
			establish_error(err_no_0, err_no_1);
			msg->message(lib::NON_FATAL_ERROR, err_no_0, err_no_1);
			// powrot do stanu: GET_SYNCHRO
			next_state = GET_SYNCHRO;
		} // end: catch(transformer::NonFatal_error nfe3)

		catch (NonFatal_error_4 nfe4) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			// zapamietanie poprzedniej odpowiedzi
			lib::REPLY_TYPE rep_type = is_reply_type();
			establish_error(nfe4.error, OK);
			reply_to_instruction();
			msg->message(lib::NON_FATAL_ERROR, nfe4.error, (uint64_t) 0);
			// przywrocenie poprzedniej odpowiedzi
			insert_reply_type(rep_type);
			// powrot do stanu: SYNCHRO_TERMINATED
			next_state = SYNCHRO_TERMINATED;
		} // end: catch(transformer::NonFatal_error nfe4)

		catch (Fatal_error fe) {
			// Obsluga bledow fatalnych
			// Konkretny numer bledu znajduje sie w skadowych error0 lub error1 obiektu fe
			// Sa to bledy dotyczace sprzetu oraz QNXa (komunikacji)
			if (receive_instruction() != lib::QUERY) {
				// blad: powinna byla nadejsc instrukcja QUERY
				establish_error(QUERY_EXPECTED, OK);
				reply_to_instruction();
				printf("QQQ\n");
				receive_instruction();
			}
			insert_reply_type(lib::ERROR);
			establish_error(fe.error0, fe.error1);
			reply_to_instruction();
			msg->message(lib::FATAL_ERROR, fe.error0, fe.error1);
			// powrot do stanu: GET_SYNCHRO
			next_state = GET_SYNCHRO;
		} // catch(transformer::Fatal_error fe)
	}

	/* Nieskoczona petla wykonujca przejscia w grafie automatu (procesu EDP_MASTER) */
	for (;;) {

		try { // w tym bloku beda wylapywane wyjatki (bledy)
			switch (next_state)
			{
				case GET_INSTRUCTION:
					// wstepna interpretacja nadesanego polecenia w celu wykrycia nieprawidlowosci
					switch (receive_instruction())
					{
						case lib::SET:
						case lib::GET:
						case lib::SET_GET:
							// potwierdzenie przyjecia polecenia (dla ECP)
							// printf("SET_GET\n");
							insert_reply_type(lib::ACKNOWLEDGE);
							reply_to_instruction();
							break;
						case lib::SYNCHRO: // blad: robot jest juz zsynchronizowany
							// okreslenie numeru bledu
							throw NonFatal_error_1(ALREADY_SYNCHRONISED);
						case lib::QUERY: // blad: nie ma o co pytac - zadne polecenie uprzednio nie zostalo wydane
							// okreslenie numeru bledu
							throw NonFatal_error_1(QUERY_NOT_EXPECTED);
						default: // blad: nieznana instrukcja
							// okreslenie numeru bledu
							throw NonFatal_error_1(UNKNOWN_INSTRUCTION);
					}
					next_state = EXECUTE_INSTRUCTION;
					break;
				case EXECUTE_INSTRUCTION:
					// wykonanie instrukcji - wszelkie bledy powoduja zgloszenie wyjtku NonFatal_error_2 lub Fatal_error
					interpret_instruction(new_instruction);
					next_state = WAIT;
					break;
				case WAIT:
					if (receive_instruction() == lib::QUERY) { // instrukcja wlasciwa =>
						// zlec jej wykonanie, czyli wyslij odpowiedz
						reply_to_instruction();
					} else { // blad: powinna byla nadejsc instrukcja QUERY
						throw NonFatal_error_3(QUERY_EXPECTED);
					}
					next_state = GET_INSTRUCTION;
					break;
				default:
					break;
			}
		}

		catch (NonFatal_error_1 nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			establish_error(nfe.error, OK);
			// printf("catch NonFatal_error_1\n");
			// informacja dla ECP o bledzie
			reply_to_instruction();
			msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
			// powrot do stanu: GET_INSTRUCTION
			next_state = GET_INSTRUCTION;
		} // end: catch(transformer::NonFatal_error_1 nfe)

		catch (NonFatal_error_2 nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
			// printf ("catch master thread NonFatal_error_2\n");
			establish_error(nfe.error, OK);
			msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
			// powrot do stanu: WAIT
			next_state = WAIT;
		} // end: catch(transformer::NonFatal_error_2 nfe)

		catch (NonFatal_error_3 nfe) {
			// Obsluga bledow nie fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
			// Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesowa
			// zapamietanie poprzedniej odpowiedzi
			// Oczekiwano na QUERY a otrzymano co innego, wiec sygnalizacja bledu i
			// dalsze oczekiwanie na QUERY
			lib::REPLY_TYPE rep_type = is_reply_type();
			uint64_t err_no_0 = is_error_no_0();
			uint64_t err_no_1 = is_error_no_1();

			establish_error(nfe.error, OK);
			// informacja dla ECP o bledzie
			reply_to_instruction();
			// przywrocenie poprzedniej odpowiedzi
			insert_reply_type(rep_type);
			establish_error(err_no_0, err_no_1);
			// printf("ERROR w EDP 3\n");
			msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
			// msg->message(lib::NON_FATAL_ERROR, err_no_0, err_no_1); // by Y - oryginalnie
			// powrot do stanu: GET_INSTRUCTION
			next_state = GET_INSTRUCTION;
		} // end: catch(transformer::NonFatal_error_3 nfe)

		catch (Fatal_error fe) {
			// Obsluga bledow fatalnych
			// Konkretny numer bledu znajduje sie w skladowej error obiektu fe
			// Sa to bledy dotyczace sprzetu oraz QNXa (komunikacji)
			establish_error(fe.error0, fe.error1);
			msg->message(lib::FATAL_ERROR, fe.error0, fe.error1);
			// Powrot do stanu: WAIT
			next_state = WAIT;
		} // end: catch(transformer::Fatal_error fe)

	} // end: for (;;)

}

} // namespace common
} // namespace edp
} // namespace mrrocpp
