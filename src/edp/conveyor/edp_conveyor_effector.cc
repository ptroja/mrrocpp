// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6ot_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robota IRp-6 na torze jezdnym
//				- definicja metod klasy edp_irp6ot_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mathtr.h"

// Klasa edp_conveyor_effector.
#include "edp/conveyor/edp_conveyor_effector.h"
// Model kinematyczny tasmociagu.
#include "kinematics/conveyor/kinematic_model_conveyor.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

// Konstruktor.
effector::effector (lib::configurator &_config) :
	irp6s_and_conv_effector (_config, ROBOT_CONVEYOR)
	{
	}


void effector::initialize (void)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	number_of_servos = CONVEYOR_NUM_OF_SERVOS;

	reset_variables();
}


/*--------------------------------------------------------------------------*/
void effector::set_rmodel (c_buffer &instruction)
{
	// BYTE previous_model;
	// BYTE previous_corrector;
	//printf(" SET RMODEL: ");
	switch (instruction.set_rmodel_type)
	{
	case ARM_KINEMATIC_MODEL:
		//printf("ARM_KINEMATIC_MODEL\n");
		// Ustawienie modelu kinematyki.
		set_kinematic_model(instruction.rmodel.kinematic_model.kinematic_model_no);
		break;

	case SERVO_ALGORITHM:
		// ustawienie algorytmw serworegulacji oraz ich parametrow
		// zmiana algorytmu regulacji
		/* Uformowanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow dla procesu SERVO_GROUP */
		servo_command.instruction_code = SERVO_ALGORITHM_AND_PARAMETERS;
		for (int i = 0; i<number_of_servos; i++)
		{
			servo_command.parameters.servo_alg_par.servo_algorithm_no[i] = servo_algorithm_ecp[i] = instruction.rmodel.servo_algorithm.servo_algorithm_no[i];
			servo_command.parameters.servo_alg_par.servo_parameters_no[i] = servo_parameters_ecp[i] = instruction.rmodel.servo_algorithm.servo_parameters_no[i];
		}
		/* Wyslanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow procesowi SERVO_GROUP */
		send_to_SERVO_GROUP (); //
		break;

	default: // blad: nie istniejca specyfikacja modelu robota
		// ustawi numer bledu
		throw NonFatal_error_2(INVALID_SET_RMODEL_TYPE);
	}
}
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void effector::get_rmodel (c_buffer &instruction)
{
	int i; // licznik obiegow petli
	//printf(" GET RMODEL: ");
	switch (instruction.get_rmodel_type)
	{
	case ARM_KINEMATIC_MODEL:
		reply.rmodel_type = ARM_KINEMATIC_MODEL;
		// okreslenie numeru zestawu parametrow przelicznika kinematycznego oraz jego korektora
		reply.rmodel.kinematic_model.kinematic_model_no = get_current_kinematic_model_no();
		break;
	case SERVO_ALGORITHM:
		reply.rmodel_type = SERVO_ALGORITHM;
		// ustawienie numeru algorytmu serworegulatora oraz numeru jego zestawu parametrow
		for (i = 0; i<number_of_servos; i++)
			if ( instruction.is_get_arm() )
			{
				reply.rmodel.servo_algorithm.servo_algorithm_no[i] = servo_algorithm_sg[i];
				reply.rmodel.servo_algorithm.servo_parameters_no[i] = servo_parameters_sg[i];
			}
			else
			{
				reply.rmodel.servo_algorithm.servo_algorithm_no[i] = servo_algorithm_sg[i];
				reply.rmodel.servo_algorithm.servo_parameters_no[i] = servo_parameters_sg[i];
			}
		break;
	default: // blad: nie istniejaca specyfikacja modelu robota
		// ustawie numer bledu
		throw NonFatal_error_2(INVALID_GET_RMODEL_TYPE);
	}
}
/*--------------------------------------------------------------------------*/



// servo_joints_and_frame_actualization_and_upload.
void effector::servo_joints_and_frame_actualization_and_upload (void)
{
	int i;
	static int catch_nr=0;
	// wyznaczenie nowych wartosci joints and frame dla obliczen w servo
	try
	{
		get_current_kinematic_model()->mp2i_transform(servo_current_motor_pos, servo_current_joints);
		catch_nr=0;
	}//: try
	catch (...)
	{
		if ((++catch_nr) == 1)
			printf("servo thread servo_joints_and_frame_actualization_and_upload throw catch exception\n");
	}//: catch

	pthread_mutex_lock( &edp_irp6s_effector_mutex );
	// przepisnie danych na zestaw globalny
	for (i=0; i < number_of_servos; i++)
	{
		global_current_motor_pos[i]=servo_current_motor_pos[i];
		global_current_joints[i]=servo_current_joints[i];
	}//: for


	pthread_mutex_unlock( &edp_irp6s_effector_mutex );
}//: servo_joints_and_frame_actualization_and_upload



// Przemieszczenie tasmociagu.
void effector::move_arm (c_buffer &instruction)
{
	// Wypenienie struktury danych transformera na podstawie parametrow polecenia
	// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych

	switch (instruction.set_arm_type)
	{
	case MOTOR:
		compute_motors(instruction);
		break;
	case JOINT:
		compute_joints(instruction);
		break;
	default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
		throw NonFatal_error_2(INVALID_SET_END_EFFECTOR_TYPE);
	}

	// wykonanie ruchu
	switch (instruction.set_arm_type)
	{
	case MOTOR:
	case JOINT:
		// Wyslanie makrokroku do realizacji SERVO_GROUP oraz
		// odebranie informacji o realizacji pierwszej fazy ruchu
		// aktualizacja transformera
		move_servos ();

		mt_tt_obj->trans_t_to_master_order_status_ready();
		break;
	default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
		throw NonFatal_error_2(INVALID_SET_END_EFFECTOR_TYPE);
	}

	// by Y - uwaga na wyjatki, po rzuceniu wyjatku nie zostanie zaktualizowany previous_set_arm_type
	previous_set_arm_type = instruction.set_arm_type;

}

// Odczytanie pozycji tasmociagu.
void effector::get_arm_position (bool read_hardware, c_buffer &instruction)
{

	if (read_hardware)
	{
		// Uformowanie rozkazu odczytu dla SERVO_GROUP
		servo_command.instruction_code = READ;
		// Wyslanie rozkazu do SERVO_GROUP
		// Pobranie z SERVO_GROUP aktualnej pozycji silnikow
		//		printf("get_arm_position read_hardware\n");

		send_to_SERVO_GROUP ();


		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for( int i = 0; i < number_of_servos; i++)
		{
			desired_motor_pos_old[i] = current_motor_pos[i];
		}
		if (synchronised)
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
	; // end: if

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	switch (instruction.get_arm_type)
	{
	case   JOINT:
		// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
		arm_joints_2_joints();
		break;
	case   MOTOR:
		arm_motors_2_motors();
		break;
	default:   // blad: nieznany sposob zapisu wspolrzednych koncowki
		printf("EFF_TYPE: %d\n", instruction.get_arm_type);
		throw NonFatal_error_2(INVALID_GET_END_EFFECTOR_TYPE);
	}

	rb_obj->lock_mutex();// by Y
	reply.servo_step=rb_obj->step_data.step;
	rb_obj->unlock_mutex();

}
; //: get_arm_position



// Stworzenie modeli kinematyki dla tasmociagu.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematic::conveyor::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}//: create_kinematic_models_for_given_robot

} // namespace conveyor

namespace common {


// Stworzenie obiektu edp_conveyor_effector.
effector* return_created_efector (lib::configurator &_config)
{
	return new conveyor::effector (_config);
}//: return_created_efector

} // namespace common
} // namespace edp
} // namespace mrrocpp
