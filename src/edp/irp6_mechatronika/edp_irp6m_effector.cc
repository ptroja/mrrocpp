// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6m_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6m_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mathtr/mathtr.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6_mechatronika/edp_irp6m_effector.h"
#include "edp/common/reader.h"
// Kinematyki.
#include "kinematics/irp6_mechatronika/kinematic_model_irp6m_with_wrist.h"
#include "kinematics/irp6_mechatronika/kinematic_model_irp6m_5dof.h"
#include "edp/common/servo_gr.h"
#include "edp/common/manip_trans_t.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

common::servo_buffer* effector::return_created_servo_buffer ()
{
	return new irp6m::servo_buffer (*this);
}



// Konstruktor.
effector::effector (lib::configurator &_config) :
        manip_effector (_config, lib::ROBOT_IRP6_MECHATRONIKA)
{
    //  Stworzenie listy dostepnych kinematyk.
    create_kinematic_models_for_given_robot();

    number_of_servos = IRP6_MECHATRONIKA_NUM_OF_SERVOS;

    reset_variables();
}



/*--------------------------------------------------------------------------*/
void effector::set_rmodel (lib::c_buffer &instruction)
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
            sb->servo_command.parameters.servo_alg_par.servo_algorithm_no[i] = servo_algorithm_ecp[i] = instruction.rmodel.servo_algorithm.servo_algorithm_no[i];
            sb->servo_command.parameters.servo_alg_par.servo_parameters_no[i] = servo_parameters_ecp[i] = instruction.rmodel.servo_algorithm.servo_parameters_no[i];
        }
        /* Wyslanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow procesowi SERVO_GROUP */
        sb->send_to_SERVO_GROUP (); //
        break;

    default: // blad: nie istniejaca specyfikacja modelu robota
        // ustawia numer bledu
        throw NonFatal_error_2(INVALID_SET_RMODEL_TYPE);
    }
}
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void effector::get_rmodel (lib::c_buffer &instruction)
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
    default: // blad: nie istniejaca specyfikacja modelu robota
        // ustawie numer bledu
        throw NonFatal_error_2(INVALID_GET_RMODEL_TYPE);
    }
}



/*--------------------------------------------------------------------------*/
void effector::move_arm (lib::c_buffer &instruction)
{ // przemieszczenie ramienia
    // Wypenienie struktury danych transformera na podstawie parametrow polecenia
    // otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych


    switch (instruction.set_arm_type)
    {
    case lib::MOTOR:
        compute_motors(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();
        break;
    case lib::JOINT:
        compute_joints(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();
        break;
    case lib::FRAME:
        compute_frame(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();
        break;
    default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
        throw NonFatal_error_2(INVALID_SET_END_EFFECTOR_TYPE);
    }

    // by Y - uwaga na wyjatki, po rzuceniu wyjatku nie zostanie zaktualizowany previous_set_arm_type
    previous_set_arm_type = instruction.set_arm_type;

}
/*--------------------------------------------------------------------------*/





// sprawdza stan EDP zaraz po jego uruchomieniu


void effector::servo_joints_and_frame_actualization_and_upload (void)
{
    static int catch_nr=0;

    // wyznaczenie nowych wartosci joints and frame dla obliczen w servo
    try
    {
        get_current_kinematic_model()->mp2i_transform(servo_current_motor_pos, servo_current_joints);

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
		local_matrix.get_mech_xyz_euler_zyz(servo_real_kartez_pos);

    	// scope-locked reader data update
    	{
    		boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			for (int i=0; i<6; i++)
			{
				rb_obj->step_data.real_cartesian_position[i] = servo_real_kartez_pos[i];
			}
    	}

		// Obliczenie polozenia robota we wsp. zewnetrznych bez narzedzia.
		get_current_kinematic_model()->i2e_wo_tool_transform(servo_current_joints, &servo_current_frame_wo_tool);

        catch_nr=0;
    }//: try

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

		// T.K.: Nad tym trzeba pomyslec - co w tym momencie dzieje sie z global_current_end_effector_frame?
		// Jezeli zmienna ta przechowyje polozenie bez narzedzia, to nazwa jest nie tylko nieadekwatna, a wrecz mylaca.
		lib::copy_frame(global_current_frame_wo_tool, servo_current_frame_wo_tool);
    }
}


/*--------------------------------------------------------------------------*/
void effector::get_arm_position (bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

    //   printf(" GET ARM\n");

    if (read_hardware)
    {
        // Uformowanie rozkazu odczytu dla SERVO_GROUP
        sb->servo_command.instruction_code = lib::READ;
        // Wyslanie rozkazu do SERVO_GROUP
        // Pobranie z SERVO_GROUP aktualnej pozycji silnikow
        //		printf("get_arm_position read_hardware\n");

        sb->send_to_SERVO_GROUP ();

        // Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
        for( int i = 0; i < number_of_servos; i++)
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
    switch (instruction.get_arm_type)
    {
    case lib::FRAME:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
        arm_frame_2_frame();
        break;

    case lib::JOINT:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        arm_joints_2_joints();
        break;
    case lib::MOTOR:
        arm_motors_2_motors();
        break;
    default:   // blad: nieznany sposob zapisu wspolrzednych koncowki
        printf("EFF_TYPE: %d\n", instruction.get_arm_type);
        throw NonFatal_error_2(INVALID_GET_END_EFFECTOR_TYPE);
    }

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

		reply.servo_step=rb_obj->step_data.step;
	}
}
/*--------------------------------------------------------------------------*/





// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
    // Stworzenie wszystkich modeli kinematyki.
    add_kinematic_model(new kinematic::irp6m::model_with_wrist());
    add_kinematic_model(new kinematic::irp6m::model_5dof());
    // Ustawienie aktywnego modelu.
    set_kinematic_model(0);
}

} // namespace irp6m
namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new irp6m::effector (_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

