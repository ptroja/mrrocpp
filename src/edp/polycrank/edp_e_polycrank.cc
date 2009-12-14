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
#include "lib/mathtr.h"

// Klasa edp_irp6ot_effector.
#include "edp/polycrank/edp_e_polycrank.h"
#include "edp/common/reader.h"
// Kinematyki.
#include "kinematics/polycrank/kinematic_model_polycrank.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/vis_server.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

common::servo_buffer* effector::return_created_servo_buffer(void)
{
	assert(0);
	return NULL;
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	// przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)
	current_instruction = new_instruction;

	switch (nm_task)
	{
	case common::MT_GET_CONTROLLER_STATE:
		get_controller_state(current_instruction);
		break;
	case common::MT_SET_RMODEL:
		set_rmodel(current_instruction);
		break;
	case common::MT_GET_ARM_POSITION:
		get_arm_position(nm_tryb, current_instruction);
		break;
	case common::MT_GET_ALGORITHMS:
		get_algorithms();
		break;
	case common::MT_SYNCHRONISE:
		synchronise();
		break;
	case common::MT_MOVE_ARM:
		move_arm(current_instruction);
		break;
	default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
	break;
	}
}

// Konstruktor.
effector::effector (lib::configurator &_config) :
        manip_effector (_config, lib::ROBOT_POLYCRANK)
{
    //  Stworzenie listy dostepnych kinematyk.
    create_kinematic_models_for_given_robot();

    number_of_servos = POLYCRANK_NUM_OF_SERVOS;

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
    case lib::TOOL_XYZ_ANGLE_AXIS:
        //printf("TOOL_XYZ_ANGLE_AXIS\n");
        // przeksztaci i przepisa specyfikacj do TRANSFORMATORa
        tool_xyz_aa_2_frame(instruction);
        break;
    case lib::TOOL_XYZ_EULER_ZYZ:
        //printf("TOOL_XYZ_EULER_ZYZ\n");
        // przeksztaci i przepisa specyfikacj do TRANSFORMATORa
        tool_xyz_eul_zyz_2_frame(instruction);
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
        //servo_command.instruction_code = lib::SERVO_ALGORITHM_AND_PARAMETERS;
        for (int i = 0; i<number_of_servos; i++)
        {
        //    servo_command.parameters.servo_alg_par.servo_algorithm_no[i] = servo_algorithm_ecp[i] = instruction.rmodel.servo_algorithm.servo_algorithm_no[i];
        //    servo_command.parameters.servo_alg_par.servo_parameters_no[i] = servo_parameters_ecp[i] = instruction.rmodel.servo_algorithm.servo_parameters_no[i];
        }
        /* Wyslanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow procesowi SERVO_GROUP */
        //send_to_SERVO_GROUP (); //
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
        // przepisa specyfikacj z TRANSFORMATORa do bufora wysykowego
        tool_frame_2_frame_rep();
        break;
    case lib::TOOL_XYZ_ANGLE_AXIS:
        //printf("TOOL_XYZ_ANGLE_AXIS\n");
        // przeksztaci i przepisa specyfikacj z TRANSFORMATORa do bufora wysykowego
        tool_frame_2_xyz_aa();
        break;
    case lib::TOOL_XYZ_EULER_ZYZ:
        //printf("TOOL_XYZ_EULER_ZYZ\n");
        // przeksztaci i przepisa specyfikacj z TRANSFORMATORa do bufora wysykowego
        tool_frame_2_xyz_eul_zyz();
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

/*--------------------------------------------------------------------------*/
void effector::arm_abs_xyz_eul_zyz_2_frame (const double *p)
{

    double x, y, z;					// wspolrzedne wektora przesuniecia
    double alfa, beta, gamma;	// Katy Eulera

    // przepisanie z tablicy pakietu komunikacyjnego
    x = p[0];
    y = p[1];
    z = p[2];

    alfa = p[3];
    beta = p[4];
    gamma = p[5];
    lib::Homog_matrix A_B_T (lib::Homog_matrix::MTR_MECH_XYZ_EULER_ZYZ, x, y, z, alfa, beta, gamma);
    //lib::Homog_matrix A_B_T (lib::XYZ_EULER_ZYZ, x, y, z, alfa, beta, gamma);
    A_B_T.get_frame_tab(desired_end_effector_frame);

}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void effector::arm_frame_2_xyz_eul_zyz ()
{

    // Przeksztacenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztacenia do
    // wewntrznych struktur danych REPLY_BUFFER
    lib::Homog_matrix A(current_end_effector_frame);
    switch (reply.reply_type)
    {
    case lib::ARM:
    case lib::ARM_INPUTS:
    case lib::ARM_RMODEL:
    case lib::ARM_RMODEL_INPUTS:
        A.get_mech_xyz_euler_zyz(reply.arm.pf_def.arm_coordinates);
        A.get_mech_xyz_euler_zyz(rb_obj->step_data.current_cartesian_position);
        //A.get_xyz_euler_zyz(reply.arm.pf_def.arm_coordinates);
        //A.get_xyz_euler_zyz(rb_obj->step_data.current_cartesian_position);
        break;
    default: // blad:
        throw NonFatal_error_2(STRANGE_GET_ARM_REQUEST);
    }

    // dla robotow track i postument - oblicz chwytak
    if ((robot_name == lib::ROBOT_IRP6_ON_TRACK) || (robot_name == lib::ROBOT_IRP6_POSTUMENT))
    {
        reply.arm.pf_def.gripper_reg_state = servo_gripper_reg_state;
        reply.arm.pf_def.gripper_coordinate = current_joints[gripper_servo_nr];
    }

}
/*--------------------------------------------------------------------------*/


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
    case lib::XYZ_EULER_ZYZ:

        // zapisanie wartosci zadanej dla readera
    	// scope-locked reader data update
    	{
    		boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

			for (int i=0; i<6;i++)
			{
				rb_obj->step_data.current_cartesian_position[i]=instruction.arm.pf_def.arm_coordinates[i];
			}
    	}

        compute_xyz_euler_zyz(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();

        break;
    case lib::XYZ_ANGLE_AXIS:
        compute_xyz_angle_axis(instruction);
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



/*--------------------------------------------------------------------------*/
void effector::get_arm_position (bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

    //   printf(" GET ARM\n");

    if (read_hardware)
    {
        // Uformowanie rozkazu odczytu dla SERVO_GROUP
      //  servo_command.instruction_code = lib::READ;
        // Wyslanie rozkazu do SERVO_GROUP
        // Pobranie z SERVO_GROUP aktualnej pozycji silnikow
        //		printf("get_arm_position read_hardware\n");

      //  send_to_SERVO_GROUP ();

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
    case lib::XYZ_ANGLE_AXIS:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
        arm_frame_2_xyz_aa();
        break;
    case lib::XYZ_EULER_ZYZ:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
        arm_frame_2_xyz_eul_zyz();   // dla sterowania pozycyjnego
        reply.arm_type = lib::XYZ_EULER_ZYZ;
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
    add_kinematic_model(new kinematic::polycrank::model());
    // Ustawienie aktywnego modelu.
    set_kinematic_model(0);
}


void effector::servo_joints_and_frame_actualization_and_upload(void)
{

}



/*--------------------------------------------------------------------------*/
void effector::create_threads ()
{
	rb_obj = new common::reader_buffer(*this);
	vis_obj = new common::vis_server(*this);
}

}
// namespace polycrank


namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new polycrank::effector (_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

