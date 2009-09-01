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
#include <semaphore.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif
#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "edp/common/edp_e_manip_and_conv.h"

#include "lib/mathtr.h"

#include "kinematics/common/kinematic_model.h"

namespace mrrocpp {
namespace edp {
namespace common {


/*--------------------------------------------------------------------------*/
manip_and_conv_effector::manip_and_conv_effector (lib::configurator &_config, lib::ROBOT_ENUM l_robot_name) :
        effector (_config, l_robot_name), manager(), step_counter(0)
{
	pthread_mutex_init(&edp_irp6s_effector_mutex, NULL);

    controller_state_edp_buf.is_synchronised = false;
    controller_state_edp_buf.is_power_on = true;
    controller_state_edp_buf.is_wardrobe_on = true;
    controller_state_edp_buf.is_controller_card_present = true;
    controller_state_edp_buf.is_robot_blocked = false;

    real_reply_type = lib::ACKNOWLEDGE;
    // inicjacja deskryptora pliku by 7&Y
    // servo_fd = name_open(lib::EDP_ATTACH_POINT, 0);

    trans_t_command = false;

    // is_get_arm_read_hardware=false;
    previous_set_arm_type = lib::MOTOR;

    motion_type = lib::ABSOLUTE;
    synchronised = false;

#ifdef __QNXNTO__
    if((servo_to_tt_chid = ChannelCreate(_NTO_CHF_UNBLOCK)) == -1) {
    	perror("ChannelCreate()");
    }
    if ((servo_fd = ConnectAttach(0, 0, servo_to_tt_chid, 0, _NTO_COF_CLOEXEC )) == -1)
    {
        perror("ConnectAttach()");
    }
    ThreadCtl (_NTO_TCTL_IO, NULL);
#else
    sem_init(&servo_command_ready, 0, 0);
    sem_init(&sg_reply_ready, 0, 0);
#endif

    // z edp_m

    if (test_mode)
    {
        msg->message("Test mode activated");
    }
}

manip_and_conv_effector::~manip_and_conv_effector() {
	// TODO: error checks
	pthread_mutex_destroy(&edp_irp6s_effector_mutex);
#ifdef __QNXNTO__
	ConnectDetach_r(servo_fd);
	ChannelDestroy_r(servo_to_tt_chid);
#else
	sem_destroy(&servo_command_ready);
	sem_destroy(&sg_reply_ready);
#endif
}

void manip_and_conv_effector::master_joints_read (double* output)
{ // by Y
    pthread_mutex_lock( &edp_irp6s_effector_mutex );
    // przepisanie danych na zestaw lokalny dla edp_master
    for (int i=0; i < number_of_servos; i++)
    {
        output[i]=global_current_joints[i];
    }
    pthread_mutex_unlock( &edp_irp6s_effector_mutex );
}

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::create_threads ()
{
    // Y&W - utworzenie watku serwa
    if (pthread_create (&serwo_tid, NULL, &servo_thread_start, (void *) this))
    {
        msg->message(lib::SYSTEM_ERROR, errno, "EDP: Failed to create SERVO_GROUP thread");
        throw System_error();
    }

    // Y&W - utworzenie watku readera
    if (pthread_create (&reader_tid, NULL, &reader_thread_start, (void *) this))
    {
        msg->message(lib::SYSTEM_ERROR, errno, "EDP: Failed to create READER thread");
        throw System_error();
    }

    if (pthread_create (&trans_t_tid, NULL, &trans_thread_start, (void *) this))
    {
        msg->message(lib::SYSTEM_ERROR, errno, "EDP: Failed to create TRANSFORMER thread");
        throw System_error();
    }

    // PT - utworzenie watku wizualizacji
    if (pthread_create (&vis_t_tid, NULL, &visualisation_thread_start, (void *) this))
    {
        msg->message(lib::SYSTEM_ERROR, errno, "EDP: Failed to create VISUALISATION thread");
        throw System_error();
    }
}


// kasuje zmienne - uwaga najpierw nalezy ustawic number_of_servos
void manip_and_conv_effector::reset_variables ()
{
    int i; // Liczniki petli
    // Serwomechanizmy
    for(i=0; i<number_of_servos; i++)
    {
        servo_algorithm_ecp[i] = 0;
        servo_parameters_ecp[i] = 0;
        servo_algorithm_sg[i] = 0;
        servo_parameters_sg[i] = 0;
    }

    for (i=0; i<number_of_servos; i++)
    {
        desired_joints[i] = 0.0;
        previous_joints[i]= 0.0;
    }
    // wspolrzedne q2 i q3 mog by zerowe ale powinny byc jak nizej
    // desired_joints[1] = LOWER_LEFT_LIMIT;
    // desired_joints[2] = LOWER_RIGHT_LIMIT;

    for (i=0; i<number_of_servos; i++)
    {
        current_joints[i] = 0.0; // ??? wspolrzedne q2 i q3 nie mog by zerowe
        servo_current_joints[i]=0.0;
    }

    for (i=0; i<number_of_servos; i++)
    {
        desired_motor_pos_new[i] = 0.0;
        desired_motor_pos_old[i] = 0.0;
        current_motor_pos[i] = 0.0;
        motor_pos_increment_reading[i] = 0.0;
        servo_current_motor_pos[i]=0.0;   // Polozenia walow silnikow -// dla watku edp_servo
    }
}

void manip_and_conv_effector::servo_joints_and_frame_actualization_and_upload(void)
{}

bool manip_and_conv_effector::is_power_on() const
{
    return controller_state_edp_buf.is_power_on;
}

bool manip_and_conv_effector::pre_synchro_motion(lib::c_buffer &instruction) const
// sprawdzenie czy jest to dopuszczalny rozkaz ruchu
// przed wykonaniem synchronizacji robota
{
    if (
        (instruction.instruction_type == lib::SET) &&
        (instruction.set_type == ARM_DV) &&
        (instruction.set_arm_type == lib::MOTOR) &&
        (instruction.motion_type == lib::RELATIVE)
    )
        return true;
    else
        return false;
}



bool manip_and_conv_effector::is_synchronised ( void ) const
{
    return synchronised;
}

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::interpret_instruction (lib::c_buffer &instruction)
{
    // interpretuje otrzyman z ECP instrukcj;
    // wypenaia struktury danych TRANSFORMATORa;
    // przygotowuje odpowied dla ECP
    // 	printf("interpret instruction poczatek\n");
    // wstpne przygotowanie bufora odpowiedzi
    rep_type(instruction); // okreslenie typu odpowiedzi
    reply.error_no.error0 = OK;
    reply.error_no.error1 = OK;
    // Wykonanie instrukcji
    switch ( instruction.instruction_type )
    {
    case lib::SET:
        // tu wykonanie instrukcji SET

        if (instruction.is_set_outputs())
            // ustawienie wyjsc
            set_outputs(instruction);
        if (instruction.is_set_rmodel())
            // zmiana modelu robota
            // set_rmodel();
            mt_tt_obj.master_to_trans_t_order(MT_SET_RMODEL, 0);
        if (instruction.is_set_arm())
        {
            // przemieszczenie koncowki
            // move_arm();
            mt_tt_obj.master_to_trans_t_order(MT_MOVE_ARM, 0);
            instruction.get_arm_type = instruction.set_arm_type;
            get_arm_position(false, instruction); // Aktualizacja transformera
            instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
            //    	printf("interpret instruction, set koniec\n");
        }

        break;
    case lib::GET:
        // tu wykonanie instrukcji GET
        // ustalenie formatu odpowiedzi
        switch (rep_type(instruction))
        {
        case lib::CONTROLLER_STATE:
            // odczytanie TCP i orientacji koncowki
            // get_arm_position(true);
            mt_tt_obj.master_to_trans_t_order(MT_GET_CONTROLLER_STATE, 0);
            break;
        case lib::ARM:
            // odczytanie TCP i orientacji koncowki
            // get_arm_position(true);
            mt_tt_obj.master_to_trans_t_order(MT_GET_ARM_POSITION, true);
            break;
        case lib::RMODEL:
            // ewentualna aktualizacja numerow algorytmow i ich zestawow parametrow
            if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM)
            {
                // get_algorithms();
                mt_tt_obj.master_to_trans_t_order(MT_GET_ALGORITHMS, 0);
            }
            // odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            get_rmodel (instruction);
            break;
        case lib::INPUTS:
            // odczytanie wejsc
            get_inputs(&reply);
            break;
        case lib::ARM_RMODEL:
            // odczytanie TCP i orientacji koncowki
            // get_arm_position(true);
            mt_tt_obj.master_to_trans_t_order(MT_GET_ARM_POSITION, true);
            // odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            get_rmodel(instruction);
            break;
        case lib::ARM_INPUTS:
            // odczytanie wej
            get_inputs(&reply);
            // odczytanie TCP i orientacji koncowki
            // get_arm_position(true);
            mt_tt_obj.master_to_trans_t_order(MT_GET_ARM_POSITION, true);
            break;
        case lib::RMODEL_INPUTS:
            // ewentualna aktualizacja numerow algorytmow i ich zestawow parametrow
            if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM)
                // get_algorithms();
                mt_tt_obj.master_to_trans_t_order(MT_GET_ALGORITHMS, 0);
            // odczytanie wej
            get_inputs(&reply);
            // odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            get_rmodel(instruction);
            break;
        case lib::ARM_RMODEL_INPUTS:
            // odczytanie wejsc
            get_inputs(&reply);
            // odczytanie TCP i orientacji koncowki
            // get_arm_position(true);
            mt_tt_obj.master_to_trans_t_order(MT_GET_ARM_POSITION, true);
            // odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            get_rmodel(instruction);
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
            // zmiana aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            //        set_rmodel();
            mt_tt_obj.master_to_trans_t_order(MT_SET_RMODEL, 0);
        if (instruction.is_set_arm())
            // przemieszczenie koncowki
            // move_arm();
            mt_tt_obj.master_to_trans_t_order(MT_MOVE_ARM, 0);
        // Cz GET
        // ustalenie formatu odpowiedzi
        switch (rep_type(instruction))
        {
        case lib::CONTROLLER_STATE:
            // odczytanie TCP i orientacji koncowki
            // get_arm_position(true);
            mt_tt_obj.master_to_trans_t_order(MT_GET_CONTROLLER_STATE, 0);
            break;
        case lib::ARM:
            // odczytanie TCP i orientacji koncowki
            if(instruction.is_set_arm())
                get_arm_position(false, instruction);
            else
                // get_arm_position(true);
                mt_tt_obj.master_to_trans_t_order(MT_GET_ARM_POSITION, true);
            break;
        case lib::RMODEL:
            if(!instruction.is_set_arm())
                // ewentualna aktualizacja numerow algorytmow i ich zestawow parametrow
                if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM)
                    // get_algorithms();
                    mt_tt_obj.master_to_trans_t_order(MT_GET_ALGORITHMS, 0);
            // odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            get_rmodel(instruction);
            break;
        case lib::INPUTS:
            // odczytanie wej
            get_inputs(&reply);
            break;
        case lib::ARM_RMODEL:
            // odczytanie TCP i orientacji koncowki
            if(instruction.is_set_arm())
                get_arm_position(false, instruction);
            else
                // get_arm_position(true);
                mt_tt_obj.master_to_trans_t_order(MT_GET_ARM_POSITION, true);
            // odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            get_rmodel(instruction);
            break;
        case lib::ARM_INPUTS:
            // odczytanie wejsc
            get_inputs(&reply);
            // odczytanie TCP i orientacji koncowki
            if(instruction.is_set_arm())
                get_arm_position(false, instruction);
            else
                // get_arm_position(true);
                mt_tt_obj.master_to_trans_t_order(MT_GET_ARM_POSITION, true);
            break;
        case lib::RMODEL_INPUTS:
            if(!instruction.is_set_arm())
                // ewentualna aktualizacja numerow algorytmow i ich zestawow parametrow
                if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM)
                    //   get_algorithms();
                    mt_tt_obj.master_to_trans_t_order(MT_GET_ALGORITHMS, 0);
            // odczytanie wej
            get_inputs(&reply);
            // odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            get_rmodel(instruction);
            break;
        case lib::ARM_RMODEL_INPUTS:
            // odczytanie wejsc
            get_inputs(&reply);
            if(instruction.is_set_arm())
                get_arm_position(false, instruction);
            else
                // get_arm_position(true);
                mt_tt_obj.master_to_trans_t_order(MT_GET_ARM_POSITION, true);
            // odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
            // jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
            get_rmodel(instruction);
            break;
            // odczytanie TCP i orientacji koncowki
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
void manip_and_conv_effector::synchronise ()
{
	common_synchronise();
}


// Synchronizacja robota.
void manip_and_conv_effector::common_synchronise ()
{
#ifdef __QNXNTO__
	flushall();
#endif
    /* Uformowanie rozkazu synchronizacji dla procesu SERVO_GROUP */
    servo_command.instruction_code = lib::SYNCHRONISE;
    /* Wyslanie rozkazu synchronizacji do realizacji procesowi SERVO_GROUP */
    send_to_SERVO_GROUP ();
    controller_state_edp_buf.is_synchronised = synchronised = true; // Ustawienie flagi zsynchronizowania robota

    // aktualizacja pozycji robota
    // Uformowanie rozkazu odczytu dla SERVO_GROUP
    servo_command.instruction_code = lib::READ;
    // Wyslanie rozkazu do SERVO_GROUP
    // Pobranie z SERVO_GROUP aktualnej pozycji silnikow
    //	printf("get_arm_position read_hardware\n");

    send_to_SERVO_GROUP ();

    // dla pierwszego wypelnienia current_joints i current_end_effector_frame
    get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

    // Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
    for( int i = 0; i < number_of_servos; i++)
    {
        servo_current_motor_pos[i] = desired_motor_pos_new_tmp[i] = desired_motor_pos_new[i] =
                                         desired_motor_pos_old[i] = current_motor_pos[i];
        desired_joints_tmp[i] = desired_joints[i] = current_joints[i];
    }
}




/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::arm_motors_2_motors (void)
{
    // Przepisanie definicji koncowki danej w postaci
    // MOTORS z wewnetrznych struktur danych TRANSFORMATORa
    // do wewntrznych struktur danych REPLY_BUFFER
    reply.arm_type = lib::MOTOR;
    switch (reply.reply_type)
    {
    case lib::ARM:
    case lib::ARM_INPUTS:
    case lib::ARM_RMODEL:
    case lib::ARM_RMODEL_INPUTS:
        for (int i=0; i<number_of_servos; i++)
        {
            reply.PWM_value[i] = PWM_value[i];
            reply.current[i] = current[i];
            reply.arm.pf_def.arm_coordinates[i] = current_motor_pos[i];
        }
        break;
    default: // blad:
        throw NonFatal_error_2(STRANGE_GET_ARM_REQUEST);
    }

    if ((robot_name == lib::ROBOT_IRP6_ON_TRACK) || (robot_name == lib::ROBOT_IRP6_POSTUMENT))
    {
        reply.arm.pf_def.gripper_reg_state = servo_gripper_reg_state;
        reply.arm.pf_def.gripper_coordinate = current_joints[gripper_servo_nr];
    }

}
/*--------------------------------------------------------------------------*/




/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::arm_joints_2_joints (void)
{
    // Przepisanie definicji koncowki danej w postaci
    // JOINTS z wewntrznych struktur danych TRANSFORMATORa
    // do wewntrznych struktur danych REPLY_BUFFER
    reply.arm_type = lib::JOINT;
    switch (reply.reply_type)
    {
    case lib::ARM:
    case lib::ARM_INPUTS:
    case lib::ARM_RMODEL:
    case lib::ARM_RMODEL_INPUTS:
        for (int i=0; i<number_of_servos; i++)
            reply.arm.pf_def.arm_coordinates[i] = current_joints[i];
        break;
    default: // blad:
            throw NonFatal_error_2(STRANGE_GET_ARM_REQUEST);
    }

    if ((robot_name == lib::ROBOT_IRP6_ON_TRACK) || (robot_name == lib::ROBOT_IRP6_POSTUMENT))
    {
        reply.arm.pf_def.gripper_reg_state = servo_gripper_reg_state;
        reply.arm.pf_def.gripper_coordinate = current_joints[gripper_servo_nr];
    }

}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::send_to_SERVO_GROUP ()
{
    // int command_size; // dulgosci rozkazu
    // sigset_t set, old_set; // zmienne opisujace sygnaly przysylane do procesu
    // Wyslanie polecenia do SERVO_GROUP

    /*
    // by Y - wywalone
    // Obliczenie dugoci rozkazu
    switch(servo_command.instruction_code) {
      case lib::READ:
      case lib::SYNCHRONISE:
         command_size = (int) (((lib::BYTE*) (&servo_command.address_byte)) - ((lib::BYTE*) (&servo_command.instruction_code)));
         break;
      case lib::MOVE:
         command_size = (int) (((lib::BYTE*) (&servo_command.parameters.move.address_byte)) - ((lib::BYTE*) (&servo_command.instruction_code)));
         break;
      case lib::SERVO_ALGORITHM_AND_PARAMETERS:
         command_size = (int) (((lib::BYTE*) (&servo_command.parameters.servo_alg_par.address_byte)) - ((lib::BYTE*) (&servo_command.instruction_code)));
         break;
}; // end: switch
    // if (Send(&servo_command, &sg_reply, command_size, sizeof(lib::servo_group_reply)) < 0) {
    */

    /* sigemptyset ( &set);
     sigaddset ( &set, SIGUSR1);

     SignalProcmask( 0,serwo_tid, SIG_BLOCK, &set, NULL ); // by Y uniemozliwienie jednoczesnego wystawiania spotkania do serwo przez edp_m i readera
     */
#ifdef __QNXNTO__
    if (MsgSend(servo_fd, &servo_command, sizeof(servo_command), &sg_reply, sizeof(sg_reply)) < 0)
    {
        uint64_t e = errno;
        perror ("Send() from EDP to SERVO error");
        msg->message(lib::SYSTEM_ERROR, e, "Send() from EDP to SERVO error");
        throw System_error();
    }
#else
    sem_post(&servo_command_ready);
    sem_wait(&sg_reply_ready);
#endif

    //   SignalProcmask( 0,serwo_tid, SIG_UNBLOCK, &set, NULL );

    if ( (sg_reply.error.error0 != OK) || (sg_reply.error.error1 != OK) )
    {
        printf("a: %llx, :%llx\n",sg_reply.error.error0,sg_reply.error.error1);
        throw Fatal_error(sg_reply.error.error0, sg_reply.error.error1);
    } // end: if((sg_reply.error.error0 != OK) || (sg_reply.error.error1 != OK))

    // skopiowanie odczytow do transformera

    for (int i=0; i<number_of_servos; i++)
    {
        /*
        if (i==6)
    {
             motor_pos_increment_reading[i] = sg_reply.position[i] * 2*M_PI / IRP6_POSTUMENT_AXIS_7_INC_PER_REVOLUTION;
    } else if (i==5)
    {
             motor_pos_increment_reading[i] = sg_reply.position[i] * 2*M_PI / IRP6_POSTUMENT_AXIS_6_INC_PER_REVOLUTION;
    } else
    {
             motor_pos_increment_reading[i] = sg_reply.position[i] * 2*M_PI / IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION;
    }
             // Aktualnie odczytane polozenia walow silnikow (w radianach)
        current_motor_pos[i] +=   motor_pos_increment_reading[i];
        */

        current_motor_pos[i] = sg_reply.abs_position[i];

        //	 printf("current motor pos: %d\n", current_motor_pos[0]);

        if(test_mode)
        {
            // W.S. Tylko przy testowaniu
            current_motor_pos[i] = desired_motor_pos_new[i];
        }

        PWM_value[i] = sg_reply.PWM_value[i];
        current[i] = sg_reply.current[i];
        servo_algorithm_sg[i] = sg_reply.algorithm_no[i];
        servo_parameters_sg[i] = sg_reply.algorithm_parameters_no[i];

    }

    // przepisanie stanu regulatora chwytaka
    servo_gripper_reg_state = sg_reply.gripper_reg_state;

    // printf("edp_irp6s_and_conv_effector::send_to_SERVO_GROUP: %f, %f\n", current_motor_pos[4], sg_reply.abs_position[4]);

    // 	printf("current motor pos: %f\n", current_motor_pos[0]*IRP6_ON_TRACK_INC_PER_REVOLUTION/2*M_PI );

}
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::set_outputs (const lib::c_buffer &instruction)
{
    // ustawienie wyjsc binarnych
    in_out_obj.set_output(&instruction.output_values);
    // throw NonFatal_error_2(NOT_IMPLEMENTED_YET);
    // printf(" OUTPUTS SET\n");
}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::get_inputs (lib::r_buffer *local_reply)
{
    // odczytanie wejsc binarnych
    in_out_obj.get_input(&((*local_reply).input_values), ((*local_reply).analog_input));
    // throw NonFatal_error_2(NOT_IMPLEMENTED_YET);
    // printf(" INPUTS GET\n");
}
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::get_algorithms ()
{
    // odczytanie numerow algorytmow i ich numerow zestawow parametrow

    // Uformowanie rozkazu odczytu dla SERVO_GROUP
    servo_command.instruction_code = lib::READ;
    // Wyslanie rozkazu do SERVO_GROUP
    // Pobranie z SERVO_GROUP aktualnej pozycji silnikow i numerow algorytmow etc.
    send_to_SERVO_GROUP ();

}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
lib::REPLY_TYPE manip_and_conv_effector::rep_type (lib::c_buffer &instruction)
{
    // ustalenie formatu odpowiedzi
    reply.reply_type = lib::ACKNOWLEDGE;
    if (instruction.is_get_inputs())
    {
        reply.reply_type = lib::INPUTS;
    }
    if (instruction.is_get_rmodel())
    {
        if (reply.reply_type == lib::ACKNOWLEDGE)
            reply.reply_type = lib::RMODEL;
        else
            reply.reply_type = lib::RMODEL_INPUTS;
    }
    if (instruction.is_get_arm())
    {
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
    }
    real_reply_type = reply.reply_type;
    if (instruction.is_set_arm())
    {// by Y ORIGINAL
        // if (is_set_arm()||is_set_force()) {// by Y DEBUG
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
    }
    // by Y
    if (instruction.is_get_controller_state())
    {
        reply.reply_type= lib::CONTROLLER_STATE;
    }

    return reply.reply_type;
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::compute_motors(const lib::c_buffer &instruction)
{
    // obliczenia dla ruchu ramienia (silnikami)
    /* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
    /* Zlecenie transformerowi przeliczenie wspolrzednych */
    const double* p;   // wskanik miejsca w strukturze przesanej z ECP, w ktorym znajduj sie wspolrzedne

    motion_type = instruction.motion_type;
    motion_steps = instruction.motion_steps;
    value_in_step_no = instruction.value_in_step_no;
    p = &instruction.arm.pf_def.arm_coordinates[0];
    if ( (motion_steps <= 0) /* || (value_in_step_no < 0) */ )
        throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
    switch (motion_type)
    {
    case lib::ABSOLUTE:   // ruch bezwzgledny
        for (int i=0; i<number_of_servos; i++)
        {
            desired_motor_pos_new_tmp[i] = p[i];
            //          printf("i: %d, d: %f, p: %f\n",i, desired_motor_pos_new[i], p[i]); // DEBUG
        }
        // sprawdzi przekroczenie dopuszczalnego zakresu
        // check_motor_position(desired_motor_pos_new);

        // dla sprawdzenia ograniczen w joints i motos
        get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new_tmp, desired_joints_tmp);
        break;
    case lib::RELATIVE:   // ruch wzgledny
        for (int i=0; i<number_of_servos; i++)
        {
            desired_motor_pos_new_tmp[i] = p[i] + desired_motor_pos_new[i];
        }
        // Jesli robot zsynchronizowany sprawdzi przekroczenie dopuszczalnego zakresu
        if (synchronised)
        {
            //  check_motor_position(desired_motor_pos_new);
            // dla sprawdzenia ograncizen w joints i motors

            get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new_tmp, desired_joints_tmp);
        }
        break;
    default:
        throw NonFatal_error_2(INVALID_MOTION_TYPE);
    }

    // kinematyka nie stwierdzila bledow, przepisanie wartosci
    for (int i=0; i< number_of_servos; i++)
    {
        desired_joints[i] = desired_joints_tmp[i];
        desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
    }

    // printf("P=%lf\n",desired_motor_pos_new[0]);
}
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::compute_joints (const lib::c_buffer &instruction)
{
    // obliczenia dla ruchu ramienia (stawami)
    /* Wypenienie struktury danych transformera na podstawie parametrow polecenia otrzymanego z ECP */
    /* Zlecenie transformerowi przeliczenie wspolrzednych */
    const double* p; // wskanik miejsca w strukturze przeslanej z ECP, w ktorym znajduje sie wspolrzedne

    motion_type = instruction.motion_type;
    motion_steps = instruction.motion_steps;
    value_in_step_no = instruction.value_in_step_no;
    p = &instruction.arm.pf_def.arm_coordinates[0];
    if ( (value_in_step_no <= 0) || (motion_steps <= 0) || (value_in_step_no   > motion_steps + 1) )
        throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
    switch (motion_type)
    {
    case lib::ABSOLUTE:   // ruch bezwzgledny
        for (int i=0; i<number_of_servos; i++)
            desired_joints_tmp[i] = p[i];
        break;
    case lib::RELATIVE:   // ruch wzgledny
            for (int i=0; i<number_of_servos; i++)
                desired_joints_tmp[i] = desired_joints[i] + p[i];
        break;
    default:
            throw NonFatal_error_2(INVALID_MOTION_TYPE);
    }
    // check_joints(desired_joints);
    get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new_tmp, desired_joints_tmp);

    // kinematyka nie stwierdzila bledow, przepisanie wartosci
    for (int i=0; i< number_of_servos; i++)
    {
        desired_joints[i] = desired_joints_tmp[i];
        desired_motor_pos_new[i] = desired_motor_pos_new_tmp[i];
    }

    for (int i=0; i<number_of_servos; i++)
        previous_joints[i] = desired_joints[i];
}
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void manip_and_conv_effector::move_servos ()
{
    /* Wyslanie polecenia ruchu do procesu SERVO_GROUP oraz odebranie wyniku
        realizacji pierwszej fazy ruchu */
    int i;

    /* Uformowanie rozkazu ruchu dla SERVO_GROUP */
    servo_command.instruction_code = lib::MOVE;
    servo_command.parameters.move.number_of_steps = motion_steps;
    servo_command.parameters.move.return_value_in_step_no = value_in_step_no;

    //		printf("edp_irp6s_and_conv_effector::move_servos: %f, %f\n", desired_motor_pos_new[1], desired_motor_pos_old[1]);

    for (i=0; i < number_of_servos; i++)
    {
        servo_command.parameters.move.macro_step[i] = desired_motor_pos_new[i] - desired_motor_pos_old[i];
        servo_command.parameters.move.abs_position[i] = desired_motor_pos_new[i]; // by Y
        //    nowa wartosc zadana staje sie stara

        desired_motor_pos_old[i] = desired_motor_pos_new[i];
    }

    /*
         printf("move_servos_aa: %f, %f, %f, %f, %f, %f, %f, %d\n", servo_command.parameters.move.abs_position[0], servo_command.parameters.move.abs_position[1], servo_command.parameters.move.abs_position[2], servo_command.parameters.move.abs_position[3],
         		 servo_command.parameters.move.abs_position[4], servo_command.parameters.move.abs_position[5], servo_command.parameters.move.abs_position[6], servo_command.parameters.move.number_of_steps);
    */
    /* Wyslanie makrokroku do realizacji procesowi SERVO_GROUP */
    /* Odebranie od procesu SERVO_GROUP informacji o realizacji pierwszej fazy ruchu */
    send_to_SERVO_GROUP ();

}
/*--------------------------------------------------------------------------*/

void manip_and_conv_effector::update_servo_current_motor_pos(double motor_position_increment, int i)
{
    servo_current_motor_pos[i]+=motor_position_increment;
}

void manip_and_conv_effector::update_servo_current_motor_pos_abs(double abs_motor_position, int i)
{
    servo_current_motor_pos[i]=abs_motor_position;
}


// sprawdza stan EDP zaraz po jego uruchomieniu





void manip_and_conv_effector::common_get_controller_state(lib::c_buffer &instruction)
{
    synchronised = reply.controller_state.is_synchronised = controller_state_edp_buf.is_synchronised;
    reply.controller_state.is_power_on = controller_state_edp_buf.is_power_on;
    reply.controller_state.is_wardrobe_on = controller_state_edp_buf.is_wardrobe_on;
    reply.controller_state.is_controller_card_present = controller_state_edp_buf.is_controller_card_present;
    reply.controller_state.is_robot_blocked = controller_state_edp_buf.is_robot_blocked;

    // aktualizacja pozycji robota
    // Uformowanie rozkazu odczytu dla SERVO_GROUP
    servo_command.instruction_code = lib::READ;
    // Wyslanie rozkazu do SERVO_GROUP
    // Pobranie z SERVO_GROUP aktualnej pozycji silnikow
    //	printf("get_arm_position read_hardware\n");

    send_to_SERVO_GROUP ();

    // dla pierwszego wypelnienia current_joints
    get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

    // Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
    for( int i = 0; i < number_of_servos; i++)
    {
        servo_current_motor_pos[i] = desired_motor_pos_new_tmp[i] = desired_motor_pos_new[i] =
                                         desired_motor_pos_old[i] = current_motor_pos[i];
        desired_joints_tmp[i] = desired_joints[i] = current_joints[i];
    }
}



//   sprawdza stan robota
void manip_and_conv_effector::get_controller_state(lib::c_buffer &instruction)
{
	common_get_controller_state(instruction);
}




void manip_and_conv_effector::main_loop ()
{

    // by Y pierwsza petla while do odpytania o stan EDP przez UI zaraz po starcie EDP
    next_state = GET_STATE;

    while ((next_state != GET_INSTRUCTION) && (next_state != GET_SYNCHRO))
    {

        try
        { // w tym bloku beda wylapywane wyjatki (bledy)
            switch (next_state)
            {
            case GET_STATE:
                // wstepna interpretacja nadeslanego polecenia w celu wykrycia nieprawidlowosci
                switch ( receive_instruction() )
                {
                case lib::GET:
                    // potwierdzenie przyjecia polecenia (dla ECP)
                    //            printf("SET_GET\n");
                    insert_reply_type(lib::ACKNOWLEDGE);
                    reply_to_instruction();

                    if ((rep_type(new_instruction)) == lib::CONTROLLER_STATE)
                    {
                        // mt_tt_obj.master_to_trans_t_order(MT_GET_CONTROLLER_STATE, 0);
                        interpret_instruction (new_instruction);
                    }
                    else
                    {
                        throw manip_and_conv_effector::NonFatal_error_1 (INVALID_INSTRUCTION_TYPE);
                    }

                    break;
                case lib::QUERY: // blad: nie ma o co pytac - zadne polecenie uprzednio nie zostalo wydane
                    // okreslenie numeru bledu
                    throw manip_and_conv_effector::NonFatal_error_1 (QUERY_NOT_EXPECTED);
                default: // blad: nieznana instrukcja
                    // okreslenie numeru bledu
                    throw manip_and_conv_effector::NonFatal_error_1 (INVALID_INSTRUCTION_TYPE);
                }
                next_state = WAIT;
                break;
            case WAIT:
                if ( receive_instruction() == lib::QUERY )
                { // instrukcja wlasciwa =>
                    // zle jej wykonanie, czyli wyslij odpowiedz
                    reply_to_instruction();
                }
                else
                { // blad: powinna byla nadejsc instrukcja QUERY
                    throw manip_and_conv_effector::NonFatal_error_3 ( QUERY_EXPECTED );
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

        catch(manip_and_conv_effector::NonFatal_error_1 nfe)
        {
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

        catch(manip_and_conv_effector::NonFatal_error_2 nfe)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
            //  printf ("catch master thread NonFatal_error_2\n");
            establish_error(nfe.error,OK);
            msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
            // powrot do stanu: WAIT
            next_state = WAIT;
        } // end: catch(transformer::NonFatal_error_2 nfe)

        catch(manip_and_conv_effector::NonFatal_error_3 nfe)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesowa
            // zapamietanie poprzedniej odpowiedzi
            // Oczekiwano na QUERY a otrzymano co innego, wiec sygnalizacja bledu i
            // dalsze oczekiwanie na QUERY
            lib::REPLY_TYPE rep_type = is_reply_type();
            uint64_t err_no_0 = is_error_no_0();
            uint64_t err_no_1 = is_error_no_1();

            establish_error(nfe.error,OK);
            // informacja dla ECP o bledzie
            reply_to_instruction();
            // przywrocenie poprzedniej odpowiedzi
            insert_reply_type(rep_type);
            establish_error(err_no_0,err_no_1);
            //     printf("ERROR w EDP 3\n");
            msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
            // msg->message(lib::NON_FATAL_ERROR, err_no_0, err_no_1); // by Y - oryginalnie
            // powrot do stanu: GET_INSTRUCTION
            next_state = GET_STATE;
        } // end: catch(transformer::NonFatal_error_3 nfe)

        catch(manip_and_conv_effector::Fatal_error fe)
        {
            //     printf("ERROR w EDP transformer fe\n");
            // Obsluga bledow fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu fe
            // Sa to bledy dotyczace sprzetu oraz QNXa (komunikacji)
            establish_error(fe.error0,fe.error1);
            msg->message(lib::FATAL_ERROR, fe.error0, fe.error1);
            // Powrot do stanu: WAIT
            next_state = WAIT;
        } // end: catch(transformer::Fatal_error fe)

    } // end while



    while (next_state != GET_INSTRUCTION)
    {

        try
        { // w tym bloku beda wylapywane wyjatki (bledy)
            switch (next_state)
            {
            case GET_SYNCHRO:
                /* Oczekiwanie na zlecenie synchronizacji robota */
                switch ( receive_instruction() )
                {
                case lib::SYNCHRO:
                    // instrukcja wlasciwa => zle jej wykonanie
                    /* Potwierdzenie przyjecia instrukcji synchronizacji do wykonania */
                    insert_reply_type(lib::ACKNOWLEDGE);
                    reply_to_instruction();
                    /* Zlecenie wykonania synchronizacji */
                    mt_tt_obj.master_to_trans_t_order(MT_SYNCHRONISE, 0);   // by Y przejscie przez watek transfor w celu ujednolicenia
                    // synchronise();
                    // Jezeli synchronizacja okae sie niemoliwa, to zostanie zgloszony wyjatek:
                    /* Oczekiwanie na poprawne zakoczenie synchronizacji */
                    next_state = SYNCHRO_TERMINATED;
                    break;
                case lib::SET:
                    // instrukcja wlasciwa => zle jej wykonanie
                    if ( pre_synchro_motion(new_instruction) )
                    {
                        /* Potwierdzenie przyjecia instrukcji ruchow presynchronizacyjnych do wykonania */
                        insert_reply_type(lib::ACKNOWLEDGE);
                        reply_to_instruction();
                        /* Zlecenie wykonania ruchow presynchronizacyjnych */
                        interpret_instruction(new_instruction);
                        // Jezeli wystapil blad w trakcie realizacji ruchow presynchronizacyjnych,
                        // to zostanie zgloszony wyjatek:

                        /* Oczekiwanie na poprawne zakoczenie synchronizacji */
                        next_state = WAIT_Q;
                    }
                    else // blad: jedyna instrukcja w tym stanie moze by polecenie
                        // synchronizacji lub ruchow presynchronizacyjnych
                        // Bez synchronizacji adna inna instrukcja nie moze by wykonana przez EDP
                        /* Informacja o bedzie polegajcym na braku polecenia synchronizacji */
                        throw manip_and_conv_effector::NonFatal_error_1(NOT_YET_SYNCHRONISED);
                    break;
                default: // blad: jedyna instrukcja w tym stanie moze by polecenie
                    // synchronizacji lub ruchow presynchronizacyjnych
                    // Bez synchronizacji adna inna instrukcja nie moze by wykonana przez EDP
                    /* Informacja o bedzie polegajcym na braku polecenia synchronizacji */
                    throw manip_and_conv_effector::NonFatal_error_1(INVALID_INSTRUCTION_TYPE);
                }
                break;
            case SYNCHRO_TERMINATED:
                /* Oczekiwanie na zapytanie od ECP o status zakonczenia synchronizacji (QUERY) */
                if ( receive_instruction() == lib::QUERY )
                { // instrukcja wlasciwa => zle jej wykonanie
                    // Budowa adekwatnej odpowiedzi
                    insert_reply_type(lib::SYNCHRO_OK);
                    reply_to_instruction();
                    next_state = GET_INSTRUCTION;
                    if (msg->message("Robot is synchronised"))
                        printf(" Nie znaleziono SR\n");
                }
                else
                { // blad: powinna byla nadejsc instrukcja QUERY
                    throw manip_and_conv_effector::NonFatal_error_4(QUERY_EXPECTED);
                }
                break;
            case WAIT_Q:
                /* Oczekiwanie na zapytanie od ECP o status zakonczenia synchronizacji (QUERY) */
                if ( receive_instruction() == lib::QUERY )
                { // instrukcja wlasciwa => zle jej wykonanie
                    // Budowa adekwatnej odpowiedzi
                    reply_to_instruction();
                    next_state = GET_SYNCHRO;
                }
                else
                { // blad: powinna byla nadejsc instrukcja QUERY
                    throw manip_and_conv_effector::NonFatal_error_3(QUERY_EXPECTED);
                }
                break;
            default:
                break;
            }
        }

        // printf("debug edp po while\n");		// by Y&W

        catch(manip_and_conv_effector::NonFatal_error_1 nfe1)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
            establish_error(nfe1.error,OK);
            reply_to_instruction();
            msg->message(lib::NON_FATAL_ERROR, nfe1.error, (uint64_t)0 );
            // powrot do stanu: GET_SYNCHRO
            next_state = GET_SYNCHRO;
        } // end: catch(transformer::NonFatal_error_1 nfe1)

        catch(manip_and_conv_effector::NonFatal_error_2 nfe2)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
            // zapamietanie poprzedniej odpowiedzi
            lib::REPLY_TYPE rep_type = is_reply_type();
            establish_error(nfe2.error,OK);
            reply_to_instruction();
            msg->message(lib::NON_FATAL_ERROR, nfe2.error, (uint64_t)0);
            // przywrocenie poprzedniej odpowiedzi
            insert_reply_type(rep_type); // powrot do stanu: WAIT_Q
            next_state = WAIT_Q;
        } // end: catch(transformer::NonFatal_error nfe2)

        catch(manip_and_conv_effector::NonFatal_error_3 nfe3)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
            // zapamietanie poprzedniej odpowiedzi
            lib::REPLY_TYPE rep_type = is_reply_type();
            uint64_t err_no_0 = is_error_no_0();
            uint64_t err_no_1 = is_error_no_1();
            establish_error(nfe3.error,OK);
            reply_to_instruction();
            msg->message(lib::NON_FATAL_ERROR, nfe3.error, (uint64_t)0);
            // przywrocenie poprzedniej odpowiedzi
            insert_reply_type(rep_type);
            establish_error(err_no_0,err_no_1);
            msg->message(lib::NON_FATAL_ERROR, err_no_0, err_no_1);
            // powrot do stanu: GET_SYNCHRO
            next_state = GET_SYNCHRO;
        } // end: catch(transformer::NonFatal_error nfe3)

        catch(manip_and_conv_effector::NonFatal_error_4 nfe4)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
            // zapamietanie poprzedniej odpowiedzi
            lib::REPLY_TYPE rep_type = is_reply_type();
            establish_error(nfe4.error,OK);
            reply_to_instruction();
            msg->message(lib::NON_FATAL_ERROR, nfe4.error, (uint64_t)0);
            // przywrocenie poprzedniej odpowiedzi
            insert_reply_type(rep_type);
            // powrot do stanu: SYNCHRO_TERMINATED
            next_state = SYNCHRO_TERMINATED;
        } // end: catch(transformer::NonFatal_error nfe4)

        catch(manip_and_conv_effector::Fatal_error fe)
        {
            // Obsluga bledow fatalnych
            // Konkretny numer bledu znajduje sie w skadowych error0 lub error1 obiektu fe
            // Sa to bledy dotyczace sprzetu oraz QNXa (komunikacji)
            if ( receive_instruction() != lib::QUERY )
            {
                // blad: powinna byla nadejsc instrukcja QUERY
                establish_error(QUERY_EXPECTED,OK);
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
    for (;;)
    {

        try
        { // w tym bloku beda wylapywane wyjatki (bledy)
            switch (next_state)
            {
            case GET_INSTRUCTION:
                // wstepna interpretacja nadesanego polecenia w celu wykrycia nieprawidlowosci
                switch ( receive_instruction() )
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
                    throw manip_and_conv_effector::NonFatal_error_1(ALREADY_SYNCHRONISED);
                case lib::QUERY: // blad: nie ma o co pytac - zadne polecenie uprzednio nie zostalo wydane
                    // okreslenie numeru bledu
                    throw manip_and_conv_effector::NonFatal_error_1(QUERY_NOT_EXPECTED);
                default: // blad: nieznana instrukcja
                    // okreslenie numeru bledu
                    throw manip_and_conv_effector::NonFatal_error_1(UNKNOWN_INSTRUCTION);
                }
                next_state = EXECUTE_INSTRUCTION;
                break;
            case EXECUTE_INSTRUCTION:
                // wykonanie instrukcji - wszelkie bledy powoduja zgloszenie wyjtku NonFatal_error_2 lub Fatal_error
                interpret_instruction (new_instruction);
                next_state = WAIT;
                break;
            case WAIT:
                if ( receive_instruction() == lib::QUERY )
                { // instrukcja wlasciwa =>
                    // zle jej wykonanie, czyli wyslij odpowiedz
                    reply_to_instruction();
                }
                else
                { // blad: powinna byla nadejsc instrukcja QUERY
                    throw manip_and_conv_effector::NonFatal_error_3(QUERY_EXPECTED);
                }
                next_state = GET_INSTRUCTION;
                break;
            default:
                break;
            }
        }

        catch(manip_and_conv_effector::NonFatal_error_1 nfe)
        {
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

        catch(manip_and_conv_effector::NonFatal_error_2 nfe)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
            // printf ("catch master thread NonFatal_error_2\n");
            establish_error(nfe.error,OK);
            msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
            // powrot do stanu: WAIT
            next_state = WAIT;
        } // end: catch(transformer::NonFatal_error_2 nfe)

        catch(manip_and_conv_effector::NonFatal_error_3 nfe)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // Sa to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesowa
            // zapamietanie poprzedniej odpowiedzi
            // Oczekiwano na QUERY a otrzymano co innego, wiec sygnalizacja bledu i
            // dalsze oczekiwanie na QUERY
            lib::REPLY_TYPE rep_type = is_reply_type();
            uint64_t err_no_0 = is_error_no_0();
            uint64_t err_no_1 = is_error_no_1();

            establish_error(nfe.error,OK);
            // informacja dla ECP o bledzie
            reply_to_instruction();
            // przywrocenie poprzedniej odpowiedzi
            insert_reply_type(rep_type);
            establish_error(err_no_0,err_no_1);
            // printf("ERROR w EDP 3\n");
            msg->message(lib::NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
            // msg->message(lib::NON_FATAL_ERROR, err_no_0, err_no_1); // by Y - oryginalnie
            // powrot do stanu: GET_INSTRUCTION
            next_state = GET_INSTRUCTION;
        } // end: catch(transformer::NonFatal_error_3 nfe)

        catch(manip_and_conv_effector::Fatal_error fe)
        {
            // Obsluga bledow fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu fe
            // Sa to bledy dotyczace sprzetu oraz QNXa (komunikacji)
            establish_error(fe.error0,fe.error1);
            msg->message(lib::FATAL_ERROR, fe.error0, fe.error1);
            // Powrot do stanu: WAIT
            next_state = WAIT;
        } // end: catch(transformer::Fatal_error fe)

    } // end: for (;;)

}



/**************************** IN_OUT_BUFFER *****************************/

in_out_buffer::in_out_buffer()
{
    binary_input = 0;
    binary_output = 0;
    for (int i=0; i<8; i++)
    {
        analog_input[i]=0;
    }

    // inicjacja spinlockow
#ifdef __QNXNTO__
    memset( &input_spinlock, 0, sizeof(input_spinlock));
    memset( &output_spinlock, 0, sizeof(output_spinlock));
#else
    pthread_spin_init(&input_spinlock, PTHREAD_PROCESS_PRIVATE);
    pthread_spin_init(&output_spinlock, PTHREAD_PROCESS_PRIVATE);
#endif
    set_output_flag=false;
}

in_out_buffer::~in_out_buffer() {
#ifndef __QNXNTO__
	pthread_spin_destroy(&input_spinlock);
	pthread_spin_destroy(&output_spinlock);
#endif
}


// ustawienie wyjsc
void in_out_buffer::set_output(const lib::WORD *out_value)
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&output_spinlock);

    set_output_flag=true;   // aby f. obslugi przerwania wiedziala ze ma ustawic wyjscie
    binary_output=*out_value;

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&output_spinlock);
}

// odczytanie wyjsc
void in_out_buffer::get_output(lib::WORD *out_value)
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&output_spinlock);

    *out_value=binary_output;

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&output_spinlock);
}


// ustawienie wejsc
void in_out_buffer::set_input (const lib::WORD *binary_in_value, const lib::BYTE *analog_in_table)
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&input_spinlock);

    binary_input=*binary_in_value;		// wejscie binarne
    for (int i=0; i<8; i++)
    {
        analog_input[i]=analog_in_table[i];
    }

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&input_spinlock);

    /*	analog_in_value = & read_analog;
    	binary_in_value =   & read_binary;*/

    // printf("%x\n", 0x00FF&(~odczyt));
}


// odczytanie wejsc
void in_out_buffer::get_input (lib::WORD *binary_in_value, lib::BYTE *analog_in_table)
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&input_spinlock);

    *binary_in_value=binary_input;		// wejscie binarne
    for (int i=0; i<8; i++)
    {
        analog_in_table[i]=analog_input[i];
    }

    /*	// ustawienie korzystanie z ukladu we-wy
    	out8(ADR_OF_SERVO_PTR, IN_OUT_PTR);

    	// odczytanie wejsc
    	// SERVO_REPLY_STATUS_ADR     0x210
    	lib::WORD read_analog = 0x00FF & in16(SERVO_REPLY_STATUS_ADR);
    	// SERVO_REPLY_REG_1_ADR       0x218
    	lib::WORD read_binary = 0x00FF & in16(SERVO_REPLY_REG_1_ADR);*/

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&input_spinlock);

    /*	analog_in_value = & read_analog;
    	binary_in_value =   & read_binary;*/

    // printf("%x\n", 0x00FF&(~odczyt));
}

/**************************** IN_OUT_BUFFER *****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp
