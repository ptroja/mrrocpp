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
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <process.h>
#include <sys/netmgr.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mis_fun.h"
#include "edp/common/edp_irp6s_postument_track.h"
#include "lib/mathtr.h"



void edp_irp6s_postument_track_effector::initialize (void)
{}
;


/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::set_rmodel (c_buffer *instruction)
{
    // BYTE previous_model;
    // BYTE previous_corrector;

    //printf(" SET RMODEL: ");
    switch ((*instruction).set_rmodel_type)
    {
    case TOOL_FRAME:
        //printf("TOOL_FRAME\n");
        // przepisa specyfikacj do TRANSFORMATORa
        tool_frame_2_frame(instruction);
        break;
    case TOOL_XYZ_ANGLE_AXIS:
        //printf("TOOL_XYZ_ANGLE_AXIS\n");
        // przeksztaci i przepisa specyfikacj do TRANSFORMATORa
        tool_xyz_aa_2_frame(instruction);
        break;
    case TOOL_XYZ_EULER_ZYZ:
        //printf("TOOL_XYZ_EULER_ZYZ\n");
        // przeksztaci i przepisa specyfikacj do TRANSFORMATORa
        tool_xyz_eul_zyz_2_frame(instruction);
        break;
    case ARM_KINEMATIC_MODEL:
        //printf("ARM_KINEMATIC_MODEL\n");
        // Ustawienie modelu kinematyki.
        set_kinematic_model((*instruction).rmodel.kinematic_model.kinematic_model_no);
        break;
    case SERVO_ALGORITHM:
        // ustawienie algorytmw serworegulacji oraz ich parametrow
        // zmiana algorytmu regulacji
        /* Uformowanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow dla procesu SERVO_GROUP */
        servo_command.instruction_code = SERVO_ALGORITHM_AND_PARAMETERS;
        for (int i = 0; i<number_of_servos; i++)
        {
            servo_command.parameters.servo_alg_par.servo_algorithm_no[i] = servo_algorithm_ecp[i] = (*instruction).rmodel.servo_algorithm.servo_algorithm_no[i];
            servo_command.parameters.servo_alg_par.servo_parameters_no[i] = servo_parameters_ecp[i] = (*instruction).rmodel.servo_algorithm.servo_parameters_no[i];
        }
        ; // end: for
        /* Wyslanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow procesowi SERVO_GROUP */
        send_to_SERVO_GROUP (); //
        break;
    case FORCE_TOOL:

        vs->force_sensor_set_tool = true;
        for (int i = 0; i<3; i++)
        {
            vs->next_force_tool_position[i] = (*instruction).rmodel.force_tool.position[i];
        }
        vs->next_force_tool_weight = (*instruction).rmodel.force_tool.weight;
        vs->check_for_command_execution_finish();
        break;
    case FORCE_BIAS:
        vs->force_sensor_do_configure = true;
        vs->check_for_command_execution_finish();
        break;
    default: // blad: nie istniejca specyfikacja modelu robota
        // ustawi numer bledu
        throw NonFatal_error_2(INVALID_SET_RMODEL_TYPE);
    }
    ; // end: switch (set_rmodel_type)
}
; // end: edp_irp6s_postument_track_effector::set_rmodel
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::get_rmodel (c_buffer *instruction)
{
    int i; // licznik obiegow petli
    //printf(" GET RMODEL: ");
    switch ((*instruction).get_rmodel_type)
    {
    case TOOL_FRAME:
        //printf("TOOL_FRAME\n");
        // przepisa specyfikacj z TRANSFORMATORa do bufora wysykowego
        tool_frame_2_frame_rep();
        break;
    case TOOL_XYZ_ANGLE_AXIS:
        //printf("TOOL_XYZ_ANGLE_AXIS\n");
        // przeksztaci i przepisa specyfikacj z TRANSFORMATORa do bufora wysykowego
        tool_frame_2_xyz_aa();
        break;
    case TOOL_XYZ_EULER_ZYZ:
        //printf("TOOL_XYZ_EULER_ZYZ\n");
        // przeksztaci i przepisa specyfikacj z TRANSFORMATORa do bufora wysykowego
        tool_frame_2_xyz_eul_zyz();
        break;
    case ARM_KINEMATIC_MODEL:
        reply.rmodel_type = ARM_KINEMATIC_MODEL;
        // okreslenie numeru zestawu parametrow przelicznika kinematycznego oraz jego korektora
        reply.rmodel.kinematic_model.kinematic_model_no = get_current_kinematic_model_no();
        break;
    case SERVO_ALGORITHM:
        reply.rmodel_type = SERVO_ALGORITHM;
        // ustawienie numeru algorytmu serworegulatora oraz numeru jego zestawu parametrow
        for (i = 0; i<number_of_servos; i++)
            if (instruction->is_get_arm())
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
    case FORCE_TOOL:
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
    ; // end: switch (get_rmodel_type)
}
; // end: edp_irp6s_postument_track_effector::get_rmodel
/*--------------------------------------------------------------------------*/




/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::arm_frame_2_xyz_eul_zyz ()
{

    // Przeksztacenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztacenia do
    // wewntrznych struktur danych REPLY_BUFFER
    Homog_matrix A(current_end_effector_frame);
    switch (reply.reply_type)
    {
    case ARM:
    case ARM_INPUTS:
    case ARM_RMODEL:
    case ARM_RMODEL_INPUTS:
        //A.get_mech_xyz_euler_zyz(reply.arm.coordinate_def.arm_coordinates);
        //A.get_mech_xyz_euler_zyz(rb_obj->step_data.current_kartez_position);
        A.get_xyz_euler_zyz(reply.arm.coordinate_def.arm_coordinates);
        A.get_xyz_euler_zyz(rb_obj->step_data.current_kartez_position);
        break;
    default: // blad:
        throw NonFatal_error_2(STRANGE_GET_ARM_REQUEST);
    }
    ; // end: switch (reply.reply_type)
    // dla robotow track i postument - oblicz chwytak
    if ((robot_name == ROBOT_IRP6_ON_TRACK) || (robot_name == ROBOT_IRP6_POSTUMENT))
    {
        reply.arm.coordinate_def.gripper_reg_state = servo_gripper_reg_state;
        reply.arm.coordinate_def.gripper_coordinate = current_joints[gripper_servo_nr];
    }

}
; // end: edp_irp6s_effector::arm_frame_2_xyz_eul_zyz
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::arm_abs_xyz_eul_zyz_2_frame (double *p)
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
    //Homog_matrix A_B_T (Homog_matrix::MTR_MECH_XYZ_EULER_ZYZ, x, y, z, alfa, beta, gamma);
    Homog_matrix A_B_T (Homog_matrix::MTR_XYZ_EULER_ZYZ, x, y, z, alfa, beta, gamma);
    A_B_T.get_frame_tab(desired_end_effector_frame);

}
; // end: edp_irp6s_effector::arm_abs_xyz_eul_zyz_2_frame
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
edp_irp6s_postument_track_effector::edp_irp6s_postument_track_effector (configurator &_config, ROBOT_ENUM l_robot_name) :
        edp_irp6s_effector (_config, l_robot_name)
{

    pthread_mutex_init (&force_mutex, NULL );

    // czujnik sil nie zostal jeszcze skonfigurowany po synchronizacji robota
    force_sensor_configured = false;

    if (config.exists("force_tryb"))
        force_tryb = config.return_int_value("force_tryb");
    else
        force_tryb = 0;

    // ustalenie ilosci stopni swobody dla funkcji obslugi przerwania i synchronizacji w zaleznosci od aktywnosci chwytaka
    if (config.exists("is_gripper_active"))
        is_gripper_active = config.return_int_value("is_gripper_active");
    else
        is_gripper_active = 1;

    vs->TERMINATE=false;

};




void edp_irp6s_postument_track_effector::pose_force_linear_move (c_buffer *instruction)
{}
;



/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::create_threads ()
{

    edp_irp6s_effector::create_threads();

    // jesli wlaczono obsluge sily
    if (force_tryb > 0)
    {
        // byY - utworzenie watku pomiarow sily
        if (pthread_create (&force_tid, NULL, &force_thread_start, (void *) this)!=EOK)
        {
            msg->message(SYSTEM_ERROR, errno, "EDP: Failed to spawn READER");
            char buf[20];
            netmgr_ndtostr(ND2S_LOCAL_STR, ND_LOCAL_NODE, buf, sizeof(buf));
            printf (" Failed to thread FORCE_thread on node: %s\n", buf);
            throw System_error();
        }

        // by Y - utworzenie watku komunikacji miedzy EDP a VSP
        if (pthread_create (&edp_vsp_tid, NULL, &edp_vsp_thread_start, (void *) this)!=EOK)
        {
            msg->message(SYSTEM_ERROR, errno, "EDP: Failed to spawn READER");
            char buf[20];
            netmgr_ndtostr(ND2S_LOCAL_STR, ND_LOCAL_NODE, buf, sizeof(buf));
            printf (" Failed to thread EDP_VSP_thread on node: %s\n", buf);
            throw System_error();
        }
    }
};





//
/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::arm_frame_2_pose_force_torque_at_frame (bool* read_hardware)
{

    double current_force[6];

    switch (reply.reply_type)
    {
    case ARM:
    case ARM_INPUTS:
    case ARM_RMODEL:
    case ARM_RMODEL_INPUTS:
        {
            if(*read_hardware)
            {
                is_get_arm_read_hardware = true;
                copy_frame(reply.arm.pose_force_torque_at_frame_def.beggining_arm_frame, current_end_effector_frame);
                copy_frame(reply.arm.pose_force_torque_at_frame_def.predicted_arm_frame, current_end_effector_frame);
                copy_frame(reply.arm.pose_force_torque_at_frame_def.present_arm_frame, current_end_effector_frame);
            }

            Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);
            Ft_v_tr ft_tr_inv_current_frameatrix (!current_frame_wo_offset, Ft_v_tr::FT);

            Homog_matrix current_tool(get_current_kinematic_model()->tool);
            Ft_v_tr ft_tr_inv_tool_matrix (!current_tool, Ft_v_tr::FT);

            force_msr_download (current_force, NULL);
            // sprowadzenie sil z ukladu bazowego do ukladu kisci
            // modyfikacja pobranych sil w ukladzie czujnika - do ukladu wyznaczonego przez force_tool_frame i reference_frame

            Ft_v_vector current_force_torque (ft_tr_inv_tool_matrix *  ft_tr_inv_current_frameatrix * Ft_v_vector (current_force));

            current_force_torque.to_table (reply.arm.pose_force_torque_at_frame_def.force_xyz_torque_xyz);


        }
        break;
    default:
        throw NonFatal_error_2 (STRANGE_GET_ARM_REQUEST);
        break;
    };
    // dla robotow track i postument - oblicz chwytak
    reply.arm.pose_force_torque_at_frame_def.gripper_reg_state = servo_gripper_reg_state;

    if ((robot_name == ROBOT_IRP6_ON_TRACK) || (robot_name == ROBOT_IRP6_POSTUMENT))
    {
        reply.arm.pose_force_torque_at_frame_def.gripper_coordinate = current_joints[gripper_servo_nr];
    }

}
; // end: edp_irp6s_postument_track_effector::arm_frame_2_xyz_eul_zyz
/*--------------------------------------------------------------------------*/


// ruch pozycyjno silowy dla ramki TFF - testowane tylko dla trybu RELATIVE
/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::pose_force_torque_at_frameove (c_buffer *instruction)
{
    //	static int debugi=0;
    //   debugi++;

    motion_type = (*instruction).motion_type;

    // zmienne z bufora wejsciowego
    WORD ECP_motion_steps = instruction->motion_steps;	// liczba krokow w makrokroku
    int ECP_value_in_step_no = instruction->value_in_step_no;	// liczba krokow po ktorych bedzie wyslana odpowiedz do ECP o przewidywanym zakonczeniu ruchu

    const unsigned long PREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE = 10;
    double pos_xyz_rot_xyz[6];			// wartosci ruchu pozycyjnego
    double force_xyz_torque_xyz[6]; 	// wartosci uchybu sily

    double inertia[6];
    double reciprocal_damping[6];
    BEHAVIOUR_SPECIFICATION behaviour[6];

    memcpy (inertia, instruction->arm.pose_force_torque_at_frame_def.inertia, sizeof (double[6]) );
    memcpy (reciprocal_damping, instruction->arm.pose_force_torque_at_frame_def.reciprocal_damping, sizeof (double[6]) );
    memcpy (behaviour, instruction->arm.pose_force_torque_at_frame_def.behaviour, sizeof (BEHAVIOUR_SPECIFICATION[6]) );
    memcpy (force_xyz_torque_xyz, instruction->arm.pose_force_torque_at_frame_def.force_xyz_torque_xyz, sizeof (double[6]) );
    double desired_gripper_coordinate = instruction->arm.pose_force_torque_at_frame_def.gripper_coordinate;
    switch (motion_type)
    {
    case ABSOLUTE:
    case RELATIVE:
        std::cout << "wrong motion_type in edp_force" << std::endl;
        break;
    }

    motion_steps = 1;
    value_in_step_no = 0;

    double current_force[6], previous_force[6];

    double beginning_gripper_coordinate;
    static double ending_gripper_coordinate;
    static frame_tab local_force_end_effector_frame;

    static unsigned long last_force_step_counter = step_counter;

    Ft_v_vector move_rot_vector;
    Ft_v_vector pos_xyz_rot_xyz_vector;
    static Ft_v_vector previous_move_rot_vector;

    // WYLICZENIE POZYCJI POCZATKOWEJ
    double begining_joints[MAX_SERVOS_NR], tmp_joints[MAX_SERVOS_NR], tmp_motor_pos[MAX_SERVOS_NR];
    frame_tab begining_frame;

    get_current_kinematic_model()->mp2i_transform (desired_motor_pos_new, begining_joints);
    get_current_kinematic_model()->i2e_transform (begining_joints, &begining_frame);
    Homog_matrix begining_end_effector_frame (begining_frame);
    Homog_matrix next_frame = begining_end_effector_frame;


    frame_tab goal_frame_tab;
    Homog_matrix goal_frame;

    Homog_matrix goal_frame_increment_in_end_effector;
    Ft_v_vector goal_xyz_angle_axis_increment_in_end_effector;


    switch (motion_type)
    {
    case PF_XYZ_ANGLE_AXIS_ABSOLUTE_POSE:
        goal_frame.set_xyz_angle_axis (instruction->arm.pose_force_torque_at_frame_def.position_velocity);
        break;
    case PF_XYZ_ANGLE_AXIS_RELATIVE_POSE:
        goal_frame.set_xyz_angle_axis (instruction->arm.pose_force_torque_at_frame_def.position_velocity); // tutaj goal_frame jako zmienna tymczasowa
        goal_frame = begining_end_effector_frame * goal_frame;
        break;
    case PF_JOINTS_ABSOLUTE_POSITION:
        get_current_kinematic_model()->i2e_transform
        (instruction->arm.pose_force_torque_at_frame_def.position_velocity, &goal_frame_tab);
        goal_frame.set_frame_tab (goal_frame_tab);
        break;
    case PF_JOINTS_RELATIVE_POSITION:
        for (int i = 0; i < MAX_SERVOS_NR; i++)
        {
            tmp_joints[i] = begining_joints[i] + instruction->arm.pose_force_torque_at_frame_def.position_velocity[i];
        }
        get_current_kinematic_model()->i2e_transform (tmp_joints, &goal_frame_tab);
        goal_frame.set_frame_tab (goal_frame_tab);
        break;
    case PF_MOTORS_ABSOLUTE_POSITION:
        get_current_kinematic_model()->mp2i_transform
        (instruction->arm.pose_force_torque_at_frame_def.position_velocity, tmp_joints);
        get_current_kinematic_model()->i2e_transform (tmp_joints, &goal_frame_tab);
        goal_frame.set_frame_tab (goal_frame_tab);
        break;
    case PF_MOTORS_RELATIVE_POSITION:
        for (int i = 0; i < MAX_SERVOS_NR; i++)
        {
            tmp_motor_pos[i] = desired_motor_pos_new[i] + instruction->arm.pose_force_torque_at_frame_def.position_velocity[i];
        }
        get_current_kinematic_model()->mp2i_transform (tmp_motor_pos, tmp_joints);
        get_current_kinematic_model()->i2e_transform (tmp_joints, &goal_frame_tab);
        goal_frame.set_frame_tab (goal_frame_tab);
        break;
    default:
        break;
    }


    switch (motion_type)
    {
    case PF_XYZ_ANGLE_AXIS_ABSOLUTE_POSE:
    case PF_JOINTS_ABSOLUTE_POSITION:
    case PF_MOTORS_ABSOLUTE_POSITION:
    case PF_XYZ_ANGLE_AXIS_RELATIVE_POSE:
    case PF_JOINTS_RELATIVE_POSITION:
    case PF_MOTORS_RELATIVE_POSITION:
        goal_frame_increment_in_end_effector = ((!begining_end_effector_frame)*goal_frame);
        goal_frame_increment_in_end_effector.get_xyz_angle_axis(goal_xyz_angle_axis_increment_in_end_effector);
        for (int i = 0; i < 6; i++)
        {
            pos_xyz_rot_xyz[i] =goal_xyz_angle_axis_increment_in_end_effector[i] *
                                (double) (1/ ( ((double)STEP)*((double)ECP_motion_steps) ) );
        }
        break;
    case PF_VELOCITY:
        memcpy (pos_xyz_rot_xyz, instruction->arm.pose_force_torque_at_frame_def.position_velocity, sizeof (double[6]) );
        break;
    }

    Ft_v_vector base_pos_xyz_rot_xyz_vector (pos_xyz_rot_xyz);

    copy_frame (reply.arm.pose_force_torque_at_frame_def.beggining_arm_frame, begining_frame);

    beginning_gripper_coordinate = begining_joints[gripper_servo_nr];

    Homog_matrix current_tool (get_current_kinematic_model()->tool);

    //	std::cout << current_tool << std::endl;

    Ft_v_tr ft_tr_tool_matrix (current_tool, Ft_v_tr::FT);
    Ft_v_tr ft_tr_inv_tool_matrix = !ft_tr_tool_matrix;
    Ft_v_tr v_tr_tool_matrix  (current_tool, Ft_v_tr::V);
    Ft_v_tr v_tr_inv_tool_matrix = !v_tr_tool_matrix;


    // poczatek generacji makrokroku
    for (int step = 1; step <= ECP_motion_steps; step++)
    {

        Homog_matrix current_frame_wo_offset = return_current_frame (WITHOUT_TRANSLATION);

        Ft_v_tr ft_tr_current_frameatrix (current_frame_wo_offset, Ft_v_tr::FT);
        Ft_v_tr ft_tr_inv_current_frameatrix = !ft_tr_current_frameatrix;
        Ft_v_tr v_tr_current_frameatrix (current_frame_wo_offset, Ft_v_tr::V);
        Ft_v_tr v_tr_inv_current_frameatrix = !v_tr_current_frameatrix;

        force_msr_download (current_force, previous_force);
        // sprowadzenie sil z ukladu bazowego do ukladu kisci
        // modyfikacja pobranych sil w ukladzie czujnika - do ukladu wyznaczonego przez force_tool_frame i reference_frame



        Homog_matrix begining_end_effector_frame_with_current_translation = begining_end_effector_frame;
        begining_end_effector_frame_with_current_translation.set_translation_vector(next_frame);

        Homog_matrix modified_beginning_to_desired_end_effector_frame = !begining_end_effector_frame_with_current_translation *
                next_frame;
        /*
        		Ft_v_tr ft_tr_modified_beginning_to_desired_end_effector_frame (modified_beginning_to_desired_end_effector_frame, Ft_v_tr::FT);
        		Ft_v_tr ft_tr_inv_modified_beginning_to_desired_end_effector_frame = !ft_tr_modified_beginning_to_desired_end_effector_frame;
        		*/
        Ft_v_tr v_tr_modified_beginning_to_desired_end_effector_frame  (modified_beginning_to_desired_end_effector_frame, Ft_v_tr::V);
        Ft_v_tr v_tr_inv_modified_beginning_to_desired_end_effector_frame  = !v_tr_modified_beginning_to_desired_end_effector_frame;

        // koniec dla trybu FIXED

        Ft_v_vector current_force_torque (ft_tr_inv_tool_matrix *  ft_tr_inv_current_frameatrix * Ft_v_vector (current_force));
        //		Ft_v_vector tmp_force_torque (Ft_v_tr((!current_tool) * (!current_frame_wo_offset), Ft_v_tr::FT) * Ft_v_vector (current_force));
        Ft_v_vector previous_force_torque (ft_tr_inv_tool_matrix *  ft_tr_inv_current_frameatrix * Ft_v_vector (previous_force));
        /*
        		switch (motion_type)
        		{
        			case PF_XYZ_ANGLE_AXIS_ABSOLUTE_POSE:
        			case PF_JOINTS_ABSOLUTE_POSITION:
        			case PF_MOTORS_ABSOLUTE_POSITION:
        				current_force_torque =  ft_tr_modified_beginning_to_desired_end_effector_frame * current_force_torque;
        				previous_force_torque = ft_tr_modified_beginning_to_desired_end_effector_frame * previous_force_torque;
        			break;
        		}
        */

        //wyzerowanie historii dla dlugiej przerwy w sterowaniu silowym

        if (step_counter - last_force_step_counter > PREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE)
        {
            //			printf("\n\nPREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE\n\n");
            previous_move_rot_vector.set_values (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            //	printf("\n\nPREVIOUS_MOVE_VECTOR_NULL_STEP_VALUE NOT\n\n");
        }


        previous_move_rot_vector =  v_tr_inv_tool_matrix * v_tr_inv_current_frameatrix * previous_move_rot_vector;
        /*
        		switch (motion_type)
        		{
        			case PF_XYZ_ANGLE_AXIS_ABSOLUTE_POSE:
        			case PF_JOINTS_ABSOLUTE_POSITION:
        			case PF_MOTORS_ABSOLUTE_POSITION:
        				previous_move_rot_vector =  v_tr_modified_beginning_to_desired_end_effector_frame * previous_move_rot_vector;
        			break;
        		}
        */
        switch (motion_type)
        {
        case PF_XYZ_ANGLE_AXIS_ABSOLUTE_POSE:
        case PF_JOINTS_ABSOLUTE_POSITION:
        case PF_MOTORS_ABSOLUTE_POSITION:
        case PF_XYZ_ANGLE_AXIS_RELATIVE_POSE:
        case PF_JOINTS_RELATIVE_POSITION:
        case PF_MOTORS_RELATIVE_POSITION:
            pos_xyz_rot_xyz_vector =  v_tr_inv_modified_beginning_to_desired_end_effector_frame * base_pos_xyz_rot_xyz_vector;
            break;
        case PF_VELOCITY:
            pos_xyz_rot_xyz_vector =  base_pos_xyz_rot_xyz_vector;
            break;
        }


        // wyznaczenie predkosci z uwzglednieniem wirtualnej inercji i wirtualnego tarcia wiskotycznego
        for (int i = 0; i < 6; i++)
        {

            // MODYFIKACJA PARAMETROW W ZALEZNOSCI OD ZALOZONEGO ZACHOWANIA DLA DANEGO KIERUNKU
            switch (behaviour[i])
            {
            case UNGUARDED_MOTION:
                reciprocal_damping[i]= 0.0; // the force influence is eliminated
                // inertia is not eleliminated
                break;
            case GUARDED_MOTION:
                force_xyz_torque_xyz[i] = 0.0; // the desired force is set to zero
                break;
            case CONTACT:
                pos_xyz_rot_xyz_vector[i] = 0.0; // the desired velocity is set to zero
                break;
            default:
                break;
            }

            // PRAWO STEROWANIA
            move_rot_vector[i] =
                (( reciprocal_damping[i] * (force_xyz_torque_xyz[i] - current_force_torque[i]) + pos_xyz_rot_xyz_vector[i] ) * STEP * STEP +
                 reciprocal_damping[i] * inertia[i] * previous_move_rot_vector[i]) / (STEP + reciprocal_damping[i] * inertia[i]) ;
        }

        /*
        		switch (motion_type)
        		{
        			case PF_XYZ_ANGLE_AXIS_ABSOLUTE_POSE:
        			case PF_JOINTS_ABSOLUTE_POSITION:
        			case PF_MOTORS_ABSOLUTE_POSITION:
        				move_rot_vector =  v_tr_inv_modified_beginning_to_desired_end_effector_frame * move_rot_vector;
        			break;
        		}
        */



        previous_move_rot_vector =  v_tr_current_frameatrix * v_tr_tool_matrix * move_rot_vector;
        // 	end: sprowadzenie predkosci ruchu do orientacji  ukladu bazowego lub ukladu koncowki

        //		if (debugi%10==0) printf("aaa: %f\n", force_xyz_torque_xyz[0] + force_torque[0]);

        Homog_matrix rot_frame (Homog_matrix::MTR_XYZ_ANGLE_AXIS, move_rot_vector);


        // wyliczenie nowej pozycji zadanej
        next_frame = next_frame * rot_frame;

        rb_obj->lock_mutex();
        next_frame.get_xyz_euler_zyz(rb_obj->step_data.current_kartez_position);
        rb_obj->unlock_mutex();

        next_frame.get_frame_tab(desired_end_effector_frame);
        desired_joints_tmp[gripper_servo_nr] = beginning_gripper_coordinate +
                                               (((desired_gripper_coordinate - beginning_gripper_coordinate) / ECP_motion_steps) * step);

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
        move_servos ();
        if (step == ECP_value_in_step_no)
        { // przygotowanie predicted frame dla ECP
            next_frame.get_frame_tab(reply.arm.pose_force_torque_at_frame_def.present_arm_frame);
            Homog_matrix predicted_frame = next_frame;
            for (int i=0; i< ECP_motion_steps-ECP_value_in_step_no; i++)
            {
                predicted_frame = predicted_frame * rot_frame;
            }
            predicted_frame.get_frame_tab(reply.arm.pose_force_torque_at_frame_def.predicted_arm_frame);

            for (int i=0; i<6; i++)
            {
                reply.arm.pose_force_torque_at_frame_def.force_xyz_torque_xyz[i] = current_force_torque[i];
            }
            mt_tt_obj->trans_t_to_master_order_status_ready();
        }

        last_force_step_counter = step_counter;

    }

    next_frame.get_frame_tab (local_force_end_effector_frame);
    ending_gripper_coordinate = desired_gripper_coordinate;

}
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::move_arm (c_buffer *instruction)
{ // przemieszczenie ramienia
    // Wypenienie struktury danych transformera na podstawie parametrow polecenia
    // otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych


    switch ((*instruction).set_arm_type)
    {
    case MOTOR:
        compute_motors(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();
        break;
    case JOINT:
        compute_joints(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();
        break;
    case XYZ_EULER_ZYZ:

        // zapisanie wartosci zadanej dla readera
        rb_obj->lock_mutex();

        for (int i=0; i<6;i++)
        {
            rb_obj->step_data.current_kartez_position[i]=(*instruction).arm.coordinate_def.arm_coordinates[i];
        }

        rb_obj->unlock_mutex();

        compute_xyz_euler_zyz(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();

        break;
    case XYZ_ANGLE_AXIS:
        compute_xyz_angle_axis(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();
        break;
    case FRAME:
        compute_frame(instruction);
        move_servos ();
        mt_tt_obj->trans_t_to_master_order_status_ready();
        break;
    case POSE_FORCE_TORQUE_AT_FRAME:
        pose_force_torque_at_frameove (instruction);
        break;
    default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
        throw NonFatal_error_2(INVALID_SET_END_EFFECTOR_TYPE);
    }
    ; // end:   switch (instruction.set_arm_type)



    // by Y - uwaga na wyjatki, po rzuceniu wyjatku nie zostanie zaktualizowany previous_set_arm_type
    previous_set_arm_type = (*instruction).set_arm_type;

}
; // end: edp_irp6s_postument_track_effector::move_arm
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void edp_irp6s_postument_track_effector::get_arm_position (bool read_hardware, c_buffer *instruction)
{ // odczytanie pozycji ramienia

    //   printf(" GET ARM\n");

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
            desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
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
    switch ((*instruction).get_arm_type)
    {
    case FRAME:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
        arm_frame_2_frame();
        break;
    case   XYZ_ANGLE_AXIS:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
        arm_frame_2_xyz_aa();
        break;
    case   XYZ_EULER_ZYZ:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
        arm_frame_2_xyz_eul_zyz();   // dla sterowania pozycyjnego
        reply.arm_type = XYZ_EULER_ZYZ;
        break;
    case   POSE_FORCE_TORQUE_AT_FRAME:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        get_current_kinematic_model()->i2e_transform(current_joints, &current_end_effector_frame);
        arm_frame_2_pose_force_torque_at_frame(&read_hardware);   // dla sterowania pozycyjno - silowego
        reply.arm_type = POSE_FORCE_TORQUE_AT_FRAME;
        break;
    case   JOINT:
        // przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
        get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
        arm_joints_2_joints();
        break;
    case   MOTOR:
        arm_motors_2_motors();
        break;
    default:   // blad: nieznany sposob zapisu wspolrzednych koncowki
        printf("EFF_TYPE: %d\n",(*instruction).get_arm_type);
        throw NonFatal_error_2(INVALID_GET_END_EFFECTOR_TYPE);
    }
    ; // end: switch (instruction.get_arm_type)

    rb_obj->lock_mutex();// by Y
    reply.servo_step=rb_obj->step_data.step;
    rb_obj->unlock_mutex();

}
; // end: edp_irp6s_postument_track_effector::get_arm_position
/*--------------------------------------------------------------------------*/



// sprawdza stan EDP zaraz po jego uruchomieniu

void edp_irp6s_postument_track_effector::servo_joints_and_frame_actualization_and_upload (void)
{
    int i;
    static int catch_nr=0;

    //	static double rkpminusone[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //	static double rkpminustwo[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //	frame_tab tmp;


    //	Homog_matrix step_increment_frame;
    //	Homog_matrix servo_current_end_effector_frame_with_tool_and_base_wo_offset;

    // wyznaczenie nowych wartosci joints and frame dla obliczen w servo
    try
    {
        get_current_kinematic_model()->mp2i_transform(servo_current_motor_pos, servo_current_joints);

        rb_obj->lock_mutex();

        for (int j = 0; j < number_of_servos; j++)
        {
            rb_obj->step_data.current_joints[j] = servo_current_joints[j];
        }

        rb_obj->unlock_mutex();



        // T.K.: Obecne wywolanie
        // get_current_kinematic_model()->i2e_transform(servo_current_joints, &servo_current_end_effector_frame, NULL);
        // zastepuje wywolaniem metody, direct_kinematics_transform(), ktora rozwiazuje proste zagadnienie kinematyki
        // bez uwzglednienia narzedzia, czyli robi to, co poprzednio i2e z TOOL = null.
        //Uwaga: w edp_conveyor_effector jest podobnie.

        get_current_kinematic_model()->direct_kinematics_transform(servo_current_joints, &servo_current_frame_wo_tool);
        Homog_matrix A(servo_current_frame_wo_tool);

#ifdef EXTRA_COMPUTATION

        A.get_xyz_euler_zyz(servo_real_kartez_pos);


        get_current_kinematic_model()->i2e_transform(servo_current_joints, &tmp);
        servo_current_end_effector_frame_with_tool_and_base.set_frame_tab(tmp);
        servo_current_end_effector_frame_with_tool_and_base_wo_offset = servo_current_end_effector_frame_with_tool_and_base;
        servo_current_end_effector_frame_with_tool_and_base_wo_offset.remove_translation();


        // wyznaczenie przyrostu miedzy dwoma kolejnymi krokami
        step_increment_frame = !servo_previous_end_effector_frame_with_tool_and_base * servo_current_end_effector_frame_with_tool_and_base;

        // przepisanie starej na nowa
        servo_previous_end_effector_frame_with_tool_and_base = servo_current_end_effector_frame_with_tool_and_base;

        // przyrost w notacji xyz_angle_axis
        step_increment_frame.get_xyz_angle_axis (servo_xyz_angle_axis_rotation, servo_xyz_angle_axis_translation);
        /*
        		printf("b rx: %f, ry: %f, rz: %f\n", servo_xyz_angle_axis_translation[0], servo_xyz_angle_axis_translation[1],
        			servo_xyz_angle_axis_translation[2]);
        		*/


        // sprowadzenie przyrostu do wsp. bazowych
        servo_xyz_angle_axis_rotation = servo_current_end_effector_frame_with_tool_and_base_wo_offset * servo_xyz_angle_axis_rotation;
        servo_xyz_angle_axis_translation = servo_current_end_effector_frame_with_tool_and_base_wo_offset * servo_xyz_angle_axis_translation;
        /*
        		printf("b rx: %f, ry: %f, rz: %f\n", servo_xyz_angle_axis_translation[0], servo_xyz_angle_axis_translation[1],
        			servo_xyz_angle_axis_translation[2]);
        		*/
        // wyliczenie predkosci i przyspieszenia
        for (int i=0; i<6; i++)
        {
            servo_real_kartez_vel[i] = servo_real_kartez_pos[i] - rkpminusone[i];
            servo_real_kartez_acc[i] = servo_real_kartez_pos[i] - 2*rkpminusone[i] + rkpminustwo[i];
            rkpminustwo[i] = rkpminusone[i];
            rkpminusone[i] = servo_real_kartez_pos[i];
        }

#endif

        // zapisanie wartosci rzeczywistej dla readera
        rb_obj->lock_mutex();

        for (int i=0; i<6; i++)
        {
            rb_obj->step_data.real_kartez_position[i] = servo_real_kartez_pos[i];
            rb_obj->step_data.real_kartez_vel[i] = servo_real_kartez_vel[i];
            rb_obj->step_data.real_kartez_acc[i] = servo_real_kartez_acc[i];
        }

        rb_obj->unlock_mutex();

        // Jesli obliczenia zwiazane z baza maja byc wykonane.
        if (get_current_kinematic_model()->global_frame_computations)
        {
            Homog_matrix tmp_eem(servo_current_frame_wo_tool);
            get_current_kinematic_model()->global_frame_transform(tmp_eem);
            tmp_eem.get_frame_tab(servo_current_frame_wo_tool);
        }//: if

        if (((!force_sensor_configured)&&(synchronised)))
        {
            vs->force_sensor_do_configure = true;
            force_sensor_configured = true;
        }
        ;//: if
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
    }
    ;//: for
    //	printf("lala: %f\n", servo_current_joints[1]);
    copy_frame(global_current_frame_wo_tool, servo_current_frame_wo_tool);

    pthread_mutex_unlock( &edp_irp6s_effector_mutex );
}
;//: edp_irp6s_postument_track_effector::servo_joints_and_frame_actualization_and_upload


Homog_matrix edp_irp6s_postument_track_effector::return_current_frame (TRANSLATION_ENUM translation_mode)
        {// by Y
            pthread_mutex_lock( &edp_irp6s_effector_mutex );
            // przepisanie danych na zestaw lokalny dla edp_force
            // copy_frame(force_current_end_effector_frame, global_current_end_effector_frame);
            Homog_matrix return_frame(global_current_frame_wo_tool);
            pthread_mutex_unlock( &edp_irp6s_effector_mutex );

            if (translation_mode == WITHOUT_TRANSLATION)
                return_frame.remove_translation();
            return return_frame;
        }

        void edp_irp6s_postument_track_effector::force_msr_upload(double *new_value)
        {// by Y wgranie globalnego zestawu danych
            pthread_mutex_lock( &force_mutex );
            for (int i=0;i<=5;i++)
            {
                prevoius_global_kartez_force_msr[i]=global_kartez_force_msr[i];
                global_kartez_force_msr[i]=new_value[i];
                // 		printf("ALARM\n");
            }
            pthread_mutex_unlock( &force_mutex );
        }



        // by Y odczytanie globalnego zestawu danych
        void edp_irp6s_postument_track_effector::force_msr_download(double *new_value, double *old_value)
        {
            pthread_mutex_lock( &force_mutex );
            for (int i=0;i<=5;i++)
            {
                if (new_value)
                    new_value[i]=global_kartez_force_msr[i];
                if (old_value)
                    old_value[i]=prevoius_global_kartez_force_msr[i];
            }
            pthread_mutex_unlock( &force_mutex );
        }
