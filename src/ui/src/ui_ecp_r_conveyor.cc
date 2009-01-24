// -------------------------------------------------------------------------
//                            ui_ecp.cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ui/ui_ecp_r_conveyor.h"

#include <math.h>
#include "lib/mathtr.h"

// ---------------------------------------------------------------
ui_conveyor_robot::ui_conveyor_robot (configurator &_config, sr_ecp* sr_ecp_msg)
        : ecp_conveyor_robot (_config, sr_ecp_msg)
{

    // Konstruktor klasy
    EDP_command_and_reply_buffer.sr_ecp_msg = sr_ecp_msg;
    EDP_command_and_reply_buffer.instruction.rmodel.kinematic_model.kinematic_model_no = 0;
    EDP_command_and_reply_buffer.instruction.get_type = ARM_DV; // ARM
    EDP_command_and_reply_buffer.instruction.get_arm_type = MOTOR;
    EDP_command_and_reply_buffer.instruction.set_type = ARM_DV; // ARM
    EDP_command_and_reply_buffer.instruction.set_arm_type = MOTOR;
    EDP_command_and_reply_buffer.instruction.motion_steps = 0;
    EDP_command_and_reply_buffer.instruction.value_in_step_no = 0;

    synchronised = false;

    MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
    JOINT_LINEAR_STEP = 0.00004;     // Przyrost liniowy w przegubach posuwistych [m]

}
// ---------------------------------------------------------------

void ui_conveyor_robot::execute_motion (void)
{

    // Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
    set_ui_state_notification(UI_N_COMMUNICATION);

    ecp_robot::execute_motion();

}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_conveyor_robot::set_desired_position ( double d_position[CONVEYOR_NUM_OF_SERVOS] )
{
    // Przepisanie polozen zadanych do tablicy desired_position[]
    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        desired_position[j] = d_position[j];

}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_conveyor_robot::get_current_position ( double c_position[CONVEYOR_NUM_OF_SERVOS])
{
    // Pobranie aktualnych polozen
    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        c_position[j] = current_position[j];

}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// zlecenie odczytu numeru modelu kinematyki i korektora oraz numerow
// algorytmow serwo i numerow zestawow parametrow algorytmow

bool ui_conveyor_robot::get_kinematic (BYTE* kinematic_model_no)
{

    // Zlecenie odczytu numeru modelu i korektora kinematyki
    EDP_command_and_reply_buffer.instruction.instruction_type = GET;
    EDP_command_and_reply_buffer.instruction.get_type = RMODEL_DV; // RMODEL
    EDP_command_and_reply_buffer.instruction.get_rmodel_type = ARM_KINEMATIC_MODEL; // RMODEL
    execute_motion();

    *kinematic_model_no  = EDP_command_and_reply_buffer.reply_package.rmodel.kinematic_model.kinematic_model_no;

    return true;
}


bool ui_conveyor_robot::get_servo_algorithm ( BYTE algorithm_no[CONVEYOR_NUM_OF_SERVOS],
        BYTE parameters_no[CONVEYOR_NUM_OF_SERVOS])
{

    // Zlecenie odczytu numerow algorytmow i zestawow parametrow
    EDP_command_and_reply_buffer.instruction.instruction_type = GET;
    EDP_command_and_reply_buffer.instruction.get_type = RMODEL_DV; // RMODEL
    EDP_command_and_reply_buffer.instruction.get_rmodel_type = SERVO_ALGORITHM; //
    execute_motion();

    // Przepisanie aktualnych numerow algorytmow i zestawow parametrow
    memcpy (algorithm_no, EDP_command_and_reply_buffer.reply_package.rmodel.servo_algorithm.servo_algorithm_no,
            CONVEYOR_NUM_OF_SERVOS*sizeof(BYTE) );
    memcpy (parameters_no, EDP_command_and_reply_buffer.reply_package.rmodel.servo_algorithm.servo_parameters_no,
            CONVEYOR_NUM_OF_SERVOS*sizeof(BYTE) );

    return true;
}


// do odczytu stanu poczatkowego robota
bool ui_conveyor_robot::get_controller_state (controller_state_t* robot_controller_initial_state_l)
{

    // Zlecenie odczytu numeru modelu i korektora kinematyki
    EDP_command_and_reply_buffer.instruction.instruction_type = GET;
    EDP_command_and_reply_buffer.instruction.get_type = CONTROLLER_STATE_DV;

    execute_motion();

    synchronised = (*robot_controller_initial_state_l).is_synchronised  = EDP_command_and_reply_buffer.reply_package.controller_state.is_synchronised;
    (*robot_controller_initial_state_l).is_power_on  = EDP_command_and_reply_buffer.reply_package.controller_state.is_power_on;
    (*robot_controller_initial_state_l).is_wardrobe_on  = EDP_command_and_reply_buffer.reply_package.controller_state.is_wardrobe_on;
    (*robot_controller_initial_state_l).is_controller_card_present  = EDP_command_and_reply_buffer.reply_package.controller_state.is_controller_card_present;
    (*robot_controller_initial_state_l).is_robot_blocked  = EDP_command_and_reply_buffer.reply_package.controller_state.is_robot_blocked;
    return true;
}



// ---------------------------------------------------------------
bool ui_conveyor_robot::set_kinematic (BYTE kinematic_model_no)
{

    // zlecenie zapisu numeru modelu kinematyki i korektora oraz numerow
    // algorytmow serwo i numerow zestawow parametrow algorytmow

    // Zlecenie zapisu numeru modelu i korektora kinematyki
    EDP_command_and_reply_buffer.instruction.instruction_type = SET;
    EDP_command_and_reply_buffer.instruction.set_type = RMODEL_DV; // RMODEL
    EDP_command_and_reply_buffer.instruction.set_rmodel_type = ARM_KINEMATIC_MODEL; // RMODEL
    EDP_command_and_reply_buffer.instruction.get_rmodel_type = ARM_KINEMATIC_MODEL; // RMODEL

    EDP_command_and_reply_buffer.instruction.rmodel.kinematic_model.kinematic_model_no = kinematic_model_no;

    execute_motion();

    return true;
}
// ---------------------------------------------------------------



// ---------------------------------------------------------------
bool ui_conveyor_robot::set_servo_algorithm (BYTE algorithm_no[CONVEYOR_NUM_OF_SERVOS],
        BYTE parameters_no[CONVEYOR_NUM_OF_SERVOS] )
{

    // Zlecenie zapisu numerow algorytmow i zestawow parametrow
    // Przepisanie zadanych numerow algorytmow i zestawow parametrow
    memcpy (EDP_command_and_reply_buffer.instruction.rmodel.servo_algorithm.servo_algorithm_no, algorithm_no,
            CONVEYOR_NUM_OF_SERVOS*sizeof(BYTE) );
    memcpy (EDP_command_and_reply_buffer.instruction.rmodel.servo_algorithm.servo_parameters_no, parameters_no,
            CONVEYOR_NUM_OF_SERVOS*sizeof(BYTE) );
    EDP_command_and_reply_buffer.instruction.instruction_type = SET;
    EDP_command_and_reply_buffer.instruction.set_type = RMODEL_DV; // RMODEL
    EDP_command_and_reply_buffer.instruction.set_rmodel_type = SERVO_ALGORITHM; //
    EDP_command_and_reply_buffer.instruction.get_rmodel_type = SERVO_ALGORITHM; //
    execute_motion();
    return true;
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
bool ui_conveyor_robot::move_motors ( double final_position[CONVEYOR_NUM_OF_SERVOS] )
{
    // Zlecenie wykonania makrokroku ruchu zadanego dla walow silnikow
    int nr_of_steps; // Liczba krokow
    double max_inc=0.0, temp = 0.0; // Zmienne pomocnicze

    /*
    	if (is_synchronised())
    		printf("zsynchronizowany move motors\n");
    	else
    		printf("niezsynchronizowany move motors\n");
    	*/
    if (is_synchronised())
    {  // Robot zsynchronizowany
        // Odczyt aktualnego polozenia
        //   	printf("is synchronised przed read motors\n");
        if (!read_motors(current_position))
        {
            //   printf("przyslowiowa p... mokra \n");
            return false;
        }
        for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        {
            temp = fabs(final_position[j] - current_position[j]);
            max_inc = (max_inc > temp) ? max_inc : temp;
        }
        nr_of_steps = (int) ceil(max_inc / MOTOR_STEP);


        //  printf("is synchronised za read motors: nr of steps %d\n", nr_of_steps);
        // Parametry zlecenia ruchu i odczytu polozenia
        EDP_command_and_reply_buffer.instruction.instruction_type = SET_GET;
        EDP_command_and_reply_buffer.instruction.motion_type = ABSOLUTE;
        EDP_command_and_reply_buffer.instruction.interpolation_type = MIM;
    }
    else
    {
        // printf("!is_synchronised: %f \n",MOTOR_STEP);
        // Robot niezsynchroniozowany
        for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        {
            temp = fabs(final_position[j]);
            max_inc = (max_inc > temp) ? max_inc : temp;
        }
        nr_of_steps = (int) ceil(max_inc / MOTOR_STEP);

        EDP_command_and_reply_buffer.instruction.instruction_type = SET;
        EDP_command_and_reply_buffer.instruction.motion_type = RELATIVE;
        EDP_command_and_reply_buffer.instruction.interpolation_type = MIM;
    }
    EDP_command_and_reply_buffer.instruction.get_type = ARM_DV; // ARM
    EDP_command_and_reply_buffer.instruction.get_arm_type = MOTOR;
    EDP_command_and_reply_buffer.instruction.set_type = ARM_DV; // ARM
    EDP_command_and_reply_buffer.instruction.set_arm_type = MOTOR;
    EDP_command_and_reply_buffer.instruction.motion_steps = nr_of_steps;
    EDP_command_and_reply_buffer.instruction.value_in_step_no = nr_of_steps;

    if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
        return true;
    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        EDP_command_and_reply_buffer.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];

    // printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",EDP_command_and_reply_buffer.instruction.motion_steps, EDP_command_and_reply_buffer.instruction.value_in_step_no, EDP_command_and_reply_buffer.instruction.arm.pf_def.arm_coordinates[1]);

    execute_motion();

    if (is_synchronised())
        for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) // Przepisanie aktualnych polozen
            current_position[j] = EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];

    return true;
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
bool ui_conveyor_robot::move_joints (double final_position[CONVEYOR_NUM_OF_SERVOS] )
{
    // Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych
    int nr_of_steps; // Liczba krokow

    double max_inc_lin = 0.0 , temp = 0.0; // Zmienne pomocnicze
    int j;


    // Odczyt aktualnego polozenia
    if (!read_joints(current_position))
        return false;

    for (j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
    {
        temp = fabs(final_position[j] - current_position[j]);
        max_inc_lin = (max_inc_lin > temp) ? max_inc_lin : temp;

    }
    nr_of_steps = (int) ceil(max_inc_lin / JOINT_LINEAR_STEP);


    // Parametry zlecenia ruchu i odczytu polozenia
    EDP_command_and_reply_buffer.instruction.instruction_type = SET_GET;
    EDP_command_and_reply_buffer.instruction.get_type = ARM_DV; // ARM
    EDP_command_and_reply_buffer.instruction.get_arm_type = JOINT;
    EDP_command_and_reply_buffer.instruction.set_type = ARM_DV; // ARM
    EDP_command_and_reply_buffer.instruction.set_arm_type = JOINT;
    EDP_command_and_reply_buffer.instruction.motion_type = ABSOLUTE;
    EDP_command_and_reply_buffer.instruction.interpolation_type = MIM;
    EDP_command_and_reply_buffer.instruction.motion_steps = nr_of_steps;
    EDP_command_and_reply_buffer.instruction.value_in_step_no = nr_of_steps;

    // cprintf("NOS=%u\n",EDP_command_and_reply_buffer.instruction.motion_steps);

    if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
        return true;

    for (j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        EDP_command_and_reply_buffer.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];

    execute_motion();

    for (j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) // Przepisanie aktualnych polozen
        current_position[j] = EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];

    return true;
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
bool ui_conveyor_robot::read_motors ( double current_position[CONVEYOR_NUM_OF_SERVOS] )
{
    // Zlecenie odczytu polozenia
    int j;
    // printf("poczatek read motors\n");
    // Parametry zlecenia ruchu i odczytu polozenia
    EDP_command_and_reply_buffer.instruction.get_type = ARM_DV;
    EDP_command_and_reply_buffer.instruction.instruction_type = GET;
    EDP_command_and_reply_buffer.instruction.get_arm_type = MOTOR;
    EDP_command_and_reply_buffer.instruction.interpolation_type = MIM;

    execute_motion();
    // printf("za query read motors\n");
    if ( EDP_command_and_reply_buffer.reply_package.reply_type == ERROR)
        return false;
    // printf("dalej za query read motors\n");
    for (j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) // Przepisanie aktualnych polozen
        // { // printf("current position: %f\n",EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j]);
        current_position[j] = EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];
    // 			    }
    // printf("koniec read motors\n");
    return true;
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
bool ui_conveyor_robot::read_joints ( double current_position[CONVEYOR_NUM_OF_SERVOS] )
{
    // Zlecenie odczytu polozenia
    int j;
    // Parametry zlecenia ruchu i odczytu polozenia
    EDP_command_and_reply_buffer.instruction.instruction_type = GET;
    EDP_command_and_reply_buffer.instruction.get_type = ARM_DV;
    EDP_command_and_reply_buffer.instruction.get_arm_type = JOINT;
    EDP_command_and_reply_buffer.instruction.interpolation_type = MIM;

    execute_motion();

    for (j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) // Przepisanie aktualnych polozen
        current_position[j] = EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];
    //   printf("read_joints: %f\n", current_position[0]);
    return true;
}
// ---------------------------------------------------------------



