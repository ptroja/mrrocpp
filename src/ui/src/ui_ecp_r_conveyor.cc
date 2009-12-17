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

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ui/ui_ecp_r_conveyor.h"

#include <math.h>
#include "lib/mrmath/mrmath.h"

// ---------------------------------------------------------------
ui_conveyor_robot::ui_conveyor_robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg)
        : robot (_config, sr_ecp_msg)
{
    // Konstruktor klasy
    ecp_command.instruction.rmodel.kinematic_model.kinematic_model_no = 0;
    ecp_command.instruction.get_type = ARM_DV; // ARM
    ecp_command.instruction.get_arm_type = lib::MOTOR;
    ecp_command.instruction.set_type = ARM_DV; // ARM
    ecp_command.instruction.set_arm_type = lib::MOTOR;
    ecp_command.instruction.motion_steps = 0;
    ecp_command.instruction.value_in_step_no = 0;

    synchronised = false;

    MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
    JOINT_LINEAR_STEP = 0.00004;     // Przyrost liniowy w przegubach posuwistych [m]

    ecp = this;
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

void ui_conveyor_robot::get_kinematic (uint8_t* kinematic_model_no)
{

    // Zlecenie odczytu numeru modelu i korektora kinematyki
    ecp_command.instruction.instruction_type = lib::GET;
    ecp_command.instruction.get_type = RMODEL_DV; // RMODEL
    ecp_command.instruction.get_rmodel_type = lib::ARM_KINEMATIC_MODEL; // RMODEL
    execute_motion();

    *kinematic_model_no  = reply_package.rmodel.kinematic_model.kinematic_model_no;
}


void ui_conveyor_robot::get_servo_algorithm ( uint8_t algorithm_no[CONVEYOR_NUM_OF_SERVOS],
        uint8_t parameters_no[CONVEYOR_NUM_OF_SERVOS])
{

    // Zlecenie odczytu numerow algorytmow i zestawow parametrow
    ecp_command.instruction.instruction_type = lib::GET;
    ecp_command.instruction.get_type = RMODEL_DV; // RMODEL
    ecp_command.instruction.get_rmodel_type = lib::SERVO_ALGORITHM; //
    execute_motion();

    // Przepisanie aktualnych numerow algorytmow i zestawow parametrow
    memcpy (algorithm_no, reply_package.rmodel.servo_algorithm.servo_algorithm_no,
            CONVEYOR_NUM_OF_SERVOS*sizeof(uint8_t) );
    memcpy (parameters_no, reply_package.rmodel.servo_algorithm.servo_parameters_no,
            CONVEYOR_NUM_OF_SERVOS*sizeof(uint8_t) );
}


// do odczytu stanu poczatkowego robota
void ui_conveyor_robot::get_controller_state (lib::controller_state_t & robot_controller_initial_state_l)
{
    // Zlecenie odczytu numeru modelu i korektora kinematyki
    ecp_command.instruction.instruction_type = lib::GET;
    ecp_command.instruction.get_type = CONTROLLER_STATE_DV;

    execute_motion();

    robot_controller_initial_state_l = reply_package.controller_state;
    synchronised = robot_controller_initial_state_l.is_synchronised;
}


// ---------------------------------------------------------------
void ui_conveyor_robot::set_kinematic (uint8_t kinematic_model_no)
{

    // zlecenie zapisu numeru modelu kinematyki i korektora oraz numerow
    // algorytmow serwo i numerow zestawow parametrow algorytmow

    // Zlecenie zapisu numeru modelu i korektora kinematyki
    ecp_command.instruction.instruction_type = lib::SET;
    ecp_command.instruction.set_type = RMODEL_DV; // RMODEL
    ecp_command.instruction.set_rmodel_type = lib::ARM_KINEMATIC_MODEL; // RMODEL
    ecp_command.instruction.get_rmodel_type = lib::ARM_KINEMATIC_MODEL; // RMODEL

    ecp_command.instruction.rmodel.kinematic_model.kinematic_model_no = kinematic_model_no;

    execute_motion();
}
// ---------------------------------------------------------------



// ---------------------------------------------------------------
void ui_conveyor_robot::set_servo_algorithm (uint8_t algorithm_no[CONVEYOR_NUM_OF_SERVOS],
        uint8_t parameters_no[CONVEYOR_NUM_OF_SERVOS] )
{
    // Zlecenie zapisu numerow algorytmow i zestawow parametrow

    // Przepisanie zadanych numerow algorytmow i zestawow parametrow
    memcpy (ecp_command.instruction.rmodel.servo_algorithm.servo_algorithm_no, algorithm_no,
            CONVEYOR_NUM_OF_SERVOS*sizeof(uint8_t) );
    memcpy (ecp_command.instruction.rmodel.servo_algorithm.servo_parameters_no, parameters_no,
            CONVEYOR_NUM_OF_SERVOS*sizeof(uint8_t) );
    ecp_command.instruction.instruction_type = lib::SET;
    ecp_command.instruction.set_type = RMODEL_DV; // RMODEL
    ecp_command.instruction.set_rmodel_type = lib::SERVO_ALGORITHM; //
    ecp_command.instruction.get_rmodel_type = lib::SERVO_ALGORITHM; //
    execute_motion();
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_conveyor_robot::move_motors ( double final_position[CONVEYOR_NUM_OF_SERVOS] )
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
        read_motors(current_position);

        for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        {
            temp = fabs(final_position[j] - current_position[j]);
            max_inc = (max_inc > temp) ? max_inc : temp;
        }
        nr_of_steps = (int) ceil(max_inc / MOTOR_STEP);


        //  printf("is synchronised za read motors: nr of steps %d\n", nr_of_steps);
        // Parametry zlecenia ruchu i odczytu polozenia
        ecp_command.instruction.instruction_type = lib::SET_GET;
        ecp_command.instruction.motion_type = lib::ABSOLUTE;
        ecp_command.instruction.interpolation_type = lib::MIM;
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

        ecp_command.instruction.instruction_type = lib::SET;
        ecp_command.instruction.motion_type = lib::RELATIVE;
        ecp_command.instruction.interpolation_type = lib::MIM;
    }
    ecp_command.instruction.get_type = ARM_DV; // ARM
    ecp_command.instruction.get_arm_type = lib::MOTOR;
    ecp_command.instruction.set_type = ARM_DV; // ARM
    ecp_command.instruction.set_arm_type = lib::MOTOR;
    ecp_command.instruction.motion_steps = nr_of_steps;
    ecp_command.instruction.value_in_step_no = nr_of_steps;

    if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
        return;
    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        ecp_command.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];

    // printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp_command.instruction.motion_steps, ecp_command.instruction.value_in_step_no, ecp_command.instruction.arm.pf_def.arm_coordinates[1]);

    execute_motion();

    if (is_synchronised())
        for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) // Przepisanie aktualnych polozen
            current_position[j] = reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_conveyor_robot::move_joints (double final_position[CONVEYOR_NUM_OF_SERVOS] )
{
    // Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych
    int nr_of_steps; // Liczba krokow

    double max_inc_lin = 0.0 , temp = 0.0; // Zmienne pomocnicze

    // Odczyt aktualnego polozenia
    read_joints(current_position);

    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
    {
        temp = fabs(final_position[j] - current_position[j]);
        max_inc_lin = (max_inc_lin > temp) ? max_inc_lin : temp;

    }
    nr_of_steps = (int) ceil(max_inc_lin / JOINT_LINEAR_STEP);

    // Parametry zlecenia ruchu i odczytu polozenia
    ecp_command.instruction.instruction_type = lib::SET_GET;
    ecp_command.instruction.get_type = ARM_DV; // ARM
    ecp_command.instruction.get_arm_type = lib::JOINT;
    ecp_command.instruction.set_type = ARM_DV; // ARM
    ecp_command.instruction.set_arm_type = lib::JOINT;
    ecp_command.instruction.motion_type = lib::ABSOLUTE;
    ecp_command.instruction.interpolation_type = lib::MIM;
    ecp_command.instruction.motion_steps = nr_of_steps;
    ecp_command.instruction.value_in_step_no = nr_of_steps;

    // cprintf("NOS=%u\n",ecp_command.instruction.motion_steps);

    if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
        return;

    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
        ecp_command.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];

    execute_motion();

    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) // Przepisanie aktualnych polozen
        current_position[j] = reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_conveyor_robot::read_motors ( double current_position[CONVEYOR_NUM_OF_SERVOS] )
{
    // Zlecenie odczytu polozenia

    // printf("poczatek read motors\n");
    // Parametry zlecenia ruchu i odczytu polozenia
    ecp_command.instruction.get_type = ARM_DV;
    ecp_command.instruction.instruction_type = lib::GET;
    ecp_command.instruction.get_arm_type = lib::MOTOR;
    ecp_command.instruction.interpolation_type = lib::MIM;

    execute_motion();

    // printf("dalej za query read motors\n");
    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) // Przepisanie aktualnych polozen
        // { // printf("current position: %f\n",reply_package.arm.pf_def.arm_coordinates[j]);
        current_position[j] = reply_package.arm.pf_def.arm_coordinates[j];
    // 			    }
    // printf("koniec read motors\n");
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_conveyor_robot::read_joints ( double current_position[CONVEYOR_NUM_OF_SERVOS] )
{
    // Zlecenie odczytu polozenia

    // Parametry zlecenia ruchu i odczytu polozenia
    ecp_command.instruction.instruction_type = lib::GET;
    ecp_command.instruction.get_type = ARM_DV;
    ecp_command.instruction.get_arm_type = lib::JOINT;
    ecp_command.instruction.interpolation_type = lib::MIM;

    execute_motion();

    for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) // Przepisanie aktualnych polozen
        current_position[j] = reply_package.arm.pf_def.arm_coordinates[j];
    //   printf("read_joints: %f\n", current_position[0]);
}
// ---------------------------------------------------------------



