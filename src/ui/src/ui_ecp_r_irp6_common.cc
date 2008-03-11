// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
// 
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/irp6_mechatronika/ecp_local.h"
#include "ui/ui_ecp_r_irp6_common.h"

#include <math.h>
#include "lib/mathtr.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

// ---------------------------------------------------------------
ui_common_robot::ui_common_robot (edp_state_def* _edp_state, configurator &_config, sr_ecp* sr_ecp_msg, ROBOT_ENUM _robot_name) 
{

	switch (_robot_name) {
		case ROBOT_IRP6_ON_TRACK:
			ecp = new ecp_irp6_on_track_robot(_config, sr_ecp_msg);
			break;
		case ROBOT_IRP6_POSTUMENT:
			ecp = new ecp_irp6_postument_robot(_config, sr_ecp_msg);
			break;
		case ROBOT_IRP6_MECHATRONIKA:
			ecp = new ecp_irp6_mechatronika_robot(_config, sr_ecp_msg);
			break;
		case ROBOT_SPEAKER:
		case ROBOT_CONVEYOR:
			break;
		default:
			fprintf(stderr, "ERROR: unknown robot name in ecp_robot ui_common_robot::ui_common_robot\n");
			ecp = NULL;
			break;
	}

  assert(ecp);

  // Konstruktor klasy
  ecp->EDP_command_and_reply_buffer.sr_ecp_msg = sr_ecp_msg;
  ecp->EDP_command_and_reply_buffer.instruction.rmodel.kinematic_model.kinematic_model_no = 0;   
  ecp->EDP_command_and_reply_buffer.instruction.get_type = ARM_DV; // ARM
  ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = MOTOR;
  ecp->EDP_command_and_reply_buffer.instruction.set_type = ARM_DV; // ARM
  ecp->EDP_command_and_reply_buffer.instruction.set_arm_type = MOTOR;
  ecp->EDP_command_and_reply_buffer.instruction.motion_steps = 0;
  ecp->EDP_command_and_reply_buffer.instruction.value_in_step_no = 0;

  ecp->synchronised = false;

  MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
  MOTOR_GRIPPER_STEP = 0.5;
  JOINT_ANGULAR_STEP = 0.0004; // Przyrost kata obrotu w przegubie obrotowym [rad] 
  JOINT_LINEAR_STEP = 0.00004;    // Przyrost liniowy w przegubach posuwistych [m]
  JOINT_GRIPPER_STEP =0.000004;     // Przyrost liniowy w chwytaku [m]
  END_EFFECTOR_LINEAR_STEP =  0.00002;// Przyrost wspolrzednej polozenia koncowki [m]
  END_EFFECTOR_ANGULAR_STEP = 0.0002;  // Przyrost wspolrzednej orientacji koncowki [rad]
  END_EFFECTOR_GRIPPER_STEP = 0.000005; // Przyrost wspolrzednej orientacji koncowki [rad]

};// end: ui_common_robot::ui_common_robot ()
// ---------------------------------------------------------------

// ---------------------------------------------------------------
/* // by Y - zdefiniowane w irp6_on_track_robot - przemyslec czy nie trzeba wstawic warunku na poprawnosc synchronizacji
void ui_common_robot::synchronise ( void ) { 
// Zlecenie synchronizacji robota
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SYNCHRO;
  ecp->EDP_command_and_reply_buffer.send(EDP_fd);  // Wyslanie zlecenia synchronizacji
  ecp->EDP_command_and_reply_buffer.query(EDP_fd); // Odebranie wyniku zlecenia  
  if (ecp->EDP_command_and_reply_buffer.reply_package.reply_type == SYNCHRO_OK)
    synchronised = true;
};// end: ui_common_robot::synchronise ()
*/
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// virtual  // by Y - wywalone

void ui_common_robot::execute_motion (void) {

// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
	set_ui_state_notification(UI_N_COMMUNICATION);

	 ecp->execute_motion();

}; // end: irp6_on_track_robot::execute_motion (void)
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_common_robot::set_desired_position ( double d_position[] ) {
// Przepisanie polozen zadanych do tablicy desired_position[]
  for (int j = 0; j < ecp->number_of_servos; j++) 
    desired_position[j] = d_position[j];

};// end: ui_common_robot::set_desired_position()
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_common_robot::get_current_position ( double c_position[]) {  
// Pobranie aktualnych polozen
  for (int j = 0; j < ecp->number_of_servos; j++) 
   c_position[j] = current_position[j];

};// end: ui_common_robot::get_current_position()
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// zlecenie odczytu numeru modelu kinematyki i korektora oraz numerow
// algorytmow serwo i numerow zestawow parametrow algorytmow

bool ui_common_robot::get_kinematic (BYTE* kinematic_model_no)
{

// Zlecenie odczytu numeru modelu i korektora kinematyki
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
  ecp->EDP_command_and_reply_buffer.instruction.get_type = RMODEL_DV; // RMODEL
  ecp->EDP_command_and_reply_buffer.instruction.get_rmodel_type = ARM_KINEMATIC_MODEL; // RMODEL
  execute_motion();

  *kinematic_model_no  = ecp->EDP_command_and_reply_buffer.reply_package.rmodel.kinematic_model.kinematic_model_no;   

  return true;
}


bool ui_common_robot::get_servo_algorithm ( BYTE algorithm_no[],
      BYTE parameters_no[])
{

 // Zlecenie odczytu numerow algorytmow i zestawow parametrow  
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
  ecp->EDP_command_and_reply_buffer.instruction.get_type = RMODEL_DV; // RMODEL
  ecp->EDP_command_and_reply_buffer.instruction.get_rmodel_type = SERVO_ALGORITHM; // 
  execute_motion();

  // Przepisanie aktualnych numerow algorytmow i zestawow parametrow
  memcpy (algorithm_no, ecp->EDP_command_and_reply_buffer.reply_package.rmodel.servo_algorithm.servo_algorithm_no,
	ecp->number_of_servos*sizeof(BYTE) );
  memcpy (parameters_no, ecp->EDP_command_and_reply_buffer.reply_package.rmodel.servo_algorithm.servo_parameters_no,
	ecp->number_of_servos*sizeof(BYTE) );

  return true;
}


// do odczytu stanu poczatkowego robota
bool ui_common_robot::get_controller_state (controller_state_t* robot_controller_initial_state_l)
{

	// Zlecenie odczytu numeru modelu i korektora kinematyki
	  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
	  ecp->EDP_command_and_reply_buffer.instruction.get_type = CONTROLLER_STATE_DV;
	  
	  execute_motion();
	
	  ecp->synchronised = (*robot_controller_initial_state_l).is_synchronised  = ecp->EDP_command_and_reply_buffer.reply_package.controller_state.is_synchronised;
	  (*robot_controller_initial_state_l).is_power_on  = ecp->EDP_command_and_reply_buffer.reply_package.controller_state.is_power_on;
	  (*robot_controller_initial_state_l).is_wardrobe_on  = ecp->EDP_command_and_reply_buffer.reply_package.controller_state.is_wardrobe_on;
	  (*robot_controller_initial_state_l).is_controller_card_present  = ecp->EDP_command_and_reply_buffer.reply_package.controller_state.is_controller_card_present;
	  (*robot_controller_initial_state_l).is_robot_blocked  = ecp->EDP_command_and_reply_buffer.reply_package.controller_state.is_robot_blocked;
	 return true; 
}



// ---------------------------------------------------------------
bool ui_common_robot::set_kinematic (BYTE kinematic_model_no)   { 

// zlecenie zapisu numeru modelu kinematyki i korektora oraz numerow
// algorytmow serwo i numerow zestawow parametrow algorytmow
 
// Zlecenie zapisu numeru modelu i korektora kinematyki
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET;
  ecp->EDP_command_and_reply_buffer.instruction.set_type = RMODEL_DV; // RMODEL
  ecp->EDP_command_and_reply_buffer.instruction.set_rmodel_type = ARM_KINEMATIC_MODEL; // RMODEL
  ecp->EDP_command_and_reply_buffer.instruction.get_rmodel_type = ARM_KINEMATIC_MODEL; // RMODEL

  ecp->EDP_command_and_reply_buffer.instruction.rmodel.kinematic_model.kinematic_model_no = kinematic_model_no;   

execute_motion();

  return true;
};// end: ui_common_robot::set_kinematic()
// ---------------------------------------------------------------



// ---------------------------------------------------------------
bool ui_common_robot::set_servo_algorithm (BYTE algorithm_no[],
                            BYTE parameters_no[] )   { 

 // Zlecenie zapisu numerow algorytmow i zestawow parametrow  
  // Przepisanie zadanych numerow algorytmow i zestawow parametrow
  memcpy (ecp->EDP_command_and_reply_buffer.instruction.rmodel.servo_algorithm.servo_algorithm_no, algorithm_no,
	 ecp->number_of_servos*sizeof(BYTE) );
  memcpy (ecp->EDP_command_and_reply_buffer.instruction.rmodel.servo_algorithm.servo_parameters_no, parameters_no, 
	ecp->number_of_servos*sizeof(BYTE) );
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET;
  ecp->EDP_command_and_reply_buffer.instruction.set_type = RMODEL_DV; // RMODEL
  ecp->EDP_command_and_reply_buffer.instruction.set_rmodel_type = SERVO_ALGORITHM; // 
  ecp->EDP_command_and_reply_buffer.instruction.get_rmodel_type = SERVO_ALGORITHM; // 
execute_motion();
  return true;
};// end: ui_common_robot::set_servo_algorithm()
// ---------------------------------------------------------------


// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
bool ui_common_robot::set_tool_xyz_angle_axis ( double tool_vector[6] )
{

  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET;
  ecp->EDP_command_and_reply_buffer.instruction.set_type = RMODEL_DV; // RMODEL
  ecp->EDP_command_and_reply_buffer.instruction.set_rmodel_type = TOOL_XYZ_ANGLE_AXIS;
  ecp->EDP_command_and_reply_buffer.instruction.get_rmodel_type = TOOL_XYZ_ANGLE_AXIS;
  
memcpy (ecp->EDP_command_and_reply_buffer.instruction.rmodel.tool_coordinate_def.tool_coordinates, tool_vector, 6*sizeof(double) );
execute_motion();

    return true;
}
// ---------------------------------------------------------------


// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
bool ui_common_robot::set_tool_xyz_euler_zyz ( double tool_vector[6] )
{

  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET;
  ecp->EDP_command_and_reply_buffer.instruction.set_type = RMODEL_DV; // RMODEL
  ecp->EDP_command_and_reply_buffer.instruction.set_rmodel_type = TOOL_XYZ_EULER_ZYZ;
  ecp->EDP_command_and_reply_buffer.instruction.get_rmodel_type = TOOL_XYZ_EULER_ZYZ;
  
memcpy (ecp->EDP_command_and_reply_buffer.instruction.rmodel.tool_coordinate_def.tool_coordinates, tool_vector, 6*sizeof(double) );
execute_motion();
   
  return true;
}
// ---------------------------------------------------------------



// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
bool ui_common_robot::read_tool_xyz_angle_axis ( double tool_vector[6] )
{

  	// Zlecenie odczytu numeru modelu i korektora kinematyki
	  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
	  ecp->EDP_command_and_reply_buffer.instruction.get_type = RMODEL_DV; // RMODEL
	  ecp->EDP_command_and_reply_buffer.instruction.set_rmodel_type = TOOL_XYZ_ANGLE_AXIS;
	  ecp->EDP_command_and_reply_buffer.instruction.get_rmodel_type = TOOL_XYZ_ANGLE_AXIS;
	  
	  execute_motion();
	
	memcpy (tool_vector, ecp->EDP_command_and_reply_buffer.reply_package.rmodel.tool_coordinate_def.tool_coordinates, 6*sizeof(double) );
 
    return true;
}
// ---------------------------------------------------------------


// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
bool ui_common_robot::read_tool_xyz_euler_zyz ( double tool_vector[6] )
{
  
  	// Zlecenie odczytu numeru modelu i korektora kinematyki
	  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
	  ecp->EDP_command_and_reply_buffer.instruction.get_type = RMODEL_DV; // RMODEL
	  ecp->EDP_command_and_reply_buffer.instruction.set_rmodel_type = TOOL_XYZ_EULER_ZYZ;
	  ecp->EDP_command_and_reply_buffer.instruction.get_rmodel_type = TOOL_XYZ_EULER_ZYZ;
	  
	  execute_motion();
	
	memcpy (tool_vector, ecp->EDP_command_and_reply_buffer.reply_package.rmodel.tool_coordinate_def.tool_coordinates, 6*sizeof(double) );
 
    return true;
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
bool ui_common_robot::move_motors ( double final_position[] ) { 
// Zlecenie wykonania makrokroku ruchu zadanego dla walow silnikow
  int nr_of_steps, nr_ang, nr_grip; // Liczba krokow 
  double max_inc=0.0, max_inc_grip = 0.0,  temp = 0.0; // Zmienne pomocnicze

/*
	if (is_synchronised()) 
		printf("zsynchronizowany move motors\n");
	else 
		printf("niezsynchronizowany move motors\n");		
	*/	
  if (ecp->is_synchronised()) {  // Robot zsynchronizowany
    // Odczyt aktualnego polozenia
//   	printf("is synchronised przed read motors\n");
    if (!read_motors(current_position)) {
  //   printf("przyslowiowa p... mokra \n");
      return false;
      }
    for (int j = 0; j < ecp->number_of_servos; j++) {
      temp = fabs(final_position[j] - current_position[j]);
	  if ( j == (ecp->number_of_servos-1) ) // gripper
	      max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
	  else
	      max_inc = (max_inc > temp) ? max_inc : temp;
    }
    nr_ang = (int) ceil(max_inc / MOTOR_STEP);
	nr_grip = (int) ceil(max_inc_grip / MOTOR_GRIPPER_STEP);
	nr_of_steps = (nr_ang > nr_grip) ? nr_ang : nr_grip;
//  printf("is synchronised za read motors: nr of steps %d\n", nr_of_steps);
    // Parametry zlecenia ruchu i odczytu polozenia
    ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET_GET;
    ecp->EDP_command_and_reply_buffer.instruction.motion_type = ABSOLUTE;
  }
  else {  
// printf("!is_synchronised: %f \n",MOTOR_STEP);
  // Robot niezsynchroniozowany
      for (int j = 0; j < ecp->number_of_servos; j++) {
      temp = fabs(final_position[j]);
	 if ( j == (ecp->number_of_servos-1) ) // gripper
	      max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
	  else
	      max_inc = (max_inc > temp) ? max_inc : temp;
    }
    nr_ang = (int) ceil(max_inc / MOTOR_STEP);
	nr_grip = (int) ceil(max_inc_grip / MOTOR_GRIPPER_STEP);
	nr_of_steps = (nr_ang > nr_grip) ? nr_ang : nr_grip;

    ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET;
    ecp->EDP_command_and_reply_buffer.instruction.motion_type = RELATIVE;
  }; // end: else
  ecp->EDP_command_and_reply_buffer.instruction.get_type = ARM_DV; // ARM
  ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = MOTOR;
  ecp->EDP_command_and_reply_buffer.instruction.set_type = ARM_DV; // ARM
  ecp->EDP_command_and_reply_buffer.instruction.set_arm_type = MOTOR;
  ecp->EDP_command_and_reply_buffer.instruction.motion_steps = nr_of_steps;
  ecp->EDP_command_and_reply_buffer.instruction.value_in_step_no = nr_of_steps;

  if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
    return true; 
  for (int j = 0; j < ecp->number_of_servos; j++) {
    ecp->EDP_command_and_reply_buffer.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];
    /*
    printf("ui_ecp_aa: %f, %f, %f, %f, %f, %f, %f, %d\n", final_position[0], final_position[1], final_position[2], final_position[3],
    		 final_position[4], final_position[5], final_position[6], ecp->EDP_command_and_reply_buffer.instruction.motion_steps);
    */
// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp->EDP_command_and_reply_buffer.instruction.motion_steps, ecp->EDP_command_and_reply_buffer.instruction.value_in_step_no, ecp->EDP_command_and_reply_buffer.instruction.arm.pf_def.arm_coordinates[1]);
 }

 execute_motion();

  if (ecp->is_synchronised()) 
    for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
      current_position[j] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];

  return true;
};// end: ui_common_robot::move_motors()
// ---------------------------------------------------------------

// ---------------------------------------------------------------
bool ui_common_robot::move_joints (double final_position[] ) { 
// Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych
  int nr_of_steps; // Liczba krokow 
  int nr_ang, nr_grip, nr_lin;
  double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0  , temp = 0.0; // Zmienne pomocnicze
  int j;

  max_inc_ang = max_inc_lin = 0.0;

  // Odczyt aktualnego polozenia
  if (!read_joints(current_position)) 
    return false;
    
  for (j = 0; j < ecp->number_of_servos; j++) {
    temp = fabs(final_position[j] - current_position[j]);
    if ( ecp->robot_name == ROBOT_IRP6_ON_TRACK && j == 0 )  // tor
      max_inc_lin = (max_inc_lin > temp) ? max_inc_lin : temp;
    else  if ( j == ecp->number_of_servos )  // gripper
      max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
    else 
      max_inc_ang = (max_inc_ang > temp) ? max_inc_ang : temp;
  }

  nr_ang = (int) ceil(max_inc_ang / JOINT_ANGULAR_STEP);
  nr_lin = (int) ceil(max_inc_lin / JOINT_LINEAR_STEP);
  nr_grip = (int) ceil(max_inc_grip / JOINT_GRIPPER_STEP);
  nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
  if (ecp->robot_name == ROBOT_IRP6_ON_TRACK) {
	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
  }
  nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;

// Parametry zlecenia ruchu i odczytu polozenia
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET_GET;
  ecp->EDP_command_and_reply_buffer.instruction.get_type = ARM_DV; // ARM
  ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = JOINT;
  ecp->EDP_command_and_reply_buffer.instruction.set_type = ARM_DV; // ARM
  ecp->EDP_command_and_reply_buffer.instruction.set_arm_type = JOINT;
  ecp->EDP_command_and_reply_buffer.instruction.motion_type = ABSOLUTE;
  ecp->EDP_command_and_reply_buffer.instruction.motion_steps = nr_of_steps;
  ecp->EDP_command_and_reply_buffer.instruction.value_in_step_no = nr_of_steps;

  if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
    return true; 

  for (j = 0; j < ecp->number_of_servos; j++) {
    ecp->EDP_command_and_reply_buffer.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];
    ecp->EDP_command_and_reply_buffer.instruction.arm.pf_def.desired_torque[j]  = final_position[j];
  }

 execute_motion();

  for (j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
    current_position[j] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];

  return true;
};// end: ui_common_robot::move_internal()
// ---------------------------------------------------------------

// ---------------------------------------------------------------
bool ui_common_robot::move_xyz_euler_zyz ( double final_position[7] ) { 
// Zlecenie wykonania makrokroku ruchu zadanego we wspolrzednych
// zewnetrznych: xyz i katy Euler'a Z-Y-Z

  int nr_of_steps; // Liczba krokow 
  int nr_ang, nr_lin, nr_grip;
  double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0 , temp_lin, temp_ang, temp_grip; // Zmienne pomocnicze
  int j;

  max_inc_ang = max_inc_lin = 0.0;   
  // Odczyt aktualnego polozenia we wsp. zewn. xyz i katy Euler'a Z-Y-Z
  if (!read_xyz_euler_zyz(current_position)) 
    return false;
  for (j = 0; j < 3; j++) {
    temp_lin = fabs(final_position[j] - current_position[j]);
    temp_ang = fabs(final_position[j+3] - current_position[j+3]);
    max_inc_ang = (max_inc_ang > temp_ang) ? max_inc_ang : temp_ang;
    max_inc_lin = (max_inc_lin > temp_lin) ? max_inc_lin : temp_lin;
  }

  temp_grip = fabs(final_position[6] - current_position[6]);
  max_inc_grip = temp_grip;
  
  nr_ang = (int) ceil(max_inc_ang / END_EFFECTOR_ANGULAR_STEP);
  nr_lin = (int) ceil(max_inc_lin / END_EFFECTOR_LINEAR_STEP);
  nr_grip = (int) ceil(max_inc_grip / END_EFFECTOR_GRIPPER_STEP);
  
  nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
  nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;

// Parametry zlecenia ruchu i odczytu polozenia
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET_GET;
  ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = XYZ_EULER_ZYZ;
  ecp->EDP_command_and_reply_buffer.instruction.set_type = ARM_DV; // ARM
  ecp->EDP_command_and_reply_buffer.instruction.set_arm_type = XYZ_EULER_ZYZ;
  ecp->EDP_command_and_reply_buffer.instruction.motion_type = ABSOLUTE;
  ecp->EDP_command_and_reply_buffer.instruction.motion_steps = nr_of_steps;
  ecp->EDP_command_and_reply_buffer.instruction.value_in_step_no = nr_of_steps;

// cprintf("eNOS=%u\n",ecp->EDP_command_and_reply_buffer.instruction.motion_steps); 
  if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
  {
    return true; 
 }
 
 
  for (j = 0; j < 6; j++) 
  {
    ecp->EDP_command_and_reply_buffer.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];
    }
   ecp->EDP_command_and_reply_buffer.instruction.arm.pf_def.gripper_coordinate = final_position[6];

 execute_motion();

  for (j = 0; j < 6; j++) { // Przepisanie aktualnych polozen
    current_position[j] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];
  }
  current_position[6] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.gripper_coordinate;

  return true;
};// end: ui_common_robot::move_xyz_euler_zyz()
// ---------------------------------------------------------------


bool ui_common_robot::move_xyz_angle_axis ( double final_position[7] ) 
{
	double aa_eul[6];	// tablica przechowujaca polecenie przetransformowane
				// do formy XYZ_EULER_ZYZ
	double x, y, z, alfa, kx, ky, kz;

	x = final_position[0];
	y = final_position[1];
	z = final_position[2];
	
	alfa = sqrt(final_position[3]*final_position[3]
		+final_position[4]*final_position[4]
		+final_position[5]*final_position[5]);
	
	kx = final_position[3]/alfa;
	ky = final_position[4]/alfa;
	kz = final_position[5]/alfa;
	
	Homog_matrix A(kx, ky, kz, alfa, x, y, z);
	A.get_xyz_euler_zyz(aa_eul);	// zadane polecenie w formie XYZ_EULER_ZYZ
			
	// Odczyt aktualnego polozenia we wsp. zewn. xyz i katy Euler'a Z-Y-Z
	if (!read_xyz_euler_zyz(current_position)) 
    		return false;
    	
    	// Wyznaczenie liczby krokow
    	
   	int nr_of_steps; // Liczba krokow 
  int nr_ang, nr_lin, nr_grip;
  double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0 , temp_lin, temp_ang, temp_grip; // Zmienne pomocnicze
	int i,j;	// licznik petli
	
	max_inc_ang = max_inc_lin = 0.0;   
	for (i = 0; i < 3; i++) 
	{
		temp_lin = fabs(aa_eul[i] - current_position[i]);
		temp_ang = fabs(aa_eul[i+3] - current_position[i+3]);
		if(temp_ang > max_inc_ang)
			max_inc_ang = temp_ang;
		if(temp_lin > max_inc_lin)
			max_inc_lin = temp_lin;
	}
	
	 temp_grip = fabs(final_position[6] - current_position[6]);
	  max_inc_grip = temp_grip;
	
	nr_ang = (int) ceil(max_inc_ang / END_EFFECTOR_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / END_EFFECTOR_LINEAR_STEP);
	   nr_grip = (int) ceil(max_inc_grip / END_EFFECTOR_GRIPPER_STEP);
	
	  nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	  nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;
  	
  	// Zadano ruch do aktualnej pozycji
	if (nr_of_steps < 1) 
    		return true; 
    	
    	ecp->EDP_command_and_reply_buffer.instruction.instruction_type = SET_GET;
	ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = XYZ_ANGLE_AXIS;
	ecp->EDP_command_and_reply_buffer.instruction.set_type = ARM_DV; // ARM
	ecp->EDP_command_and_reply_buffer.instruction.set_arm_type = XYZ_ANGLE_AXIS;
	ecp->EDP_command_and_reply_buffer.instruction.motion_type = ABSOLUTE;
	ecp->EDP_command_and_reply_buffer.instruction.motion_steps = nr_of_steps;
	ecp->EDP_command_and_reply_buffer.instruction.value_in_step_no = nr_of_steps;

  for (j = 0; j < 6; j++) 
  {
    ecp->EDP_command_and_reply_buffer.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];
    }

   ecp->EDP_command_and_reply_buffer.instruction.arm.pf_def.gripper_coordinate = final_position[6];
 execute_motion();

  for (j = 0; j < 6; j++) { // Przepisanie aktualnych polozen
    current_position[j] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];
  }
  current_position[6] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.gripper_coordinate;
	
return true;
};

// ---------------------------------------------------------------
bool ui_common_robot::read_motors ( double current_position[] ) { 
// Zlecenie odczytu polozenia
  int j;
// printf("poczatek read motors\n");
// Parametry zlecenia ruchu i odczytu polozenia
  ecp->EDP_command_and_reply_buffer.instruction.get_type = ARM_DV;
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
  ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = MOTOR;
 
execute_motion();
// printf("dalej za query read motors\n");
  for (j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
// { // printf("current position: %f\n",ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j]);
    current_position[j] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];
		// 			    }
// printf("koniec read motors\n");
  return true;
};// end: ui_common_robot::read_motors()
// ---------------------------------------------------------------

// ---------------------------------------------------------------
bool ui_common_robot::read_joints ( double current_position[] ) { 
// Zlecenie odczytu polozenia
  int j;
// Parametry zlecenia ruchu i odczytu polozenia
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
  ecp->EDP_command_and_reply_buffer.instruction.get_type = ARM_DV;
  ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = JOINT;
 
execute_motion();

  for (j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
    current_position[j] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];
  return true;
};// end: ui_common_robot::read_joints()
// ---------------------------------------------------------------

// ---------------------------------------------------------------
bool ui_common_robot::read_xyz_euler_zyz (double current_position[]) { 
// Zlecenie odczytu polozenia
  int j;
// Parametry zlecenia ruchu i odczytu polozenia
  ecp->EDP_command_and_reply_buffer.instruction.get_type = ARM_DV;
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
  ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = XYZ_EULER_ZYZ;
 
execute_motion();

  for (j = 0; j < 6; j++) // Przepisanie aktualnych polozen
    current_position[j] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[j];
  current_position[6] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.gripper_coordinate;

  return true;
};// end: ui_common_robot::read_external()
// ---------------------------------------------------------------


bool ui_common_robot::read_xyz_angle_axis (double current_position[]) 
{
// Pobranie aktualnego polozenia ramienia robota

  ecp->EDP_command_and_reply_buffer.instruction.get_type = ARM_DV;
  ecp->EDP_command_and_reply_buffer.instruction.instruction_type = GET;
  ecp->EDP_command_and_reply_buffer.instruction.get_arm_type = XYZ_ANGLE_AXIS;
 
execute_motion();
  
  for(int i=0; i<6; i++)
      current_position[i] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.arm_coordinates[i];
  current_position[6] = ecp->EDP_command_and_reply_buffer.reply_package.arm.pf_def.gripper_coordinate;

return true;
};
