// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_IRP6_COMMON_H
#define _UI_ECP_R_IRP6_COMMON_H

#include "ecp/common/ecp_robot.h"

#include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"


// ---------------------------------------------------------------
class ui_common_robot {
// Klasa do obslugi robotow irp6 z poziomu UI

// Dopuszczalne przyrosty polozenia w pojedynczym kroku [2ms] przy ruchach
// recznych dla roznych wspolrzednych
 double MOTOR_STEP;                // Przyrost kata obrotu walu silnika [rad]
 double MOTOR_GRIPPER_STEP;
 double JOINT_ANGULAR_STEP;        // Przyrost kata obrotu w przegubie obrotowym [rad]
 double JOINT_LINEAR_STEP;         // Przyrost liniowy w przegubach posuwistych [mm]
 double JOINT_GRIPPER_STEP;
 double END_EFFECTOR_LINEAR_STEP;  // Przyrost wspolrzednej polozenia koncowki [mm]
 double END_EFFECTOR_ANGULAR_STEP; // Przyrost wspolrzednej orientacji koncowki [rad]
 double END_EFFECTOR_GRIPPER_STEP; // Przyrost wspolrzednej orientacji koncowki [rad]

 double desired_position[IRP6_ON_TRACK_NUM_OF_SERVOS]; // polozenie zadane
 double current_position[IRP6_ON_TRACK_NUM_OF_SERVOS]; // polozenie aktualne

 public:
	 ecp::common::ecp_robot *ecp;

// ecp_buffer ui_edp_package; // by Y
  ui_common_robot (lib::configurator &_config, lib::sr_ecp* _sr_ecp_msg, ROBOT_ENUM _robot_name); // Konstruktor

  virtual ~ui_common_robot();

  virtual void execute_motion ( void );
// virtual void set_edp_master_pid ( pid_t edppid ) {EDP_Pid = edppid;};
                                     // Przekazanie identyfikatora procesu EDP
  // virtual void synchronise ( void ); // Zlecenie synchronizacji robota
  // virtual bool is_synchronised( void ) { return synchronised;};

  virtual void set_desired_position ( double des_position[IRP6_ON_TRACK_NUM_OF_SERVOS] );
                                     // Przepisanie polozen zadanych
                                     // do tablicy desired_position[]
  virtual void get_current_position ( double c_position[IRP6_ON_TRACK_NUM_OF_SERVOS] );  // Pobranie aktualnych polozen

  // by Y - do odczytu stanu poczatkowego robota
  bool get_controller_state (controller_state_t* robot_controller_initial_state_l);

  // Zlecenie ruchu
  bool move_motors ( double final_position[IRP6_ON_TRACK_NUM_OF_SERVOS] );
  bool move_joints ( double final_position[IRP6_ON_TRACK_NUM_OF_SERVOS] );
  bool move_xyz_euler_zyz ( double final_position[7] );
  bool move_xyz_angle_axis ( double final_position[7] );
  bool set_tool_xyz_angle_axis ( double tool_vector[6] );
  bool set_tool_xyz_euler_zyz ( double tool_vector[6] );
  bool set_kinematic (BYTE kinematic_model_no);
  bool set_servo_algorithm (BYTE algorithm_no[IRP6_ON_TRACK_NUM_OF_SERVOS],
  BYTE parameters_no[IRP6_ON_TRACK_NUM_OF_SERVOS] );

  // Odczyt polozenia
  bool read_motors ( double current_position[IRP6_ON_TRACK_NUM_OF_SERVOS] );
  bool read_joints ( double current_position[IRP6_ON_TRACK_NUM_OF_SERVOS] );
  bool read_xyz_euler_zyz ( double current_position[7] );
  bool read_xyz_angle_axis ( double current_position[7] );
  bool read_tool_xyz_angle_axis ( double tool_vector[6] );
  bool read_tool_xyz_euler_zyz ( double tool_vector[6] );
  bool get_kinematic (BYTE* kinematic_model_no);
  bool get_servo_algorithm ( BYTE algorithm_no[IRP6_ON_TRACK_NUM_OF_SERVOS],
  BYTE parameters_no[IRP6_ON_TRACK_NUM_OF_SERVOS]);

};
#endif
