// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_CONVEYOR_H
#define _UI_ECP_R_CONVEYOR_H



#include "ecp/conveyor/ecp_r_conv.h"

#include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"



// ---------------------------------------------------------------
class ui_conveyor_robot: public ecp::conveyor::robot {
// Klasa do obslugi robota conveyor (sztywnego) z poziomu UI

// Dopuszczalne przyrosty polozenia w pojedynczym kroku [2ms] przy ruchach
// recznych dla roznych wspolrzednych
 double MOTOR_STEP;                // Przyrost kata obrotu walu silnika [rad]
 double JOINT_ANGULAR_STEP;        // Przyrost kata obrotu w przegubie obrotowym [rad]
 double JOINT_LINEAR_STEP;         // Przyrost liniowy w przegubach posuwistych [mm]
 double END_EFFECTOR_LINEAR_STEP;  // Przyrost wspolrzednej polozenia koncowki [mm]
 double END_EFFECTOR_ANGULAR_STEP; // Przyrost wspolrzednej orientacji koncowki [rad]

 double desired_position[CONVEYOR_NUM_OF_SERVOS]; // polozenie zadane
 double current_position[CONVEYOR_NUM_OF_SERVOS]; // polozenie aktualne

 public:
  ui_conveyor_robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg); // Konstruktor

  virtual void execute_motion ( void );

  virtual void set_desired_position ( double des_position[CONVEYOR_NUM_OF_SERVOS] );
                                     // Przepisanie polozen zadanych
                                     // do tablicy desired_position[]
  virtual void get_current_position ( double c_position[CONVEYOR_NUM_OF_SERVOS] );
									 // Pobranie aktualnych polozen

  // by Y - do odczytu stanu poczatkowego robota
  void get_controller_state (lib::controller_state_t & robot_controller_initial_state_l);

  // Zlecenie ruchu
  void move_motors ( double final_position[CONVEYOR_NUM_OF_SERVOS] );
  void move_joints ( double final_position[CONVEYOR_NUM_OF_SERVOS] );
  void set_kinematic (uint8_t kinematic_model_no);
  void set_servo_algorithm (uint8_t algorithm_no[CONVEYOR_NUM_OF_SERVOS],
	 uint8_t parameters_no[CONVEYOR_NUM_OF_SERVOS] );

  // Odczyt polozenia
  void read_motors ( double current_position[CONVEYOR_NUM_OF_SERVOS] );
  void read_joints ( double current_position[CONVEYOR_NUM_OF_SERVOS] );
  void get_kinematic (uint8_t* kinematic_model_no);
  void get_servo_algorithm ( uint8_t algorithm_no[CONVEYOR_NUM_OF_SERVOS],
      uint8_t parameters_no[CONVEYOR_NUM_OF_SERVOS]);

  // for ui_common_robot compatibility
  ecp::common::ecp_robot *ecp;

}; // end: class ui_conveyor_robot



#endif
