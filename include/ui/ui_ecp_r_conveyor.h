// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_CONVEYOR_H
#define _UI_ECP_R_CONVEYOR_H



#include "ecp/conveyor/ecp_local.h"

#include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"



// ---------------------------------------------------------------
class ui_conveyor_robot: public ecp::conveyor::ecp_conveyor_robot {
// Klasa do obslugi robota conveyor (sztywnego) z poziomu UI

// pid_t EDP_Pid; // identyfikator procesu driver'a edp_m // by Y
// bool synchronised; // Flaga ustawiana po synchronizacji robota // by Y

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
// ecp_buffer ui_edp_package; // by Y
  ui_conveyor_robot (lib::configurator &_config, lib::sr_ecp* _sr_ecp_msg); // Konstruktor

  virtual void execute_motion ( void );
// virtual void set_edp_master_pid ( pid_t edppid ) {EDP_Pid = edppid;};
                                     // Przekazanie identyfikatora procesu EDP
  // virtual void synchronise ( void ); // Zlecenie synchronizacji robota
  // virtual bool is_synchronised( void ) { return synchronised;};

  virtual void set_desired_position ( double des_position[CONVEYOR_NUM_OF_SERVOS] );
                                     // Przepisanie polozen zadanych
                                     // do tablicy desired_position[]
  virtual void get_current_position ( double c_position[CONVEYOR_NUM_OF_SERVOS] );  // Pobranie aktualnych polozen


  // by Y - do odczytu stanu poczatkowego robota
  bool get_controller_state (controller_state_t* robot_controller_initial_state_l);

  // Zlecenie ruchu
  bool move_motors ( double final_position[CONVEYOR_NUM_OF_SERVOS] );
  bool move_joints ( double final_position[CONVEYOR_NUM_OF_SERVOS] );
  bool set_kinematic (BYTE kinematic_model_no);
  bool set_servo_algorithm (BYTE algorithm_no[CONVEYOR_NUM_OF_SERVOS],
	 BYTE parameters_no[CONVEYOR_NUM_OF_SERVOS] );

  // Odczyt polozenia
  bool read_motors ( double current_position[CONVEYOR_NUM_OF_SERVOS] );
  bool read_joints ( double current_position[CONVEYOR_NUM_OF_SERVOS] );
  bool get_kinematic (BYTE* kinematic_model_no);
  bool get_servo_algorithm ( BYTE algorithm_no[CONVEYOR_NUM_OF_SERVOS],
      BYTE parameters_no[CONVEYOR_NUM_OF_SERVOS]);

  // Przekazanie identyfikatora procesu MP

}; // end: class ui_conveyor_robot



#endif
