// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_COMMON_H
#define _UI_ECP_R_COMMON_H

#include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"

#include "base/ecp/ecp_robot.h"

namespace mrrocpp {
namespace ui {
namespace common {

// ---------------------------------------------------------------
class EcpRobot
{
protected:
	// Klasa do obslugi robotow irp6 z poziomu UI

	// Dopuszczalne przyrosty polozenia w pojedynczym kroku [2ms] przy ruchach
	// recznych dla roznych wspolrzednych
	double MOTOR_STEP; // Przyrost kata obrotu walu silnika [rad]
	double MOTOR_GRIPPER_STEP;
	double JOINT_ANGULAR_STEP; // Przyrost kata obrotu w przegubie obrotowym [rad]
	double JOINT_LINEAR_STEP; // Przyrost liniowy w przegubach posuwistych [mm]
	double JOINT_GRIPPER_STEP;
	double END_EFFECTOR_LINEAR_STEP; // Przyrost wspolrzednej polozenia koncowki [mm]
	double END_EFFECTOR_ANGULAR_STEP; // Przyrost wspolrzednej orientacji koncowki [rad]
	double END_EFFECTOR_GRIPPER_STEP; // Przyrost wspolrzednej orientacji koncowki [rad]

	double desired_position[lib::MAX_SERVOS_NR]; // polozenie zadane
	double current_position[lib::MAX_SERVOS_NR]; // polozenie aktualne

public:
	ecp::common::robot::ecp_robot *ecp;

	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name); // Konstruktor

	virtual ~EcpRobot();

	virtual void execute_motion(void);
	// virtual void set_edp_master_pid ( pid_t edppid ) {EDP_Pid = edppid;};
	// Przekazanie identyfikatora procesu EDP
	// virtual void synchronise ( void ); // Zlecenie synchronizacji robota
	// virtual bool is_synchronised( void ) { return synchronised;};

	virtual void set_desired_position(const double des_position[lib::MAX_SERVOS_NR]);
	// Przepisanie polozen zadanych
	// do tablicy desired_position[]
	virtual void get_current_position(double c_position[lib::MAX_SERVOS_NR]); // Pobranie aktualnych polozen

	// by Y - do odczytu stanu poczatkowego robota
	void get_controller_state(lib::controller_state_t & robot_controller_initial_state_l);

	// Zlecenie ruchu
	void set_kinematic(uint8_t kinematic_model_no);
	void set_servo_algorithm(uint8_t algorithm_no[lib::MAX_SERVOS_NR], uint8_t parameters_no[lib::MAX_SERVOS_NR]);

	// Odczyt polozenia
	void read_motors(double current_position[lib::MAX_SERVOS_NR]);
	void read_joints(double current_position[lib::MAX_SERVOS_NR]);
	void get_kinematic(uint8_t* kinematic_model_no);
	void get_servo_algorithm(uint8_t algorithm_no[lib::MAX_SERVOS_NR], uint8_t parameters_no[lib::MAX_SERVOS_NR]);

};

}
} //namespace ui
} //namespace mrrocpp
#endif
