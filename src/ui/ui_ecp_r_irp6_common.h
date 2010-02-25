
// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_MANIP_H
#define _UI_ECP_R_MANIP_H

#include "ecp/common/ecp_robot.h"

#include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "lib/mrmath/mrmath.h"



// ---------------------------------------------------------------
class ui_irp6_common_robot
{
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

	double desired_position[MAX_SERVOS_NR]; // polozenie zadane
	double current_position[MAX_SERVOS_NR]; // polozenie aktualne

public:
	ecp::common::ecp_robot *ecp;

	// ecp_buffer ui_edp_package; // by Y
	ui_irp6_common_robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name); // Konstruktor

	virtual ~ui_irp6_common_robot();

	virtual void execute_motion(void);
	// virtual void set_edp_master_pid ( pid_t edppid ) {EDP_Pid = edppid;};
	// Przekazanie identyfikatora procesu EDP
	// virtual void synchronise ( void ); // Zlecenie synchronizacji robota
	// virtual bool is_synchronised( void ) { return synchronised;};

	virtual void set_desired_position(double des_position[MAX_SERVOS_NR]);
	// Przepisanie polozen zadanych
	// do tablicy desired_position[]
	virtual void get_current_position(double c_position[MAX_SERVOS_NR]); // Pobranie aktualnych polozen

	// by Y - do odczytu stanu poczatkowego robota
	void get_controller_state(lib::controller_state_t & robot_controller_initial_state_l);

	// Zlecenie ruchu
	void move_motors(const double final_position[MAX_SERVOS_NR]);
	void move_joints(const double final_position[MAX_SERVOS_NR]);
	void move_xyz_euler_zyz(const double final_position[7]);
	void move_xyz_angle_axis(const double final_position[7]);
	void move_xyz_angle_axis_relative(const double position_increment[7]);
	void set_tool_xyz_angle_axis(const lib::Xyz_Angle_Axis_vector &tool_vector);
	void set_tool_xyz_euler_zyz(const lib::Xyz_Euler_Zyz_vector &tool_vector);
	void set_kinematic(uint8_t kinematic_model_no);
	void set_servo_algorithm(uint8_t algorithm_no[MAX_SERVOS_NR], uint8_t parameters_no[MAX_SERVOS_NR]);

	// Odczyt polozenia
	void read_motors(double current_position[MAX_SERVOS_NR]);
	void read_joints(double current_position[MAX_SERVOS_NR]);
	void read_xyz_euler_zyz(double current_position[7]);
	void read_xyz_angle_axis(double current_position[7]);
	void read_tool_xyz_angle_axis(lib::Xyz_Angle_Axis_vector & tool_vector);
	void read_tool_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector &tool_vector);
	void get_kinematic(uint8_t* kinematic_model_no);
	void get_servo_algorithm(uint8_t algorithm_no[MAX_SERVOS_NR], uint8_t parameters_no[MAX_SERVOS_NR]);

};
#endif
