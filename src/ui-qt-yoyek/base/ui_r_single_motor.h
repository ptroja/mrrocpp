// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SINGLE_MOTOR_H
#define __UI_R_SINGLE_MOTOR_H

#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "robot/conveyor/const_conveyor.h"

class wgt_conveyor_move;

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;

}
namespace single_motor {
class EcpRobot;

class UiRobot : public common::UiRobot
{
private:

public:

	double current_pos[1];// pozycja biezaca
	double desired_pos[1]; // pozycja zadana

	EcpRobot *ui_ecp_robot;
	UiRobot(common::Interface& _interface, const std::string & edp_section_name, const std::string & ecp_section_name, lib::robot_name_t _robot_name, int _number_of_servos, const std::string & _activation_string);

};

}
} //namespace ui
} //namespace mrrocpp

#endif

