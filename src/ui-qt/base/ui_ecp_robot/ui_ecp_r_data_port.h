// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_DATA_PORT_H
#define _UI_ECP_R_DATA_PORT_H

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_robot.h"

#include "../ui.h"
#include "../ui_robot.h"
#include "../interface.h"

namespace mrrocpp {
namespace ui {
namespace common {

template <typename ECP_ROBOT_T>
class _EcpRobotDataPort
{
public:
	//! Type of the template instance itself
	typedef _EcpRobotDataPort <ECP_ROBOT_T> EcpRobotDataPort;

	//! Type of the robot class
	typedef ECP_ROBOT_T robot_t;

	UiRobot& ui_robot;

	boost::shared_ptr <ECP_ROBOT_T> the_robot;

	// do odczytu stanu poczatkowego robota
	void get_controller_state(lib::controller_state_t & robot_controller_initial_state_l)
	{
		the_robot->ecp_command.instruction_type = lib::GET;
		the_robot->ecp_command.get_type = CONTROLLER_STATE_DEFINITION;

		the_robot->execute_motion();

		robot_controller_initial_state_l = the_robot->reply_package.controller_state;
		the_robot->synchronised = robot_controller_initial_state_l.is_synchronised;
	}

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
	virtual void execute_motion(void)
	{
		//printf("EcpRobotDataPort::execute_motion by pthread_t = %lu\n", pthread_self());

		ui_robot.interface.set_ui_state_notification(UI_N_COMMUNICATION);

		the_robot->create_command();

		the_robot->execute_motion();

		the_robot->get_reply();
	}

	_EcpRobotDataPort(UiRobot& _ui_robot) :
		ui_robot(_ui_robot)
	{
	}

	virtual ~_EcpRobotDataPort()
	{
	}
};

typedef _EcpRobotDataPort <ecp::common::robot::ecp_robot> EcpRobotDataPort;

}
} //namespace ui
} //namespace mrrocpp

#endif
