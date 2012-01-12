#ifndef _UI_ECP_R_DATA_PORT_H
#define _UI_ECP_R_DATA_PORT_H

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_robot.h"

#include "../ui.h"
#include "../ui_robot.h"
#include "../interface.h"
#include "ui_ecp_r_base.h"

namespace mrrocpp {
namespace ui {
namespace common {

template <typename ECP_ROBOT_T>
class _EcpRobotDataPort : public common::EcpRobot
{
public:
	//! Type of the template instance itself
	typedef _EcpRobotDataPort <ECP_ROBOT_T> EcpRobotDataPort;

	//! Type of the robot class
	typedef ECP_ROBOT_T robot_t;

	boost::shared_ptr <ECP_ROBOT_T> the_robot;

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
	virtual void execute_motion(void)
	{
		//printf("EcpRobotDataPort::execute_motion by pthread_t = %lu\n", pthread_self());

		ui_robot.interface.set_ui_state_notification(UI_N_COMMUNICATION);

		the_robot->is_new_data = false;
		the_robot->is_new_request = false;

		the_robot->create_command();
		the_robot->finalize_data_port_command();

		//	printf("data port execute_motion\n");
		the_robot->execute_motion();

		the_robot->get_reply();
	}

	_EcpRobotDataPort(UiRobot& _ui_robot) :
			common::EcpRobot(_ui_robot)
	{
	}

	virtual ~_EcpRobotDataPort()
	{
	}
};

}
} //namespace ui
} //namespace mrrocpp

#endif
