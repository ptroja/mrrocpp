#ifndef __UI_R_SBENCH_H
#define __UI_R_SBENCH_H

/*!
 * @file
 * @brief File contains UiRobot class declaration for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <QObject>
#include <QMenu>
#include "../base/ui.h"
#include "../base/ui_robot.h"

#include "wgt_sbench_voltage_command.h"
#include "wgt_sbench_preasure_command.h"

namespace Ui {
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace sbench {

// forward declarations
class EcpRobot;

/*!
 * @class
 * @brief class of sbench UiRobot
 * @author yoyek
 *
 *  @ingroup sbench
 */
class UiRobot : public common::UiRobot
{
	Q_OBJECT

public:

	/**
	 * @brief pointer to EcpRobot
	 */
	EcpRobot *ui_ecp_robot;

	/**
	 * @brief constructor
	 * @param _interface Interface object reference
	 */
	UiRobot(common::Interface& _interface);

	/**
	 * @brief created ui_ecp_robot object
	 */
	void create_ui_ecp_robot();

	/**
	 * @brief activates and deactivates particular menu items
	 * it takes into account interface state
	 */
	void manage_interface();

	/**
	 * @brief deletes ui_ecp_robot onbject
	 */
	void delete_ui_ecp_robot();

	/**
	 * @brief zeros ui_ecp_robot pointer
	 */
	void null_ui_ecp_robot();

	/**
	 * @brief synchronizes robot
	 */
	void synchronise();

	/**
	 * @brief gets_controller_state (edp controller)
	 */
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);

	/**
	 * @brief buils menu bar
	 */
	void setup_menubar();

private:

	/**
	 * @brief voltage_command_window menu action
	 */
	QAction *action_voltage_command;

	/**
	 * @brief preasure_command_window menu action
	 */
	QAction *action_preasure_command;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

