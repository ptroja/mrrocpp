#ifndef WGT_SBENCH_VOLTAGE_COMMAND_H
#define WGT_SBENCH_VOLTAGE_COMMAND_H

/*!
 * @file
 * @brief File contains wgt_sbench_voltage_command class declaration for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "wgt_sbench_command.h"

namespace mrrocpp {
namespace ui {
namespace sbench {
/*!
 * @brief window identificator
 */
const std::string WGT_SBENCH_VOLTAGE_COMMAND = "WGT_SBENCH_VOLTAGE_COMMAND";
}
}
}

/*!
 * @class
 * @brief class of sbench voltage (power supply relay control) command window
 * @author yoyek
 *
 *  @ingroup sbench
 */
class wgt_sbench_voltage_command : public wgt_sbench_command
{
	Q_OBJECT

public:
	/**
	 * @brief constructor
	 * @param _widget_label widget label
	 * @param _interface Interface object reference
	 * @param _robot UiRobot object pointer
	 * @param parent pointer to parent widget
	 */
	wgt_sbench_voltage_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);

	/**
	 * @brief destructor
	 */
	~wgt_sbench_voltage_command();

	/**
	 * @brief gathers information about robot state
	 */
	void init();

	/**
	 * @brief executes the command (i.e. sends it to robot)
	 */
	void execute();

	/**
	 * @brief sets the window state taking the info about robot state
	 */
	void read_and_copy();
};

#endif
