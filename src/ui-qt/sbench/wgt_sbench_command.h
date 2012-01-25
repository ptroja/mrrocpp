#ifndef WGT_SBENCH_COMMAND_H
#define WGT_SBENCH_COMMAND_H

/*!
 * @file
 * @brief File contains wgt_sbench_command class declaration for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <QtGui/QWidget>
#include <QDockWidget>
#include <QCheckBox>

#include "ui_wgt_sbench_command.h"
#include "../base/wgt_base.h"

#include "robot/sbench/dp_sbench.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
namespace sbench {
class UiRobot;
}
}
}

/*!
 * @class
 * @brief Base class of sbench windows
 * @author yoyek
 * @ingroup sbench
 */
class wgt_sbench_command : public wgt_base
{
	Q_OBJECT

public:
	/*!
	 * @brief maximum row index
	 */
	const static int SBENCH_MAX_ROW = 8;

	/*!
	 * @brief maximum column index
	 */
	const static int SBENCH_MAX_COL = 8;

	/*!
	 * @brief maximum element index
	 */
	const static int SBENCH_MAX_EL = 64;

	/**
	 * @brief constructor
	 * @param _widget_label widget label
	 * @param _interface Interface object reference
	 * @param _robot UiRobot object pointer
	 * @param parent pointer to parent widget
	 */
	wgt_sbench_command(const QString & _widget_label, mrrocpp::ui::common::Interface & _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);

	/**
	 * @brief destructor
	 */
	~wgt_sbench_command();

	/**
	 * @brief array of QCheckBox that points to the graphical interface widgents
	 *
	 * it is an interface between the apllication logic and user interface
	 */
	QCheckBox* docks[SBENCH_MAX_ROW][SBENCH_MAX_COL];

	/**
	 * @brief graphical interface object
	 */
	Ui::wgt_sbench_commandClass ui;

	/**
	 * @brief UiRobot object pointer
	 */
	mrrocpp::ui::sbench::UiRobot* robot;

protected:
	/**
	 * @brief gathers information about robot state
	 */
	void init();

	/**
	 * @brief set the window state taking the info about robot state
	 */
	void set(const lib::sbench::bench_state & state);

	/**
	 * @brief copy widget settings into command buffer
	 */
	void get(lib::sbench::bench_state & state);

	/*!
	 * @brief Refreshes the checkboxes and underlines depending on the given state.
	 */
	void refresh_dock_widgets(const lib::sbench::bench_state & state);

	/*!
	 * @brief Refreshes the checkboxes and underlines depending on the current state.
	 */
	void virtual reshresh_widgets() = 0;

private:

	/**
	 * @brief procedure reimplemented form base class to perform certain information
	 *
	 * it is activated when window gots focus.
	 * Here it calls the init() procedure to gather information about robot state
	 */
	void showEvent(QShowEvent * event);

	/**
	 * @brief procedure to be implemented in derived classed
	 *
	 * it executes the command (i.e. sends it to robot)
	 */
	virtual void execute() = 0;

	/**
	 * @brief procedure to be implemented in derived classed
	 *
	 * it sets the window state taking the info about robot state
	 */
	virtual void read_and_set() = 0;

private slots:

	/**
	 * @brief read widget pushButton callback
	 */
	void on_pushButton_read_clicked();

	/**
	 * @brief read_and_copy widget pushButton callback
	 */
	void on_pushButton_read_and_copy_clicked();

	/**
	 * @brief clear widget pushButton callback
	 */
	void on_pushButton_clear_clicked();

	/**
	 * @brief execute widget pushButton callback
	 */
	void on_pushButton_execute_clicked();

};

#endif // WGT_SBENCH_COMMAND_H
