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
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_sbench_command.h"
#include "../base/wgt_base.h"
#include <QTimer>
#include <QCheckBox>

#define SBENCH_MAX_ROW 8
#define SBENCH_MAX_COL 8
#define SBENCH_MAX_EL 64

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

class wgt_sbench_command : public wgt_base
{
	Q_OBJECT

public:
	wgt_sbench_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);

	~wgt_sbench_command();

	QCheckBox* docks[SBENCH_MAX_ROW][SBENCH_MAX_COL];
	Ui::wgt_sbench_commandClass ui;
	mrrocpp::ui::sbench::UiRobot* robot;

private:

	void showEvent(QShowEvent * event);
	virtual void init() = 0;
	virtual void execute() = 0;
	virtual void read_and_copy() = 0;

private slots:

	void on_pushButton_read_clicked();
	void on_pushButton_read_and_copy_clicked();
	void on_pushButton_clear_clicked();
	void on_pushButton_execute_clicked();

};

#endif // WGT_SBENCH_COMMAND_H
