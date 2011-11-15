#ifndef WGT_SBENCH_COMMAND_H
#define WGT_SBENCH_COMMAND_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_sbench_command.h"
#include "../base/wgt_base.h"
#include <QTimer>
#include <QCheckBox>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
namespace sbench {
class UiRobot;
const std::string WGT_SBENCH_COMMAND = "WGT_SBENCH_COMMAND";
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

private:
	Ui::wgt_sbench_commandClass ui;
	mrrocpp::ui::sbench::UiRobot* robot;

	QVector <QCheckBox*> checkBox_Vector;

private slots:

	void on_pushButton_read_clicked();
	void on_pushButton_read_and_copy_clicked();
	void on_pushButton_clear_clicked();
	void on_pushButton_execute_clicked();

};

#endif // WGT_SBENCH_COMMAND_H
