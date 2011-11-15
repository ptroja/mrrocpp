#include "../sbench/ui_ecp_r_sbench.h"
#include "../sbench/ui_r_sbench.h"
#include "robot/sbench/const_sbench.h"

#include "wgt_sbench_command.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

#include <QFont>

#define SBENCH_MAX_ROW 8
#define SBENCH_MAX_COL 8
#define SBENCH_MAX_EL 64

wgt_sbench_command::wgt_sbench_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		wgt_base(_widget_label, _interface, parent)
{
	ui.setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::sbench::UiRobot *>(_robot);

	// utworzenie list widgetow
	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			QCheckBox *tmp_checkbox;
			tmp_checkbox = new QCheckBox(this);

			std::stringstream tmp_stringsteam;
			tmp_stringsteam << i * SBENCH_MAX_COL + j;

			tmp_checkbox->setText(tmp_stringsteam.str().c_str());
			/*
			 QPalette pal;
			 pal.setColor(QPalette::WindowText, Qt::blue);
			 pal.setColor(QPalette::Background, Qt::blue);

			 QFont font;
			 font.setBold(true);
			 tmp_checkbox->setPalette(pal);
			 tmp_checkbox->setFont(font);
			 tmp_checkbox->repaint();
			 tmp_checkbox->update();
			 */
			int k = i % 2;
			ui.gridLayout->addWidget(tmp_checkbox, i, (2 * j) + k);
			checkBox_Vector.append(tmp_checkbox);
		}
	}

}

wgt_sbench_command::~wgt_sbench_command()
{

}

void wgt_sbench_command::on_pushButton_read_clicked()
{
	robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.set_request();
	robot->ui_ecp_robot->execute_motion();
	robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.get();

	QFont font;
	QPalette pal;

	for (int i = 0; i < SBENCH_MAX_EL; i++) {
		if (robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.data[i]) {

			font.setUnderline(true);

			pal.setColor(QPalette::WindowText, Qt::blue);
			pal.setColor(QPalette::Background, Qt::blue);
		} else {

			font.setUnderline(false);

			pal.setColor(QPalette::WindowText, Qt::black);
			pal.setColor(QPalette::Background, Qt::black);
		}

		checkBox_Vector[i]->setFont(font);
		checkBox_Vector[i]->setPalette(pal);
	}

}

void wgt_sbench_command::on_pushButton_read_and_copy_clicked()
{
	on_pushButton_read_clicked();

	for (int i; i < SBENCH_MAX_EL; i++) {
		checkBox_Vector[i]->setChecked(robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.data[i]);
	}

}

void wgt_sbench_command::on_pushButton_clear_clicked()
{
	for (int i; i < SBENCH_MAX_EL; i++) {
		checkBox_Vector[i]->setChecked(false);
	}
}

void wgt_sbench_command::on_pushButton_execute_clicked()
{

	for (int i = 0; i < SBENCH_MAX_EL; i++) {

		robot->ui_ecp_robot->the_robot->sbench_command_data_port.data[i] = checkBox_Vector[i]->isChecked();
	}

	robot->ui_ecp_robot->the_robot->sbench_command_data_port.set();

	robot->ui_ecp_robot->execute_motion();

	on_pushButton_read_clicked();
}

