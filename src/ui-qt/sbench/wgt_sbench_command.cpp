#include "../sbench/ui_ecp_r_sbench.h"
#include "../sbench/ui_r_sbench.h"
#include "robot/sbench/const_sbench.h"

#include "wgt_sbench_command.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

#include <QFont>

wgt_sbench_command::wgt_sbench_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		wgt_base(_widget_label, _interface, parent)
{
	ui.setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::sbench::UiRobot *>(_robot);

	// utworzenie list widgetow
	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			int k = (i + 1) % 2;

			QCheckBox *tmp_checkbox;
			tmp_checkbox = new QCheckBox(this);

			std::stringstream tmp_stringsteam;
			tmp_stringsteam << i + 1 << "-";

			switch (j)
			{
				case 0:
					tmp_stringsteam << "I";
					break;
				case 1:
					tmp_stringsteam << "II";
					break;
				case 2:
					tmp_stringsteam << "III";
					break;
				case 3:
					tmp_stringsteam << "IV";
					break;
				case 4:
					tmp_stringsteam << "V";
					break;
				case 5:
					tmp_stringsteam << "VI";
					break;
				case 6:
					tmp_stringsteam << "VII";
					break;
				case 7:
					tmp_stringsteam << "VIII";
					break;
			}

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

			ui.gridLayout->addWidget(tmp_checkbox, i, (2 * j) + k);
		//	checkBox_Vector.append(tmp_checkbox);

			docks[i][j] = tmp_checkbox;

			// unused docks are disabled
			if (((k == 0) && (i > 6)) || ((k == 1) && (j > 6))) {
				tmp_checkbox->setDisabled(true);
			}

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

//	int sum = 0;

	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			if (robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.data.get_value(i, j)) {

				font.setUnderline(true);

				pal.setColor(QPalette::WindowText, Qt::blue);
				pal.setColor(QPalette::Background, Qt::blue);
			} else {

				font.setUnderline(false);

				pal.setColor(QPalette::WindowText, Qt::black);
				pal.setColor(QPalette::Background, Qt::black);
			}
			docks[i][j]->setFont(font);
			docks[i][j]->setPalette(pal);
			//	checkBox_Vector[sum]->setFont(font);
			//	checkBox_Vector[sum]->setPalette(pal);
			//	sum++;
		}
	}

}

void wgt_sbench_command::on_pushButton_read_and_copy_clicked()
{
	on_pushButton_read_clicked();
//	int sum = 0;

	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			docks[i][j]->setChecked(robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.data.get_value(i, j));
			//checkBox_Vector[sum]->setChecked(robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.data.get_value(i, j));
			//sum++;
		}
	}

}

void wgt_sbench_command::on_pushButton_clear_clicked()
{

	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			docks[i][j]->setChecked(false);

		}
	}
	/*
	 for (int i; i < SBENCH_MAX_EL; i++) {
	 checkBox_Vector[i]->setChecked(false);
	 }
	 */
}

void wgt_sbench_command::on_pushButton_execute_clicked()
{
//	int sum = 0;

	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			robot->ui_ecp_robot->the_robot->sbench_command_data_port.data.set_value(i, j, docks[i][j]->isChecked());
			//	sum++;
		}
	}

	robot->ui_ecp_robot->the_robot->sbench_command_data_port.set();

	robot->ui_ecp_robot->execute_motion();

	on_pushButton_read_clicked();
}

