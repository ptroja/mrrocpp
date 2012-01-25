/*!
 * @file
 * @brief File contains wgt_sbench_command class definition for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "../sbench/ui_ecp_r_sbench.h"
#include "../sbench/ui_r_sbench.h"
#include "robot/sbench/const_sbench.h"

#include "wgt_sbench_command.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

#include <QFont>

wgt_sbench_command::wgt_sbench_command(const QString & _widget_label, mrrocpp::ui::common::Interface & _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		wgt_base(_widget_label, _interface, parent)
{
	ui.setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::sbench::UiRobot *>(_robot);

	// utworzenie list widgetow
	for (int row = 0; row < SBENCH_MAX_ROW; ++row) {
		for (int column = 0; column < SBENCH_MAX_COL; ++column) {
			QCheckBox *tmp_checkbox = new QCheckBox(this);

			std::stringstream tmp_stringsteam;
			tmp_stringsteam << row + 1 << "-";

			switch (column)
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


			int shift = (row + 1) % 2;

			ui.gridLayout->addWidget(tmp_checkbox, SBENCH_MAX_ROW-1-row, 2*SBENCH_MAX_COL - (2 * column) - shift);
			//	checkBox_Vector.append(tmp_checkbox);

			docks[row][column] = tmp_checkbox;

			// unused docks are disabled
			if (((shift == 0) && (row > 6)) || ((shift == 1) && (column > 6))) {
				tmp_checkbox->setDisabled(true);
			}
		}
	}
}

wgt_sbench_command::~wgt_sbench_command()
{

}

void wgt_sbench_command::init()
{
	robot->ui_ecp_robot->the_robot->data_request_port.set_request();
	robot->ui_ecp_robot->execute_motion();
	robot->ui_ecp_robot->the_robot->data_request_port.get();

	reshresh_widgets();
}

void wgt_sbench_command::on_pushButton_read_clicked()
{
	init();
}

void wgt_sbench_command::on_pushButton_read_and_copy_clicked()
{
	read_and_set();
}

void wgt_sbench_command::set(const lib::sbench::bench_state & state)
{

	for (int row = 0; row < SBENCH_MAX_ROW; ++row) {
		for (int column = 0; column < SBENCH_MAX_COL; ++column) {
			docks[row][column]->setChecked(state.get_value(row + 1, column + 1));
		}
	}

}

void wgt_sbench_command::get(lib::sbench::bench_state & state)
{

	for (int row = 0; row < SBENCH_MAX_ROW; ++row) {
		for (int column = 0; column < SBENCH_MAX_COL; ++column) {
			state.set_value(row + 1, column + 1, docks[row][column]->isChecked());
		}
	}

}

void wgt_sbench_command::refresh_dock_widgets(const lib::sbench::bench_state & state)
{
	for (int row = 0; row < SBENCH_MAX_ROW; ++row) {
		for (int column = 0; column < SBENCH_MAX_COL; ++column) {
			QFont font;
			QPalette pal;

			if (state.get_value(row + 1, column + 1)) {

				font.setUnderline(true);

				pal.setColor(QPalette::WindowText, Qt::red);
				pal.setColor(QPalette::Background, Qt::red);
			} else {

				font.setUnderline(false);

				pal.setColor(QPalette::WindowText, Qt::black);
				pal.setColor(QPalette::Background, Qt::black);
			}
			docks[row][column]->setFont(font);
			docks[row][column]->setPalette(pal);
		}
	}

}

void wgt_sbench_command::on_pushButton_clear_clicked()
{

	for (int row = 0; row < SBENCH_MAX_ROW; ++row) {
		for (int column = 0; column < SBENCH_MAX_COL; ++column) {
			docks[row][column]->setChecked(false);
		}
	}

}

void wgt_sbench_command::on_pushButton_execute_clicked()
{
	execute();
}

void wgt_sbench_command::showEvent(QShowEvent * event)
{
	init();
}
