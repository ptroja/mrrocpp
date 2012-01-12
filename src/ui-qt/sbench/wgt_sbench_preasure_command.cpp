/*!
 * @file
 * @brief File contains wgt_sbench_preasure_command class definition for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */


#include "../sbench/ui_ecp_r_sbench.h"
#include "../sbench/ui_r_sbench.h"
#include "robot/sbench/const_sbench.h"

#include "wgt_sbench_preasure_command.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

#include <QFont>

wgt_sbench_preasure_command::wgt_sbench_preasure_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		wgt_sbench_command(_widget_label, _interface, _robot, parent)
{
}

wgt_sbench_preasure_command::~wgt_sbench_preasure_command()
{

}

void wgt_sbench_preasure_command::init()
{
	robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.set_request();
	robot->ui_ecp_robot->execute_motion();
	robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.get();

	QFont font;
	QPalette pal;

	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			if (robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.data.preasure_buf.get_value(i, j)) {

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
		}
	}
}

void wgt_sbench_preasure_command::read_and_copy()
{
	init();

	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			docks[i][j]->setChecked(robot->ui_ecp_robot->the_robot->sbench_reply_data_request_port.data.preasure_buf.get_value(i, j));
		}
	}

}

void wgt_sbench_preasure_command::execute()
{

	for (int i = 0; i < SBENCH_MAX_ROW; i++) {
		for (int j = 0; j < SBENCH_MAX_COL; j++) {
			robot->ui_ecp_robot->the_robot->sbench_command_preasure_data_port.data.set_value(i, j, docks[i][j]->isChecked());
		}
	}

	robot->ui_ecp_robot->the_robot->sbench_command_preasure_data_port.set();
	robot->ui_ecp_robot->execute_motion();

	init();
}

