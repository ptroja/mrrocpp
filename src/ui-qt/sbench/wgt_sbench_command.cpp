#include "../sbench/ui_ecp_r_sbench.h"
#include "../sbench/ui_r_sbench.h"
#include "robot/sbench/const_sbench.h"

#include "wgt_sbench_command.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

#include <QCheckBox>
#include <QFont>

#define SBENCH_MAX_ROW 8
#define SBENCH_MAX_COL 8

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
		}
	}

}

wgt_sbench_command::~wgt_sbench_command()
{

}

