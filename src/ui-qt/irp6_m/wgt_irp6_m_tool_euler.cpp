//#include "ui_ecp_r_irp6_m.h"
#include "ui_r_irp6_m.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "wgt_irp6_m_tool_euler.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

const int wgt_irp6_m_tool_euler::aa_number = 6;

wgt_irp6_m_tool_euler::wgt_irp6_m_tool_euler(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::irp6_m::UiRobot& _robot, QWidget *parent) :
	wgt_base(_widget_label, _interface, parent), robot(_robot)
{
	ui.setupUi(this);

	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(init_and_copy_signal()), this, SLOT(init_and_copy_slot()), Qt::QueuedConnection);

	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p1);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p2);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p3);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p4);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p5);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p6);

	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p1);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p2);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p3);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p4);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p5);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p6);

}

wgt_irp6_m_tool_euler::~wgt_irp6_m_tool_euler()
{

}

int wgt_irp6_m_tool_euler::synchro_depended_widgets_disable(bool _set_disabled)
{
	//ui.pushButton_execute->setDisabled(_set_disabled);
	ui.pushButton_copy->setDisabled(_set_disabled);
	//ui.pushButton_export->setDisabled(_set_disabled);
	//ui.pushButton_import->setDisabled(_set_disabled);
	ui.pushButton_read->setDisabled(_set_disabled);

	for (int i = 0; i < aa_number; i++) {
		doubleSpinBox_cur_Vector[i]->setDisabled(_set_disabled);
		doubleSpinBox_des_Vector[i]->setDisabled(_set_disabled);
	}

	return 1;
}

void wgt_irp6_m_tool_euler::my_open(bool set_on_top)
{
	wgt_base::my_open(set_on_top);
	init_and_copy_slot();
}

void wgt_irp6_m_tool_euler::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_irp6_m_tool_euler::init_and_copy()
{
	emit init_and_copy_signal();
}

void wgt_irp6_m_tool_euler::init_and_copy_slot()
{
	init();
	copy();
}

void wgt_irp6_m_tool_euler::synchro_depended_init_slot()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);
			}
		}

	} // end try
	CATCH_SECTION_UI
}

// slots
void wgt_irp6_m_tool_euler::on_pushButton_read_clicked()
{
	printf("read\n");
	init();
}

int wgt_irp6_m_tool_euler::init()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

				robot.ui_ecp_robot->read_tool_xyz_euler_zyz(tool_vector);// co tutaj ma być?

				for (int i = 0; i < aa_number; i++) {
					doubleSpinBox_cur_Vector[i]->setValue(tool_vector[i]);

				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);

			}
		}

	} // end try
	CATCH_SECTION_UI

	return 1;
}

void wgt_irp6_m_tool_euler::on_pushButton_copy_clicked()
{
	copy();
}

int wgt_irp6_m_tool_euler::copy()
{

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_set->setDisabled(false);

			for (int i = 0; i < aa_number; i++) {
				doubleSpinBox_des_Vector[i]->setValue(doubleSpinBox_cur_Vector[i]->value());
			}
		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui.pushButton_set->setDisabled(true);
		}

	}

	return 1;
}

void wgt_irp6_m_tool_euler::on_pushButton_set_clicked()
{
	get_desired_position();
	move_it();
}

int wgt_irp6_m_tool_euler::get_desired_position()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			for (int i = 0; i < aa_number; i++) {
				tool_vector[i] = doubleSpinBox_des_Vector[i]->value();
			}
		} else {

			for (int i = 0; i < aa_number; i++) {
				tool_vector[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_irp6_m_tool_euler::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			//robot.ui_ecp_robot->move_tool_euler(robot.desired_pos);

			//robot.ui_ecp_robot->move_tool_euler(robot.desired_pos);
			robot.ui_ecp_robot->set_tool_xyz_euler_zyz(tool_vector);

			//robot.ui_ecp_robot->interface.irp6_m->ui_ecp_robot->move_tool_euler(robot.desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_tool_euler(interface.irp6_m->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_tool_euler(interface.irp6_m->desired_pos);
			//robot.ui_ecp_robot->move_tool_euler(robot.desired_pos);

			if ((robot.state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot.state.edp.is_synchronised)
				for (int i = 0; i < aa_number; i++) {
					doubleSpinBox_des_Vector[i]->setValue(tool_vector[i]);
				}
				init();
			}
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

