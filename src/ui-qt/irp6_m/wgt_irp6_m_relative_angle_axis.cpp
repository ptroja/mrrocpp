//#include "ui_ecp_r_irp6_m.h"
#include "ui_r_irp6_m.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "wgt_irp6_m_relative_angle_axis.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

const int wgt_irp6_m_relative_angle_axis::aa_number = 6;

wgt_irp6_m_relative_angle_axis::wgt_irp6_m_relative_angle_axis(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::irp6_m::UiRobot& _robot, QWidget *parent) :
	wgt_base(_widget_label, _interface, parent), robot(_robot)
{
	ui.setupUi(this);

	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(init_and_copy_signal()), this, SLOT(init_and_copy_slot()), Qt::QueuedConnection);

	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p1);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p2);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p3);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p4);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p5);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p6);
}

wgt_irp6_m_relative_angle_axis::~wgt_irp6_m_relative_angle_axis()
{

}

int wgt_irp6_m_relative_angle_axis::synchro_depended_widgets_disable(bool _set_disabled)
{
	//	ui.pushButton_execute->setDisabled(_set_disabled);
	//	ui.pushButton_copy->setDisabled(_set_disabled);
	//	ui.pushButton_export->setDisabled(_set_disabled);
	//	ui.pushButton_import->setDisabled(_set_disabled);
	//	ui.pushButton_read->setDisabled(_set_disabled);

	for (int i = 0; i < aa_number; i++) {
		doubleSpinBox_des_Vector[i]->setDisabled(_set_disabled);
	}

	return 1;
}

void wgt_irp6_m_relative_angle_axis::my_open(bool set_on_top)
{
	wgt_base::my_open(set_on_top);
	init_and_copy_slot();
}

void wgt_irp6_m_relative_angle_axis::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_irp6_m_relative_angle_axis::init_and_copy()
{
	emit init_and_copy_signal();
}

void wgt_irp6_m_relative_angle_axis::init_and_copy_slot()
{
	init();
	//	copy();
}

void wgt_irp6_m_relative_angle_axis::synchro_depended_init_slot()
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

int wgt_irp6_m_relative_angle_axis::init()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

				//robot.ui_ecp_robot->read_xyz_angle_axis(robot.current_pos);

				for (int i = 0; i < aa_number; i++) {
					doubleSpinBox_des_Vector[i]->setValue(0.0);
					robot.desired_pos[i] = robot.current_pos[i];
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

void wgt_irp6_m_relative_angle_axis::zero_desired_position()
{
	for (int i = 0; i < aa_number; i++)
		robot.desired_pos[i] = 0.0;
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_r_clicked()
{
	zero_desired_position(); // zerowanie desired position
	for (int i = 0; i < aa_number; i++)
		robot.desired_pos[i] = doubleSpinBox_des_Vector[i]->value();

	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_l_clicked()
{
	zero_desired_position(); // zerowanie desired position
	for (int i = 0; i < aa_number; i++)
		robot.desired_pos[i] = -doubleSpinBox_des_Vector[i]->value();

	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_1l_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[0] = -ui.doubleSpinBox_des_p1->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_2l_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[1] = -ui.doubleSpinBox_des_p2->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_3l_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[2] = -ui.doubleSpinBox_des_p3->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_4l_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[3] = -ui.doubleSpinBox_des_p4->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_5l_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[4] = -ui.doubleSpinBox_des_p5->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_6l_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[5] = -ui.doubleSpinBox_des_p6->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_1r_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[0] = ui.doubleSpinBox_des_p1->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_2r_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[1] = ui.doubleSpinBox_des_p2->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_3r_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[2] = ui.doubleSpinBox_des_p3->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_4r_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[3] = ui.doubleSpinBox_des_p4->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_5r_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[4] = ui.doubleSpinBox_des_p5->value();
	move_it();
}

void wgt_irp6_m_relative_angle_axis::on_pushButton_6r_clicked()
{
	zero_desired_position(); // zerowanie desired position
	robot.desired_pos[5] = ui.doubleSpinBox_des_p6->value();
	move_it();
}

int wgt_irp6_m_relative_angle_axis::get_desired_position()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			for (int i = 0; i < aa_number; i++) {
				robot.desired_pos[i] = doubleSpinBox_des_Vector[i]->value();
			}
		} else {

			for (int i = 0; i < aa_number; i++) {
				robot.desired_pos[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_irp6_m_relative_angle_axis::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			robot.ui_ecp_robot->move_xyz_angle_axis_relative(robot.desired_pos);

			//robot.ui_ecp_robot->interface.irp6_m->ui_ecp_robot->move_relative_angle_axis(robot.desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_relative_angle_axis(interface.irp6_m->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_relative_angle_axis(interface.irp6_m->desired_pos);
			//robot.ui_ecp_robot->move_relative_angle_axis(robot.desired_pos);
			/*
			 if ((robot.state.edp.is_synchronised)
			 for (int i = 0; i < aa_number; i++) {
			 doubleSpinBox_des_Vector[i]->setValue(robot.desired_pos[i]);
			 }
			 //	init();
			 }
			 */
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

