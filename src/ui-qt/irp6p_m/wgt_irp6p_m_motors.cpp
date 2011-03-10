//#include "ui_ecp_r_irp6p_m.h"
#include "ui_r_irp6p_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "wgt_irp6p_m_motors.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

wgt_irp6p_m_motors::wgt_irp6p_m_motors(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::irp6p_m::UiRobot& _robot, QWidget *parent) :
	wgt_base("irp6p_m motors moves", _interface, parent), robot(_robot)
{
	ui.setupUi(this);
}

wgt_irp6p_m_motors::~wgt_irp6p_m_motors()
{

}

void wgt_irp6p_m_motors::my_open()
{
	wgt_base::my_open();
	init();
	copy();
}

// slots
void wgt_irp6p_m_motors::on_pushButton_read_clicked()
{
	printf("read\n");
	init();
}

int wgt_irp6p_m_motors::init()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				ui.pushButton_execute->setDisabled(false);
				robot.ui_ecp_robot->read_motors(robot.current_pos);

				ui.doubleSpinBox_cur_p1->setValue(robot.current_pos[0]);
				ui.doubleSpinBox_cur_p2->setValue(robot.current_pos[1]);
				ui.doubleSpinBox_cur_p3->setValue(robot.current_pos[2]);
				ui.doubleSpinBox_cur_p4->setValue(robot.current_pos[3]);
				ui.doubleSpinBox_cur_p5->setValue(robot.current_pos[4]);
				ui.doubleSpinBox_cur_p6->setValue(robot.current_pos[5]);
				ui.doubleSpinBox_cur_p7->setValue(robot.current_pos[6]);

				for (int i = 0; i < robot.number_of_servos; i++) {
					robot.desired_pos[i] = robot.current_pos[i];
				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				ui.pushButton_execute->setDisabled(true);
			}
		}

	} // end try
	CATCH_SECTION_UI

	return 1;
}

void wgt_irp6p_m_motors::on_pushButton_import_clicked()
{
	double val[robot.number_of_servos];

	for (int i = 0; i < robot.number_of_servos; i++) {
		val[i] = 0.0;
	}

	interface.get_main_window()->get_lineEdit_position(val, robot.number_of_servos);

	ui.doubleSpinBox_des_p1->setValue(val[0]);
	ui.doubleSpinBox_des_p2->setValue(val[1]);
	ui.doubleSpinBox_des_p3->setValue(val[2]);
	ui.doubleSpinBox_des_p4->setValue(val[3]);
	ui.doubleSpinBox_des_p5->setValue(val[4]);
	ui.doubleSpinBox_des_p6->setValue(val[5]);
	ui.doubleSpinBox_des_p7->setValue(val[6]);

}

void wgt_irp6p_m_motors::on_pushButton_export_clicked()
{

	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << "edp_irp6p_m INTERNAL POSITION\n " << ui.doubleSpinBox_des_p1->value() << " "
			<< ui.doubleSpinBox_des_p2->value() << " " << ui.doubleSpinBox_des_p3->value() << " "
			<< ui.doubleSpinBox_des_p4->value() << " " << ui.doubleSpinBox_des_p5->value() << " "
			<< ui.doubleSpinBox_des_p6->value() << " " << ui.doubleSpinBox_des_p7->value();

	interface.ui_msg->message(buffer.str());
}

void wgt_irp6p_m_motors::on_pushButton_copy_clicked()
{
	copy();
}

int wgt_irp6p_m_motors::copy()
{

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_execute->setDisabled(false);

			ui.doubleSpinBox_des_p1->setValue(ui.doubleSpinBox_cur_p1->value());
			ui.doubleSpinBox_des_p2->setValue(ui.doubleSpinBox_cur_p2->value());
			ui.doubleSpinBox_des_p3->setValue(ui.doubleSpinBox_cur_p3->value());
			ui.doubleSpinBox_des_p4->setValue(ui.doubleSpinBox_cur_p4->value());
			ui.doubleSpinBox_des_p5->setValue(ui.doubleSpinBox_cur_p5->value());
			ui.doubleSpinBox_des_p6->setValue(ui.doubleSpinBox_cur_p6->value());
			ui.doubleSpinBox_des_p7->setValue(ui.doubleSpinBox_cur_p7->value());

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui.pushButton_execute->setDisabled(true);
		}

	}

	return 1;
}

void wgt_irp6p_m_motors::on_pushButton_execute_clicked()
{
	get_desired_position();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_1l_clicked()
{
	get_desired_position();
	robot.desired_pos[0] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_2l_clicked()
{
	get_desired_position();
	robot.desired_pos[1] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_3l_clicked()
{
	get_desired_position();
	robot.desired_pos[2] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_4l_clicked()
{
	get_desired_position();
	robot.desired_pos[3] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_5l_clicked()
{
	get_desired_position();
	robot.desired_pos[4] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_6l_clicked()
{
	get_desired_position();
	robot.desired_pos[5] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_7l_clicked()
{
	get_desired_position();
	robot.desired_pos[6] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_1r_clicked()
{
	get_desired_position();
	robot.desired_pos[0] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_2r_clicked()
{
	get_desired_position();
	robot.desired_pos[1] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_3r_clicked()
{
	get_desired_position();
	robot.desired_pos[2] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_4r_clicked()
{
	get_desired_position();
	robot.desired_pos[3] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_5r_clicked()
{
	get_desired_position();
	robot.desired_pos[4] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_6r_clicked()
{
	get_desired_position();
	robot.desired_pos[5] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_irp6p_m_motors::on_pushButton_7r_clicked()
{
	get_desired_position();
	robot.desired_pos[6] += ui.doubleSpinBox_step->value();
	move_it();
}

int wgt_irp6p_m_motors::get_desired_position()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			robot.desired_pos[0] = ui.doubleSpinBox_des_p1->value();
			robot.desired_pos[1] = ui.doubleSpinBox_des_p2->value();
			robot.desired_pos[2] = ui.doubleSpinBox_des_p3->value();
			robot.desired_pos[3] = ui.doubleSpinBox_des_p4->value();
			robot.desired_pos[4] = ui.doubleSpinBox_des_p5->value();
			robot.desired_pos[5] = ui.doubleSpinBox_des_p6->value();
			robot.desired_pos[6] = ui.doubleSpinBox_des_p7->value();

		} else {

			for (int i = 0; i < robot.number_of_servos; i++) {
				robot.desired_pos[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_irp6p_m_motors::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			//robot.ui_ecp_robot->move_motors(robot.desired_pos);

			//robot.ui_ecp_robot->move_motors(robot.desired_pos);
			robot.ui_ecp_robot->move_motors(robot.desired_pos);

			//robot.ui_ecp_robot->interface.irp6p_m->ui_ecp_robot->move_motors(robot.desired_pos);
			//interface.irp6p_m->ui_ecp_robot->move_motors(interface.irp6p_m->desired_pos);
			//interface.irp6p_m->ui_ecp_robot->move_motors(interface.irp6p_m->desired_pos);
			//robot.ui_ecp_robot->move_motors(robot.desired_pos);

			if ((robot.state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot.state.edp.is_synchronised)
				ui.doubleSpinBox_des_p1->setValue(robot.desired_pos[0]);
				ui.doubleSpinBox_des_p2->setValue(robot.desired_pos[1]);
				ui.doubleSpinBox_des_p3->setValue(robot.desired_pos[2]);
				ui.doubleSpinBox_des_p4->setValue(robot.desired_pos[3]);
				ui.doubleSpinBox_des_p5->setValue(robot.desired_pos[4]);
				ui.doubleSpinBox_des_p6->setValue(robot.desired_pos[5]);
				ui.doubleSpinBox_des_p7->setValue(robot.desired_pos[6]);
				init();
			}
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

