//#include "ui_ecp_r_polycrank.h"
#include "ui_r_polycrank.h"
#include "robot/polycrank/const_polycrank.h"
//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "wgt_polycrank_int.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

wgt_polycrank_int::wgt_polycrank_int(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
	wgt_base(_widget_label, _interface, parent)
{
	ui.setupUi(this);
	robot = dynamic_cast<mrrocpp::ui::polycrank::UiRobot *>(_robot);

	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p1);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p2);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p3);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p4);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p5);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p6);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p7);

	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p1);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p2);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p3);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p4);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p5);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p6);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p7);

}

wgt_polycrank_int::~wgt_polycrank_int()
{

}

// slots
void wgt_polycrank_int::on_pushButton_read_clicked()
{
	printf("read\n");
	init();
}

int wgt_polycrank_int::init()
{

	try {

		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				ui.pushButton_execute->setDisabled(false);
				robot->ui_ecp_robot->read_joints(robot->current_pos);

				for (int i = 0; i < robot->number_of_servos; i++) {
					doubleSpinBox_cur_Vector[i]->setValue(robot->current_pos[i]);
					robot->desired_pos[i] = robot->current_pos[i];
				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				ui.pushButton_execute->setDisabled(true);
			}
		}

	} // end try
	CATCH_SECTION_UI_PTR

	return 1;
}

void wgt_polycrank_int::on_pushButton_import_clicked()
{
	double val[robot->number_of_servos];

	for (int i = 0; i < robot->number_of_servos; i++) {
		val[i] = 0.0;
	}

	interface.get_main_window()->get_lineEdit_position(val, robot->number_of_servos);

	for (int i = 0; i < robot->number_of_servos; i++) {
		doubleSpinBox_des_Vector[i]->setValue(val[i]);
	}

}

void wgt_polycrank_int::on_pushButton_export_clicked()
{

	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << "edp_polycrank INCREMENTAL POSITION\n ";

	for (int i = 0; i < robot->number_of_servos; i++) {
		buffer << " " << doubleSpinBox_des_Vector[i]->value();
	}

	interface.ui_msg->message(buffer.str());
}

void wgt_polycrank_int::on_pushButton_copy_clicked()
{
	copy();
}

int wgt_polycrank_int::copy()
{

	if (robot->state.edp.pid != -1) {
		if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_execute->setDisabled(false);

			for (int i = 0; i < robot->number_of_servos; i++) {
				doubleSpinBox_des_Vector[i]->setValue(doubleSpinBox_cur_Vector[i]->value());
			}
		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui.pushButton_execute->setDisabled(true);
		}

	}

	return 1;
}

void wgt_polycrank_int::on_pushButton_execute_clicked()
{
	get_desired_position();
	move_it();
}

void wgt_polycrank_int::on_pushButton_1l_clicked()
{
	get_desired_position();
	robot->desired_pos[0] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_2l_clicked()
{
	get_desired_position();
	robot->desired_pos[1] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_3l_clicked()
{
	get_desired_position();
	robot->desired_pos[2] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_4l_clicked()
{
	get_desired_position();
	robot->desired_pos[3] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_5l_clicked()
{
	get_desired_position();
	robot->desired_pos[4] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_6l_clicked()
{
	get_desired_position();
	robot->desired_pos[5] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_7l_clicked()
{
	get_desired_position();
	robot->desired_pos[6] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_1r_clicked()
{
	get_desired_position();
	robot->desired_pos[0] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_2r_clicked()
{
	get_desired_position();
	robot->desired_pos[1] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_3r_clicked()
{
	get_desired_position();
	robot->desired_pos[2] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_4r_clicked()
{
	get_desired_position();
	robot->desired_pos[3] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_5r_clicked()
{
	get_desired_position();
	robot->desired_pos[4] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_6r_clicked()
{
	get_desired_position();
	robot->desired_pos[5] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_polycrank_int::on_pushButton_7r_clicked()
{
	get_desired_position();
	robot->desired_pos[6] += ui.doubleSpinBox_step->value();
	move_it();
}

int wgt_polycrank_int::get_desired_position()
{

	if (robot->state.edp.pid != -1) {

		if (robot->state.edp.is_synchronised) {

			for (int i = 0; i < robot->number_of_servos; i++) {
				robot->desired_pos[i] = doubleSpinBox_des_Vector[i]->value();
			}
		} else {

			for (int i = 0; i < robot->number_of_servos; i++) {
				robot->desired_pos[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_polycrank_int::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot->state.edp.pid != -1) {

			//robot->ui_ecp_robot->move_motors(robot->desired_pos);

			//robot->ui_ecp_robot->move_motors(robot->desired_pos);
			robot->ui_ecp_robot->move_joints(robot->desired_pos);

			//robot->ui_ecp_robot->interface.polycrank->ui_ecp_robot->move_joints(robot->desired_pos);
			//interface.polycrank->ui_ecp_robot->move_motors(interface.polycrank->desired_pos);
			//interface.polycrank->ui_ecp_robot->move_joints(interface.polycrank->desired_pos);
			//robot->ui_ecp_robot->move_motors(robot->desired_pos);

			if ((robot->state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot->state.edp.is_synchronised)
				for (int i = 0; i < robot->number_of_servos; i++) {
					doubleSpinBox_des_Vector[i]->setValue(robot->desired_pos[i]);
				}

				init();
			}
		} // end if (robot->state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI_PTR

	return 1;
}

