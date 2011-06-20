#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "../base/ui_r_single_motor.h"

#include "wgt_single_motor_move.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

wgt_single_motor_move::wgt_single_motor_move(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::single_motor::UiRobot& _robot, QWidget *parent) :
	wgt_base(_widget_label, _interface, parent), robot(_robot)
{
	ui.setupUi(this);

	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(init_and_copy_signal()), this, SLOT(init_and_copy_slot()), Qt::QueuedConnection);

	//	ui.doubleSpinBox_des_p0->setMaximum(robot.kinematic_params.upper_motor_pos_limits[0]);
	//	ui.doubleSpinBox_des_p0->setMinimum(robot.kinematic_params.lower_motor_pos_limits[0]);
}

wgt_single_motor_move::~wgt_single_motor_move()
{

}

void wgt_single_motor_move::my_open(bool set_on_top)
{
	wgt_base::my_open(set_on_top);
	init_and_copy_slot();
}

void wgt_single_motor_move::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_single_motor_move::init_and_copy()
{
	emit init_and_copy_signal();
}

int wgt_single_motor_move::synchro_depended_widgets_disable(bool _set_disabled)
{
	ui.pushButton_execute_mr->setDisabled(_set_disabled);
	ui.pushButton_read_mr->setDisabled(_set_disabled);
	ui.pushButton_export_mr->setDisabled(_set_disabled);
	ui.pushButton_import_mr->setDisabled(_set_disabled);
	ui.pushButton_copy_mr->setDisabled(_set_disabled);
	ui.doubleSpinBox_des_mr->setDisabled(_set_disabled);

	ui.pushButton_execute_si->setDisabled(_set_disabled);
	ui.pushButton_read_si->setDisabled(_set_disabled);
	ui.pushButton_export_si->setDisabled(_set_disabled);
	ui.pushButton_import_si->setDisabled(_set_disabled);
	ui.doubleSpinBox_step_si->setDisabled(_set_disabled);
	ui.pushButton_copy_si->setDisabled(_set_disabled);
	ui.doubleSpinBox_des_si->setDisabled(_set_disabled);
	ui.pushButton_l_si->setDisabled(_set_disabled);
	ui.pushButton_r_si->setDisabled(_set_disabled);

	return 1;
}

void wgt_single_motor_move::init_and_copy_slot()
{
	init_mr_and_si();
	copy_mr_and_si();
}

void wgt_single_motor_move::synchro_depended_init_slot()
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

/***************************
 * MOTORS
 ****************************/

void wgt_single_motor_move::on_pushButton_read_mr_clicked()
{
	init_mr_and_si();
}

void wgt_single_motor_move::init_mr_and_si()
{
	init_mr();
	init_si();
}
void wgt_single_motor_move::copy_mr_and_si()
{
	copy_mr();
	copy_si();
}

int wgt_single_motor_move::init_mr()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

				robot.ui_ecp_robot->read_motors(robot.current_pos); // Odczyt polozenia walow silnikow
				ui.doubleSpinBox_cur_mr->setValue(robot.current_pos[0]);

				robot.desired_pos[0] = robot.current_pos[0];

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);
			}
		}

	} // end try
	CATCH_SECTION_UI

	return 1;
}

void wgt_single_motor_move::on_pushButton_import_mr_clicked()
{
	double val[robot.number_of_servos];

	interface.get_main_window()->get_lineEdit_position(val, robot.number_of_servos);

	ui.doubleSpinBox_des_mr->setValue(val[0]);
}

void wgt_single_motor_move::on_pushButton_export_mr_clicked()
{
	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << widget_label.toStdString() << " INCREMENTAL POSITION\n " << ui.doubleSpinBox_des_mr->value();

	interface.ui_msg->message(buffer.str());
}

void wgt_single_motor_move::on_pushButton_copy_mr_clicked()
{
	copy_mr();
}

int wgt_single_motor_move::copy_mr()
{

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_execute_mr->setDisabled(false);

			ui.doubleSpinBox_des_mr->setValue(ui.doubleSpinBox_cur_mr->value());

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui.pushButton_execute_mr->setDisabled(true);
		}

	}

	return 1;
}

void wgt_single_motor_move::on_pushButton_execute_mr_clicked()
{
	get_desired_position_mr();
	move_it_mr();
}

void wgt_single_motor_move::on_pushButton_l_mr_clicked()
{
	get_desired_position_mr();
	robot.desired_pos[0] -= ui.doubleSpinBox_step_mr->value();
	move_it_mr();
}

void wgt_single_motor_move::on_pushButton_r_mr_clicked()
{
	get_desired_position_mr();
	robot.desired_pos[0] += ui.doubleSpinBox_step_mr->value();
	move_it_mr();
}

int wgt_single_motor_move::get_desired_position_mr()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			robot.desired_pos[0] = ui.doubleSpinBox_des_mr->value();

		} else {

			robot.desired_pos[0] = 0.0;

		}
	}
	return 1;
}

int wgt_single_motor_move::move_it_mr()
{
	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			robot.ui_ecp_robot->move_motors(robot.desired_pos);

			if (robot.state.edp.is_synchronised) {
				ui.doubleSpinBox_des_mr->setValue(robot.desired_pos[0]);

				init_mr_and_si();
			}

		} // end if (robot.state.edp.pid!=-1)

	} // end try

	CATCH_SECTION_UI

	return 1;
}

/***************************
 * JOINS
 ****************************/

void wgt_single_motor_move::on_pushButton_read_si_clicked()
{
	init_mr_and_si();
}

int wgt_single_motor_move::init_si()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

				//				robot.ui_ecp_robot->the_robot->epos_reply_data_request_port.set_request();
				//				robot.ui_ecp_robot->execute_motion();
				//				robot.ui_ecp_robot->the_robot->epos_reply_data_request_port.get();
				//
				//				set_single_axis(0, ui.doubleSpinBox_mcur_0, ui.doubleSpinBox_cur_p0, ui.radioButton_mip_0);


				robot.ui_ecp_robot->read_joints(robot.current_pos); // Odczyt polozenia walow silnikow
				ui.doubleSpinBox_cur_si->setValue(robot.current_pos[0]);

				robot.desired_pos[0] = robot.current_pos[0];

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);
			}
		}

	} // end try
	CATCH_SECTION_UI

	return 1;
}

void wgt_single_motor_move::on_pushButton_import_si_clicked()
{
	double val[robot.number_of_servos];

	interface.get_main_window()->get_lineEdit_position(val, robot.number_of_servos);

	ui.doubleSpinBox_des_si->setValue(val[0]);
}

void wgt_single_motor_move::on_pushButton_export_si_clicked()
{
	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << widget_label.toStdString() << " JOINTS POSITION\n " << ui.doubleSpinBox_des_si->value();

	interface.ui_msg->message(buffer.str());
}

void wgt_single_motor_move::on_pushButton_copy_si_clicked()
{
	copy_si();
}

int wgt_single_motor_move::copy_si()
{

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_execute_si->setDisabled(false);

			ui.doubleSpinBox_des_si->setValue(ui.doubleSpinBox_cur_si->value());

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui.pushButton_execute_si->setDisabled(true);
		}

	}

	return 1;
}

void wgt_single_motor_move::on_pushButton_execute_si_clicked()
{
	get_desired_position_si();
	move_it_si();
}

void wgt_single_motor_move::on_pushButton_l_si_clicked()
{
	get_desired_position_si();
	robot.desired_pos[0] -= ui.doubleSpinBox_step_si->value();
	move_it_si();
}

void wgt_single_motor_move::on_pushButton_r_si_clicked()
{
	get_desired_position_si();
	robot.desired_pos[0] += ui.doubleSpinBox_step_si->value();
	move_it_si();
}

int wgt_single_motor_move::get_desired_position_si()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			robot.desired_pos[0] = ui.doubleSpinBox_des_si->value();

		} else {

			robot.desired_pos[0] = 0.0;

		}
	}
	return 1;
}

int wgt_single_motor_move::move_it_si()
{
	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			robot.ui_ecp_robot->move_joints(robot.desired_pos);

			if (robot.state.edp.is_synchronised) {
				ui.doubleSpinBox_des_si->setValue(robot.desired_pos[0]);

				init_mr_and_si();
			}

		} // end if (robot.state.edp.pid!=-1)

	} // end try

	CATCH_SECTION_UI

	return 1;
}
