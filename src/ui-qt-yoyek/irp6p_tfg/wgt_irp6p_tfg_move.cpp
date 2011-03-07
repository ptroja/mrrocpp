//#include "ui_ecp_r_irp6p_tfg.h"
#include "../base/ui_ecp_robot/ui_ecp_r_tfg_and_conv.h"
#include "ui_r_irp6p_tfg.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"

#include "wgt_irp6p_tfg_move.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

wgt_irp6p_tfg_move::wgt_irp6p_tfg_move(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::irp6p_tfg::UiRobot& _robot, QWidget *parent) :
	wgt_base("Sarkofag incremental motion", _interface, parent), robot(_robot)
{
	ui.setupUi(this);

	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);

	//	ui.doubleSpinBox_des_p0->setMaximum(robot.kinematic_params.upper_motor_pos_limits[0]);
	//	ui.doubleSpinBox_des_p0->setMinimum(robot.kinematic_params.lower_motor_pos_limits[0]);
}

wgt_irp6p_tfg_move::~wgt_irp6p_tfg_move()
{

}

void wgt_irp6p_tfg_move::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_irp6p_tfg_move::on_pushButton_read_mr_clicked()
{
	init();
}

int wgt_irp6p_tfg_move::synchro_depended_widgets_disable(bool _set_disabled)
{
	ui.pushButton_execute_mr->setDisabled(_set_disabled);
	ui.pushButton_read_mr->setDisabled(_set_disabled);
	ui.pushButton_copy_mr->setDisabled(_set_disabled);
	ui.doubleSpinBox_des_mr->setDisabled(_set_disabled);

	return 1;
}

void wgt_irp6p_tfg_move::synchro_depended_init_slot()
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

int wgt_irp6p_tfg_move::init()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				//synchro_depended_widgets_disable(false);

				//				robot.ui_ecp_robot->the_robot->epos_reply_data_request_port.set_request();
				//				robot.ui_ecp_robot->execute_motion();
				//				robot.ui_ecp_robot->the_robot->epos_reply_data_request_port.get();
				//
				//				set_single_axis(0, ui.doubleSpinBox_mcur_0, ui.doubleSpinBox_cur_p0, ui.radioButton_mip_0);


				robot.ui_ecp_robot->read_motors(interface.irp6p_tfg->current_pos); // Odczyt polozenia walow silnikow
				ui.doubleSpinBox_cur_mr->setValue(interface.irp6p_tfg->current_pos[0]);

				for (int i = 0; i < robot.number_of_servos; i++) {
					robot.desired_pos[i] = robot.current_pos[i];
				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				//synchro_depended_widgets_disable(true);
			}
		}

	} // end try
	CATCH_SECTION_UI

	return 1;
}

void wgt_irp6p_tfg_move::on_pushButton_import_mr_clicked()
{
	double val[robot.number_of_servos];

	interface.get_main_window()->get_lineEdit_position(val, robot.number_of_servos);

	ui.doubleSpinBox_des_mr->setValue(val[0]);
}

void wgt_irp6p_tfg_move::on_pushButton_export_mr_clicked()
{
	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << "edp_irp6p_tfg INCREMENTAL POSITION\n " << ui.doubleSpinBox_des_mr->value();

	interface.ui_msg->message(buffer.str());
}

void wgt_irp6p_tfg_move::on_pushButton_copy_mr_clicked()
{
	copy();
}

int wgt_irp6p_tfg_move::copy()
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

void wgt_irp6p_tfg_move::on_pushButton_execute_mr_clicked()
{
	get_desired_position();
	move_it();
}

void wgt_irp6p_tfg_move::on_pushButton_l_mr_clicked()
{
	get_desired_position();
	robot.desired_pos[0] -= ui.doubleSpinBox_step_mr->value();
	move_it();
}

void wgt_irp6p_tfg_move::on_pushButton_r_mr_clicked()
{
	get_desired_position();
	robot.desired_pos[0] += ui.doubleSpinBox_step_mr->value();
	move_it();
}

int wgt_irp6p_tfg_move::get_desired_position()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			robot.desired_pos[0] = ui.doubleSpinBox_des_mr->value();

		} else {

			for (int i = 0; i < robot.number_of_servos; i++) {
				robot.desired_pos[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_irp6p_tfg_move::move_it()
{
	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			robot.ui_ecp_robot->move_motors(robot.desired_pos);

			if ((robot.state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot.state.edp.is_synchronised)
				ui.doubleSpinBox_des_mr->setValue(robot.desired_pos[0]);

			}
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

