//#include "ui_ecp_r_sarkofag.h"
#include "../base/ui_ecp_robot/ui_ecp_r_tfg_and_conv.h"
#include "ui_r_sarkofag.h"
#include "robot/sarkofag/const_sarkofag.h"

#include "wgt_sarkofag_inc.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

wgt_sarkofag_inc::wgt_sarkofag_inc(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::sarkofag::UiRobot& _robot, QWidget *parent) :
	wgt_base("Sarkofag incremental motion", _interface, parent), robot(_robot)
{
	ui.setupUi(this);
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(on_timer_slot()));
	timer->start(interface.position_refresh_interval);
	//	ui.radioButton_non_sync_trapezoidal->setChecked(true);

	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);

	//	ui.doubleSpinBox_des_p0->setMaximum(robot.kinematic_params.upper_motor_pos_limits[0]);
	//	ui.doubleSpinBox_des_p0->setMinimum(robot.kinematic_params.lower_motor_pos_limits[0]);
}

wgt_sarkofag_inc::~wgt_sarkofag_inc()
{

}

void wgt_sarkofag_inc::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_sarkofag_inc::on_timer_slot()
{
	if ((dwgt->isVisible()) && (ui.checkBox_cyclic_read->isChecked())) {
		init();
	}
}

void wgt_sarkofag_inc::on_pushButton_read_clicked()
{
	init();
}

int wgt_sarkofag_inc::synchro_depended_widgets_disable(bool _set_disabled)
{
	ui.pushButton_execute->setDisabled(_set_disabled);
	ui.pushButton_read->setDisabled(_set_disabled);
	ui.pushButton_copy->setDisabled(_set_disabled);
	ui.checkBox_cyclic_read->setDisabled(_set_disabled);
	ui.doubleSpinBox_des_p0->setDisabled(_set_disabled);

	return 1;
}

void wgt_sarkofag_inc::synchro_depended_init_slot()
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

int wgt_sarkofag_inc::init()
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


				robot.ui_ecp_robot->read_motors(interface.sarkofag->current_pos); // Odczyt polozenia walow silnikow
				set_single_axis(0, ui.doubleSpinBox_mcur_0, ui.doubleSpinBox_cur_p0, ui.radioButton_mip_0);

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

int wgt_sarkofag_inc::set_single_axis(int axis, QDoubleSpinBox* qdsb_mcur, QDoubleSpinBox* qdsb_cur_p, QAbstractButton* qab_mip)
{

	//	lib::epos::epos_reply &er = robot.ui_ecp_robot->the_robot->epos_reply_data_request_port.data;
	//	qdsb_mcur->setValue(er.epos_controller[axis].current);
	//	qdsb_cur_p->setValue(er.epos_controller[axis].position);
	qdsb_cur_p->setValue(interface.sarkofag->current_pos[axis]);

	//	if (er.epos_controller[axis].motion_in_progress) {
	//		qab_mip->setChecked(true);
	//	} else {
	//		qab_mip->setChecked(false);
	//	}

	return 1;
}

void wgt_sarkofag_inc::on_pushButton_import_clicked()
{
	double val[robot.number_of_servos];

	interface.get_main_window()->get_lineEdit_position(val, robot.number_of_servos);

	ui.doubleSpinBox_des_p0->setValue(val[0]);
}

void wgt_sarkofag_inc::on_pushButton_export_clicked()
{
	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << "edp_sarkofag INCREMENTAL POSITION\n " << ui.doubleSpinBox_des_p0->value();

	interface.ui_msg->message(buffer.str());
}

void wgt_sarkofag_inc::on_pushButton_copy_clicked()
{
	copy();
}

void wgt_sarkofag_inc::on_pushButton_stop_clicked()
{
	//	robot.execute_stop_motor();
}

int wgt_sarkofag_inc::copy()
{

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui.pushButton_execute->setDisabled(false);

			ui.doubleSpinBox_des_p0->setValue(ui.doubleSpinBox_cur_p0->value());

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui.pushButton_execute->setDisabled(true);
		}

	}

	return 1;
}

void wgt_sarkofag_inc::on_pushButton_execute_clicked()
{
	get_desired_position();
	move_it();
}

void wgt_sarkofag_inc::on_pushButton_0l_clicked()
{
	get_desired_position();
	robot.desired_pos[0] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_sarkofag_inc::on_pushButton_0r_clicked()
{
	get_desired_position();
	robot.desired_pos[0] += ui.doubleSpinBox_step->value();
	move_it();
}

int wgt_sarkofag_inc::get_desired_position()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			robot.desired_pos[0] = ui.doubleSpinBox_des_p0->value();

		} else {

			for (int i = 0; i < robot.number_of_servos; i++) {
				robot.desired_pos[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_sarkofag_inc::move_it()
{
	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			robot.ui_ecp_robot->move_motors(robot.desired_pos);

			if ((robot.state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot.state.edp.is_synchronised)
				ui.doubleSpinBox_des_p0->setValue(robot.desired_pos[0]);

			}
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

