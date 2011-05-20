#include "ui_ecp_r_spkm.h"
#include "ui_r_spkm.h"
#include "robot/spkm/const_spkm.h"

#include "wgt_spkm_inc.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

wgt_spkm_inc::wgt_spkm_inc(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::spkm::UiRobot& _robot, QWidget *parent) :
	wgt_base("Spkm incremental motion", _interface, parent), robot(_robot)
{
	ui.setupUi(this);

	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p0);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p1);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p2);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p3);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p4);
	doubleSpinBox_cur_Vector.append(ui.doubleSpinBox_cur_p5);

	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p0);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p1);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p2);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p3);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p4);
	doubleSpinBox_des_Vector.append(ui.doubleSpinBox_des_p5);

	doubleSpinBox_mcur_Vector.append(ui.doubleSpinBox_mcur_0);
	doubleSpinBox_mcur_Vector.append(ui.doubleSpinBox_mcur_1);
	doubleSpinBox_mcur_Vector.append(ui.doubleSpinBox_mcur_2);
	doubleSpinBox_mcur_Vector.append(ui.doubleSpinBox_mcur_3);
	doubleSpinBox_mcur_Vector.append(ui.doubleSpinBox_mcur_4);
	doubleSpinBox_mcur_Vector.append(ui.doubleSpinBox_mcur_5);

	radioButton_mip_Vector.append(ui.radioButton_mip_0);
	radioButton_mip_Vector.append(ui.radioButton_mip_1);
	radioButton_mip_Vector.append(ui.radioButton_mip_2);
	radioButton_mip_Vector.append(ui.radioButton_mip_3);
	radioButton_mip_Vector.append(ui.radioButton_mip_4);
	radioButton_mip_Vector.append(ui.radioButton_mip_5);

	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(timer_slot()));
	timer->start(interface.position_refresh_interval);
	ui.radioButton_non_sync_trapezoidal->setChecked(true);

	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);

	for (int i = 0; i < robot.number_of_servos; i++) {
		doubleSpinBox_des_Vector[i]->setMaximum(robot.kinematic_params.upper_motor_pos_limits[i]);
		doubleSpinBox_des_Vector[i]->setMinimum(robot.kinematic_params.lower_motor_pos_limits[i]);
	}

}

void wgt_spkm_inc::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_spkm_inc::timer_slot()
{
	if ((dwgt->isVisible()) && (ui.checkBox_cyclic_read->isChecked())) {
		init();
	}

}

wgt_spkm_inc::~wgt_spkm_inc()
{

}

// slots

void wgt_spkm_inc::on_pushButton_read_clicked()
{
	init();
}

int wgt_spkm_inc::synchro_depended_widgets_disable(bool _set_disabled)
{
	ui.pushButton_execute->setDisabled(_set_disabled);
	ui.pushButton_read->setDisabled(_set_disabled);
	ui.pushButton_copy->setDisabled(_set_disabled);
	ui.checkBox_cyclic_read->setDisabled(_set_disabled);

	for (int i = 0; i < robot.number_of_servos; i++) {
		doubleSpinBox_des_Vector[i]->setDisabled(_set_disabled);
	}

	return 1;
}

void wgt_spkm_inc::synchro_depended_init_slot()
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

int wgt_spkm_inc::init()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

				robot.ui_ecp_robot->the_robot->epos_reply_data_request_port.set_request();
				robot.ui_ecp_robot->execute_motion();
				robot.ui_ecp_robot->the_robot->epos_reply_data_request_port.get();

				for (int i = 0; i < robot.number_of_servos; i++) {
					set_single_axis(i, doubleSpinBox_mcur_Vector[i], doubleSpinBox_cur_Vector[i], radioButton_mip_Vector[i]);
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

int wgt_spkm_inc::set_single_axis(int axis, QDoubleSpinBox* qdsb_mcur, QDoubleSpinBox* qdsb_cur_p, QAbstractButton* qab_mip)
{

	lib::epos::epos_reply &er = robot.ui_ecp_robot->the_robot->epos_reply_data_request_port.data;
	qdsb_mcur->setValue(er.epos_controller[axis].current);
	qdsb_cur_p->setValue(er.epos_controller[axis].position);

	if (er.epos_controller[axis].motion_in_progress) {
		qab_mip->setChecked(true);
	} else {
		qab_mip->setChecked(false);
	}

	return 1;
}

void wgt_spkm_inc::on_pushButton_import_clicked()
{
	double val[robot.number_of_servos];

	interface.get_main_window()->get_lineEdit_position(val, robot.number_of_servos);

	for (int i = 0; i < robot.number_of_servos; i++) {
		doubleSpinBox_des_Vector[i]->setValue(val[i]);
	}

}

void wgt_spkm_inc::on_pushButton_export_clicked()
{
	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << "edp_spkm INCREMENTAL POSITION\n";

	for (int i = 0; i < robot.number_of_servos; i++) {
		buffer << " " << doubleSpinBox_des_Vector[i]->value();
	}

	interface.ui_msg->message(buffer.str());

}

void wgt_spkm_inc::on_pushButton_copy_clicked()
{
	copy();
}

void wgt_spkm_inc::on_pushButton_stop_clicked()
{
	robot.execute_stop_motor();
}

int wgt_spkm_inc::copy()
{

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			synchro_depended_widgets_disable(false);

			for (int i = 0; i < robot.number_of_servos; i++) {
				doubleSpinBox_des_Vector[i]->setValue(doubleSpinBox_cur_Vector[i]->value());
			}
		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			synchro_depended_widgets_disable(true);
		}

	}

	return 1;
}

void wgt_spkm_inc::on_pushButton_execute_clicked()
{
	get_desired_position();
	move_it();
}

void wgt_spkm_inc::on_pushButton_0l_clicked()
{
	get_desired_position();
	robot.desired_pos[0] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_1l_clicked()
{
	get_desired_position();
	robot.desired_pos[1] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_2l_clicked()
{
	get_desired_position();
	robot.desired_pos[2] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_3l_clicked()
{
	get_desired_position();
	robot.desired_pos[3] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_4l_clicked()
{
	get_desired_position();
	robot.desired_pos[4] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_5l_clicked()
{
	get_desired_position();
	robot.desired_pos[5] -= ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_0r_clicked()
{
	get_desired_position();
	robot.desired_pos[0] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_1r_clicked()
{
	get_desired_position();
	robot.desired_pos[1] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_2r_clicked()
{
	get_desired_position();
	robot.desired_pos[2] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_3r_clicked()
{
	get_desired_position();
	robot.desired_pos[3] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_4r_clicked()
{
	get_desired_position();
	robot.desired_pos[4] += ui.doubleSpinBox_step->value();
	move_it();
}

void wgt_spkm_inc::on_pushButton_5r_clicked()
{
	get_desired_position();
	robot.desired_pos[5] += ui.doubleSpinBox_step->value();
	move_it();
}

int wgt_spkm_inc::get_desired_position()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			for (int i = 0; i < robot.number_of_servos; i++) {
				robot.desired_pos[i] = doubleSpinBox_des_Vector[i]->value();
			}
		} else {

			for (int i = 0; i < robot.number_of_servos; i++) {
				robot.desired_pos[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_spkm_inc::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			lib::epos::EPOS_MOTION_VARIANT motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

			if (ui.radioButton_non_sync_trapezoidal->isChecked()) {
				motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;
			}

			else if (ui.radioButton_sync_trapezoidal->isChecked()) {
				motion_variant = lib::epos::SYNC_TRAPEZOIDAL;
			}

			else if (ui.radioButton_sync_polynomal->isChecked()) {
				motion_variant = lib::epos::SYNC_POLYNOMIAL;
			}

			else if (ui.radioButton_operational->isChecked()) {
				motion_variant = lib::epos::OPERATIONAL;
			}

			robot.ui_ecp_robot->move_motors(robot.desired_pos, motion_variant);

			if ((robot.state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot.state.edp.is_synchronised)
				for (int i = 0; i < robot.number_of_servos; i++) {
					doubleSpinBox_des_Vector[i]->setValue(robot.desired_pos[i]);
				}

				init();
			}
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

