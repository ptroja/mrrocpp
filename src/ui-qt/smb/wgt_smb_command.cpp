#include "ui_ecp_r_smb.h"
#include "ui_r_smb.h"
#include "robot/smb/const_smb.h"

#include "wgt_smb_command.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

wgt_smb_command::wgt_smb_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		wgt_base(_widget_label, _interface, parent)
{
	ui.setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::smb::UiRobot *>(_robot);

	// utworzenie list widgetow

	checkBox_fl_in_Vector.append(ui.checkBox_fl1_in);
	checkBox_fl_in_Vector.append(ui.checkBox_fl2_in);
	checkBox_fl_in_Vector.append(ui.checkBox_fl3_in);

	checkBox_fl_out_Vector.append(ui.checkBox_fl1_out);
	checkBox_fl_out_Vector.append(ui.checkBox_fl2_out);
	checkBox_fl_out_Vector.append(ui.checkBox_fl3_out);

	checkBox_fl_undetachable_Vector.append(ui.checkBox_fl1_udetachable);
	checkBox_fl_undetachable_Vector.append(ui.checkBox_fl2_udetachable);
	checkBox_fl_undetachable_Vector.append(ui.checkBox_fl3_udetachable);

	checkBox_fl_attached_Vector.append(ui.checkBox_fl1_attached);
	checkBox_fl_attached_Vector.append(ui.checkBox_fl2_attached);
	checkBox_fl_attached_Vector.append(ui.checkBox_fl3_attached);

	checkBox_m_mip_Vector.append(ui.checkBox_ml_mip);
	checkBox_m_mip_Vector.append(ui.checkBox_ms_mip);

	radioButton_fl_in_Vector.append(ui.radioButton_fl1_in);
	radioButton_fl_in_Vector.append(ui.radioButton_fl2_in);
	radioButton_fl_in_Vector.append(ui.radioButton_fl3_in);

	radioButton_fl_out_Vector.append(ui.radioButton_fl1_out);
	radioButton_fl_out_Vector.append(ui.radioButton_fl2_out);
	radioButton_fl_out_Vector.append(ui.radioButton_fl3_out);

	doubleSpinBox_m_current_position_Vector.append(ui.doubleSpinBox_ml_current_position);
	doubleSpinBox_m_current_position_Vector.append(ui.doubleSpinBox_ms_current_position);

	doubleSpinBox_m_absolute_Vector.append(ui.doubleSpinBox_ml_absolute);
	doubleSpinBox_m_absolute_Vector.append(ui.doubleSpinBox_ms_absolute);

	doubleSpinBox_m_relative_Vector.append(ui.doubleSpinBox_ml_relative);
	doubleSpinBox_m_relative_Vector.append(ui.doubleSpinBox_ms_relative);

	// uruchomienei timera
	timer = (boost::shared_ptr <QTimer>) new QTimer(this);
	connect(timer.get(), SIGNAL(timeout()), this, SLOT(timer_slot()));
	timer->start(interface.position_refresh_interval);

	// podpiecie pozostalych sygnalow do slotow
	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);

}

wgt_smb_command::~wgt_smb_command()
{

}

void wgt_smb_command::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_smb_command::synchro_depended_init_slot()
{

	try {

		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);
			}
		}

	} // end try
	CATCH_SECTION_UI_PTR

}

int wgt_smb_command::init()
{
	interface.ui_msg->message("init");

	try {

		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);
				if (ui.radioButton_m_motor->isChecked()) {
					robot->ui_ecp_robot->the_robot->epos_motor_reply_data_request_port.set_request();
				} else if (ui.radioButton_m_joint->isChecked()) {
					robot->ui_ecp_robot->the_robot->epos_joint_reply_data_request_port.set_request();
				} else if (ui.radioButton_m_ext->isChecked()) {
					robot->ui_ecp_robot->the_robot->epos_external_reply_data_request_port.set_request();
				}
				robot->ui_ecp_robot->the_robot->smb_multi_leg_reply_data_request_port.set_request();
				robot->ui_ecp_robot->execute_motion();
				robot->ui_ecp_robot->the_robot->smb_multi_leg_reply_data_request_port.get();
				lib::epos::epos_reply *er;
				lib::smb::smb_ext_epos_reply *ser;

				if (ui.radioButton_m_motor->isChecked()) {

					robot->ui_ecp_robot->the_robot->epos_motor_reply_data_request_port.get();
					er = &robot->ui_ecp_robot->the_robot->epos_motor_reply_data_request_port.data;
				} else if (ui.radioButton_m_joint->isChecked()) {
					robot->ui_ecp_robot->the_robot->epos_joint_reply_data_request_port.get();
					er = &robot->ui_ecp_robot->the_robot->epos_joint_reply_data_request_port.data;
				} else if (ui.radioButton_m_ext->isChecked()) {
					robot->ui_ecp_robot->the_robot->epos_external_reply_data_request_port.get();
					ser = &robot->ui_ecp_robot->the_robot->epos_external_reply_data_request_port.data;
				}

				// sets leg state

				lib::smb::multi_leg_reply_td &mlr =
						robot->ui_ecp_robot->the_robot->smb_multi_leg_reply_data_request_port.data;

				for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
					checkBox_fl_in_Vector[i]->setChecked(mlr.leg[i].is_in);
					checkBox_fl_out_Vector[i]->setChecked(mlr.leg[i].is_out);
					checkBox_fl_attached_Vector[i]->setChecked(mlr.leg[i].is_attached);
				}

				if (ui.radioButton_m_motor->isChecked()) {
					for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
						checkBox_m_mip_Vector[i]->setChecked(er->epos_controller[i].motion_in_progress);
						doubleSpinBox_m_current_position_Vector[i]->setValue(er->epos_controller[i].position);
					}
				} else if (ui.radioButton_m_joint->isChecked()) {
					for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
						checkBox_m_mip_Vector[i]->setChecked(er->epos_controller[i].motion_in_progress);
						doubleSpinBox_m_current_position_Vector[i]->setValue(er->epos_controller[i].position);
					}
				} else if (ui.radioButton_m_ext->isChecked()) {
					for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
						checkBox_m_mip_Vector[i]->setChecked(ser->epos_controller[i].motion_in_progress);
						doubleSpinBox_m_current_position_Vector[i]->setValue(ser->epos_controller[i].position);
					}
				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);
			}
		}

	} // end try
	CATCH_SECTION_UI_PTR

	return 1;
}

int wgt_smb_command::synchro_depended_widgets_disable(bool _set_disabled)
{

	ui.pushButton_m_execute->setDisabled(_set_disabled);
	ui.pushButton_ml_copy->setDisabled(_set_disabled);
	ui.pushButton_ms_copy->setDisabled(_set_disabled);

	for (int i = 0; i < robot->number_of_servos; i++) {
		doubleSpinBox_m_absolute_Vector[i]->setDisabled(_set_disabled);

	}

	return 1;
}

void wgt_smb_command::timer_slot()
{
	if ((dwgt->isVisible()) && (ui.checkBox_cyclic_read->isChecked())) {
		init();
	}

}

int wgt_smb_command::get_desired_position()
{

	if (robot->state.edp.pid != -1) {

		if (robot->state.edp.is_synchronised) {

			for (int i = 0; i < robot->number_of_servos; i++) {
				robot->desired_pos[i] = doubleSpinBox_m_absolute_Vector[i]->value();
			}
		} else {

			for (int i = 0; i < robot->number_of_servos; i++) {
				robot->desired_pos[i] = 0.0;
			}
		}
	}
	return 1;
}

int wgt_smb_command::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot->state.edp.pid != -1) {

			lib::epos::EPOS_MOTION_VARIANT motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;
			/*
			 motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;
			 motion_variant = lib::epos::SYNC_TRAPEZOIDAL;
			 motion_variant = lib::epos::SYNC_POLYNOMIAL;
			 motion_variant = lib::epos::OPERATIONAL;
			 */

			if (ui.radioButton_m_motor->isChecked()) {
				robot->ui_ecp_robot->move_motors(robot->desired_pos, motion_variant);
			} else if (ui.radioButton_m_joint->isChecked()) {
				robot->ui_ecp_robot->move_joints(robot->desired_pos, motion_variant);
			} else if (ui.radioButton_m_ext->isChecked()) {
				robot->ui_ecp_robot->move_external(robot->desired_pos, motion_variant, 10);
			}

			if ((robot->state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot->state.edp.is_synchronised)
				for (int i = 0; i < robot->number_of_servos; i++) {
					doubleSpinBox_m_absolute_Vector[i]->setValue(robot->desired_pos[i]);
				}

				init();
			}
		} // end if (robot->state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI_PTR

	return 1;
}

// buttons callbacks

void wgt_smb_command::on_pushButton_fl_execute_clicked()
{
	try {

		lib::smb::festo_command_td &fc = robot->ui_ecp_robot->the_robot->smb_festo_command_data_port.data;

		// dla kazdej z nog
		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
			// wybierz wariant

			if (radioButton_fl_in_Vector[i]->isChecked()) {
				fc.leg[i] = lib::smb::IN;
			} else if (radioButton_fl_out_Vector[i]->isChecked()) {
				fc.leg[i] = lib::smb::OUT;
			}

			fc.undetachable[i] = false;
			if (checkBox_fl_undetachable_Vector[i]->isChecked()) {
				if (ui.checkBox_fl_all_undetachable->isChecked()) {
					fc.undetachable[i] = true;
				} else {
					interface.ui_msg->message(lib::NON_FATAL_ERROR, "special undetachable mode not set");
				}
			}

		}
		robot->ui_ecp_robot->the_robot->smb_festo_command_data_port.set();
		robot->ui_ecp_robot->execute_motion();

		init();

	} // end try
	CATCH_SECTION_UI_PTR

}

void wgt_smb_command::on_pushButton_fl_all_in_clicked()
{
// dla kazdej z nog
	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		// wybierz wariant
		radioButton_fl_in_Vector[i]->setChecked(true);
	}
}

void wgt_smb_command::on_pushButton_fl_all_out_clicked()
{
// dla kazdej z nog
	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		// wybierz wariant
		radioButton_fl_out_Vector[i]->setChecked(true);
	}
}

void wgt_smb_command::on_pushButton_m_execute_clicked()
{
	get_desired_position();
	move_it();
}

void wgt_smb_command::on_pushButton_read_clicked()
{
	init();
}

void wgt_smb_command::on_pushButton_ml_copy_clicked()
{
	ui.doubleSpinBox_ml_absolute->setValue(ui.doubleSpinBox_ml_current_position->value());
}

void wgt_smb_command::on_pushButton_ms_copy_clicked()
{
	ui.doubleSpinBox_ms_absolute->setValue(ui.doubleSpinBox_ms_current_position->value());
}

void wgt_smb_command::on_pushButton_ml_left_clicked()
{
	get_desired_position();
	robot->desired_pos[0] -= doubleSpinBox_m_relative_Vector[0]->value();
	move_it();
}

void wgt_smb_command::on_pushButton_ml_rigth_clicked()
{
	get_desired_position();
	robot->desired_pos[0] += doubleSpinBox_m_relative_Vector[0]->value();
	move_it();
}

void wgt_smb_command::on_pushButton_ms_left_clicked()
{
	get_desired_position();
	robot->desired_pos[1] -= doubleSpinBox_m_relative_Vector[1]->value();
	move_it();
}

void wgt_smb_command::on_pushButton_ms_rigth_clicked()
{
	get_desired_position();
	robot->desired_pos[1] += doubleSpinBox_m_relative_Vector[1]->value();
	move_it();
}

void wgt_smb_command::on_pushButton_stop_clicked()
{
	interface.ui_msg->message("on_pushButton_stop_clicked");
	robot->execute_stop_motor();
}

void wgt_smb_command::on_radioButton_m_motor_toggled()
{
	if (ui.radioButton_m_motor->isChecked()) {
		//	interface.ui_msg->message("on_radioButton_m_motor_clicked");

		ui.doubleSpinBox_ml_absolute->setMinimum(-100000);
		ui.doubleSpinBox_ml_absolute->setMaximum(100000);
		ui.doubleSpinBox_ml_absolute->setSingleStep(1000);
		ui.doubleSpinBox_ml_absolute->setDecimals(0);

		ui.doubleSpinBox_ml_relative->setMinimum(-100000);
		ui.doubleSpinBox_ml_relative->setMaximum(100000);
		ui.doubleSpinBox_ml_relative->setSingleStep(1000);
		ui.doubleSpinBox_ml_relative->setDecimals(0);

		ui.doubleSpinBox_ms_absolute->setMinimum(-120000);
		ui.doubleSpinBox_ms_absolute->setMaximum(120000);
		ui.doubleSpinBox_ms_absolute->setSingleStep(1000);
		ui.doubleSpinBox_ms_absolute->setDecimals(0);

		ui.doubleSpinBox_ms_relative->setMinimum(-120000);
		ui.doubleSpinBox_ms_relative->setMaximum(120000);
		ui.doubleSpinBox_ms_relative->setSingleStep(1000);
		ui.doubleSpinBox_ms_relative->setDecimals(0);

		// Set precision of widgets with current positions.
		ui.doubleSpinBox_ml_current_position->setDecimals(0);
		ui.doubleSpinBox_ms_current_position->setDecimals(0);

		init();

		on_pushButton_ml_copy_clicked();
		on_pushButton_ms_copy_clicked();
	}
}

void wgt_smb_command::on_radioButton_m_joint_toggled()
{
	if (ui.radioButton_m_joint->isChecked()) {
		//	interface.ui_msg->message("on_radioButton_m_joint_clicked");

		ui.doubleSpinBox_ml_absolute->setMinimum(-3.1415);
		ui.doubleSpinBox_ml_absolute->setMaximum(3.1415);
		ui.doubleSpinBox_ml_absolute->setSingleStep(0.1);
		ui.doubleSpinBox_ml_absolute->setDecimals(3);

		ui.doubleSpinBox_ml_relative->setMinimum(-3.1415);
		ui.doubleSpinBox_ml_relative->setMaximum(3.1415);
		ui.doubleSpinBox_ml_relative->setSingleStep(0.1);
		ui.doubleSpinBox_ml_relative->setDecimals(3);

		ui.doubleSpinBox_ms_absolute->setMinimum(-3.1415);
		ui.doubleSpinBox_ms_absolute->setMaximum(3.1415);
		ui.doubleSpinBox_ms_absolute->setSingleStep(0.1);
		ui.doubleSpinBox_ms_absolute->setDecimals(3);

		ui.doubleSpinBox_ms_relative->setMinimum(-3.1415);
		ui.doubleSpinBox_ms_relative->setMaximum(3.1415);
		ui.doubleSpinBox_ms_relative->setSingleStep(0.1);
		ui.doubleSpinBox_ms_relative->setDecimals(3);

		// Set precision of widgets with current positions.
		ui.doubleSpinBox_ml_current_position->setDecimals(3);
		ui.doubleSpinBox_ms_current_position->setDecimals(3);

		init();

		on_pushButton_ml_copy_clicked();
		on_pushButton_ms_copy_clicked();
	}
}

void wgt_smb_command::on_radioButton_m_ext_toggled()
{
	if (ui.radioButton_m_ext->isChecked()) {
		//	interface.ui_msg->message("on_radioButton_m_ext_clicked");

		ui.doubleSpinBox_ml_absolute->setMinimum(-100000);
		ui.doubleSpinBox_ml_absolute->setMaximum(100000);
		ui.doubleSpinBox_ml_absolute->setSingleStep(1);
		ui.doubleSpinBox_ml_absolute->setDecimals(0);

		ui.doubleSpinBox_ml_relative->setMinimum(-100000);
		ui.doubleSpinBox_ml_relative->setMaximum(100000);
		ui.doubleSpinBox_ml_relative->setSingleStep(1);
		ui.doubleSpinBox_ml_relative->setDecimals(0);

		ui.doubleSpinBox_ms_absolute->setMinimum(-3.1415);
		ui.doubleSpinBox_ms_absolute->setMaximum(3.1415);
		ui.doubleSpinBox_ms_absolute->setSingleStep(0.1);
		ui.doubleSpinBox_ms_absolute->setDecimals(3);

		ui.doubleSpinBox_ms_relative->setMinimum(-3.1415);
		ui.doubleSpinBox_ms_relative->setMaximum(3.1415);
		ui.doubleSpinBox_ms_relative->setSingleStep(0.1);
		ui.doubleSpinBox_ms_relative->setDecimals(3);

		// Set precision of widgets with current positions.
		ui.doubleSpinBox_ml_current_position->setDecimals(0);
		ui.doubleSpinBox_ms_current_position->setDecimals(3);

		init();

		on_pushButton_ml_copy_clicked();
		on_pushButton_ms_copy_clicked();
	}
}

// events

void wgt_smb_command::showEvent(QShowEvent * event)
{
//	emit gotFocus();

	init();
}

