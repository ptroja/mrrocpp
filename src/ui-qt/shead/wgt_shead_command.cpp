#include "ui_ecp_r_shead.h"
#include "ui_r_shead.h"
#include "robot/shead/const_shead.h"

#include "wgt_shead_command.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

wgt_shead_command::wgt_shead_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		wgt_base(_widget_label, _interface, parent)
{
	ui.setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::shead::UiRobot *>(_robot);

	// utworzenie list widgetow

	checkBox_m_mip_Vector.append(ui.checkBox_ml_mip);

	checkBox_contacts_Vector.append(ui.checkBox_con_p1);
	checkBox_contacts_Vector.append(ui.checkBox_con_p2);
	checkBox_contacts_Vector.append(ui.checkBox_con_p3);

	doubleSpinBox_m_current_position_Vector.append(ui.doubleSpinBox_ml_current_position);

	doubleSpinBox_m_absolute_Vector.append(ui.doubleSpinBox_ml_absolute);

	doubleSpinBox_m_relative_Vector.append(ui.doubleSpinBox_ml_relative);

	// uruchomienei timera
	timer = (boost::shared_ptr <QTimer>) new QTimer(this);
	connect(timer.get(), SIGNAL(timeout()), this, SLOT(timer_slot()));
	timer->start(interface.position_refresh_interval);

	// podpiecie pozostalych sygnalow do slotow
	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);

}

wgt_shead_command::~wgt_shead_command()
{

}

void wgt_shead_command::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_shead_command::synchro_depended_init_slot()
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

int wgt_shead_command::init()
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

				robot->ui_ecp_robot->the_robot->shead_reply_data_request_port.set_request();
				robot->ui_ecp_robot->execute_motion();
				robot->ui_ecp_robot->the_robot->shead_reply_data_request_port.get();

				lib::epos::epos_reply *er;

				if (ui.radioButton_m_motor->isChecked()) {
					robot->ui_ecp_robot->the_robot->epos_motor_reply_data_request_port.get();
					er = &robot->ui_ecp_robot->the_robot->epos_motor_reply_data_request_port.data;
				} else if (ui.radioButton_m_joint->isChecked()) {
					robot->ui_ecp_robot->the_robot->epos_joint_reply_data_request_port.get();
					er = &robot->ui_ecp_robot->the_robot->epos_joint_reply_data_request_port.data;
				} else if (ui.radioButton_m_ext->isChecked()) {
					robot->ui_ecp_robot->the_robot->epos_external_reply_data_request_port.get();
					er = &robot->ui_ecp_robot->the_robot->epos_external_reply_data_request_port.data;
				}

				lib::shead::reply &rep = robot->ui_ecp_robot->the_robot->shead_reply_data_request_port.data;

				// sets soldification state
				switch (rep.soldification_state)
				{
					case lib::shead::SOLDIFICATION_STATE_ON: {
						ui.checkBox_sol_on->setChecked(true);
						ui.checkBox_sol_off->setChecked(false);
						ui.checkBox_sol_int->setChecked(false);
					}
						break;
					case lib::shead::SOLDIFICATION_STATE_OFF: {
						ui.checkBox_sol_on->setChecked(false);
						ui.checkBox_sol_off->setChecked(true);
						ui.checkBox_sol_int->setChecked(false);
					}
						break;
					case lib::shead::SOLDIFICATION_STATE_INTERMEDIATE: {
						ui.checkBox_sol_on->setChecked(false);
						ui.checkBox_sol_off->setChecked(false);
						ui.checkBox_sol_int->setChecked(true);
					}
						break;
					default: {
					}
						break;
				}

				// sets vacumization state
				switch (rep.vacuum_state)
				{
					case lib::shead::VACUUM_STATE_ON: {
						ui.checkBox_vac_on->setChecked(true);
						ui.checkBox_vac_off->setChecked(false);
						ui.checkBox_vac_int->setChecked(false);
					}
						break;
					case lib::shead::VACUUM_STATE_OFF: {
						ui.checkBox_vac_on->setChecked(false);
						ui.checkBox_vac_off->setChecked(true);
						ui.checkBox_vac_int->setChecked(false);
					}
						break;
					case lib::shead::VACUUM_STATE_INTERMEDIATE: {
						ui.checkBox_vac_on->setChecked(false);
						ui.checkBox_vac_off->setChecked(false);
						ui.checkBox_vac_int->setChecked(true);
					}
						break;
					default: {
					}
						break;
				}

				// sets head contact state
				for (int i = 0; i < 3; i++) {
					checkBox_contacts_Vector[i]->setChecked(rep.contacts[i]);
				}

				for (int i = 0; i < lib::shead::NUM_OF_SERVOS; i++) {
					checkBox_m_mip_Vector[i]->setChecked(er->epos_controller[i].motion_in_progress);
					doubleSpinBox_m_current_position_Vector[i]->setValue(er->epos_controller[i].position);
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

int wgt_shead_command::synchro_depended_widgets_disable(bool _set_disabled)
{

	ui.pushButton_m_execute->setDisabled(_set_disabled);
	ui.pushButton_ml_copy->setDisabled(_set_disabled);

	for (int i = 0; i < robot->number_of_servos; i++) {
		doubleSpinBox_m_absolute_Vector[i]->setDisabled(_set_disabled);

	}

	return 1;
}

void wgt_shead_command::timer_slot()
{
	if ((dwgt->isVisible()) && (ui.checkBox_cyclic_read->isChecked())) {
		init();
	}

}

int wgt_shead_command::get_desired_position()
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

int wgt_shead_command::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot->state.edp.pid != -1) {

			lib::epos::EPOS_MOTION_VARIANT motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

			if (ui.radioButton_m_motor->isChecked()) {
				robot->ui_ecp_robot->move_motors(robot->desired_pos, motion_variant);
			} else if (ui.radioButton_m_joint->isChecked()) {
				robot->ui_ecp_robot->move_joints(robot->desired_pos, motion_variant);
			} else if (ui.radioButton_m_ext->isChecked()) {
				robot->ui_ecp_robot->move_external(robot->desired_pos, motion_variant, 10);
			}

			if (robot->state.edp.is_synchronised) { // by Y o dziwo nie dziala poprawnie 	 if (robot->state.edp.is_synchronised)
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

void wgt_shead_command::on_pushButton_sol_execute_clicked()
{
	interface.ui_msg->message("on_pushButton_sol_execute_clicked");

	try {

		lib::shead::SOLIDIFICATION_ACTIVATION &sa =
				robot->ui_ecp_robot->the_robot->shead_head_soldification_data_port.data;

		if (ui.radioButton_sol_activate->isChecked()) {
			sa = lib::shead::SOLIDIFICATION_ON;
		} else if (ui.radioButton_sol_desactivate->isChecked()) {
			sa = lib::shead::SOLIDIFICATION_OFF;
		}

		robot->ui_ecp_robot->the_robot->shead_head_soldification_data_port.set();
		robot->ui_ecp_robot->execute_motion();

		init();

	} // end try
	CATCH_SECTION_UI_PTR

}

void wgt_shead_command::on_pushButton_vac_execute_clicked()
{
	interface.ui_msg->message("on_pushButton_vac_execute_clicked");

	try {

		lib::shead::VACUUM_ACTIVATION &va = robot->ui_ecp_robot->the_robot->shead_vacuum_activation_data_port.data;

		if (ui.radioButton_vac_activate->isChecked()) {
			va = lib::shead::VACUUM_ON;
		} else if (ui.radioButton_vac_desactivate->isChecked()) {
			va = lib::shead::VACUUM_OFF;
		}

		robot->ui_ecp_robot->the_robot->shead_vacuum_activation_data_port.set();
		robot->ui_ecp_robot->execute_motion();

		init();

	} // end try
	CATCH_SECTION_UI_PTR
}

void wgt_shead_command::on_pushButton_m_execute_clicked()
{
	get_desired_position();
	move_it();
}

void wgt_shead_command::on_pushButton_read_clicked()
{
	init();
}

void wgt_shead_command::on_pushButton_ml_copy_clicked()
{
	ui.doubleSpinBox_ml_absolute->setValue(ui.doubleSpinBox_ml_current_position->value());
}

void wgt_shead_command::on_pushButton_ml_left_clicked()
{
	get_desired_position();
	robot->desired_pos[0] -= doubleSpinBox_m_relative_Vector[0]->value();
	move_it();
}

void wgt_shead_command::on_pushButton_ml_rigth_clicked()
{
	get_desired_position();
	robot->desired_pos[0] += doubleSpinBox_m_relative_Vector[0]->value();
	move_it();
}

void wgt_shead_command::on_pushButton_stop_clicked()
{
	interface.ui_msg->message("on_pushButton_stop_clicked");
	robot->execute_stop_motor();
}

void wgt_shead_command::on_radioButton_m_motor_toggled()
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

		// Set precision of widgets with current positions.
		ui.doubleSpinBox_ml_current_position->setDecimals(0);

		init();

		on_pushButton_ml_copy_clicked();
	}
}

void wgt_shead_command::on_radioButton_m_joint_toggled()
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

		// Set precision of widgets with current positions.
		ui.doubleSpinBox_ml_current_position->setDecimals(3);

		init();

		on_pushButton_ml_copy_clicked();
	}
}

void wgt_shead_command::on_radioButton_m_ext_toggled()
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

		// Set precision of widgets with current positions.
		ui.doubleSpinBox_ml_current_position->setDecimals(0);

		init();

		on_pushButton_ml_copy_clicked();
	}
}

// events

void wgt_shead_command::showEvent(QShowEvent * event)
{
//	emit gotFocus();

	init();
}

