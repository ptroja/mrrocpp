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

	checkBox_fl_up_Vector.append(ui.checkBox_fl1_up);
	checkBox_fl_up_Vector.append(ui.checkBox_fl2_up);
	checkBox_fl_up_Vector.append(ui.checkBox_fl3_up);

	checkBox_fl_down_Vector.append(ui.checkBox_fl1_down);
	checkBox_fl_down_Vector.append(ui.checkBox_fl2_down);
	checkBox_fl_down_Vector.append(ui.checkBox_fl3_down);

	checkBox_m_mip_Vector.append(ui.checkBox_ml_mip);
	checkBox_m_mip_Vector.append(ui.checkBox_ms_mip);

	checkBox_m_no_Vector.append(ui.checkBox_ml_no);
	checkBox_m_no_Vector.append(ui.checkBox_ms_no);

	radioButton_fl_no_Vector.append(ui.radioButton_fl1_no);
	radioButton_fl_no_Vector.append(ui.radioButton_fl2_no);
	radioButton_fl_no_Vector.append(ui.radioButton_fl3_no);

	radioButton_fl_up_Vector.append(ui.radioButton_fl1_up);
	radioButton_fl_up_Vector.append(ui.radioButton_fl2_up);
	radioButton_fl_up_Vector.append(ui.radioButton_fl3_up);

	radioButton_fl_down_Vector.append(ui.radioButton_fl1_down);
	radioButton_fl_down_Vector.append(ui.radioButton_fl2_down);
	radioButton_fl_down_Vector.append(ui.radioButton_fl3_down);

	radioButton_fl_detach_Vector.append(ui.radioButton_fl1_detach);
	radioButton_fl_detach_Vector.append(ui.radioButton_fl2_detach);
	radioButton_fl_detach_Vector.append(ui.radioButton_fl3_detach);

	doubleSpinBox_m_current_position_Vector.append(ui.doubleSpinBox_ml_current_position);
	doubleSpinBox_m_current_position_Vector.append(ui.doubleSpinBox_ms_current_position);

	doubleSpinBox_m_absolute_Vector.append(ui.doubleSpinBox_ml_absolute);
	doubleSpinBox_m_absolute_Vector.append(ui.doubleSpinBox_ms_absolute);

	doubleSpinBox_m_relative_Vector.append(ui.doubleSpinBox_ml_relative);
	doubleSpinBox_m_relative_Vector.append(ui.doubleSpinBox_ms_relative);

	// podpiecie sygnalow do slotow
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
	/*
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
	 */
}

int wgt_smb_command::init()
{
	/*
	 try {

	 if (robot->state.edp.pid != -1) {
	 if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
	 {
	 synchro_depended_widgets_disable(false);

	 robot->ui_ecp_robot->the_robot->epos_reply_data_request_port.set_request();
	 robot->ui_ecp_robot->execute_motion();
	 robot->ui_ecp_robot->the_robot->epos_reply_data_request_port.get();

	 for (int i = 0; i < robot->number_of_servos; i++) {
	 set_single_axis(i, doubleSpinBox_mcur_Vector[i], doubleSpinBox_cur_Vector[i], radioButton_mip_Vector[i]);
	 robot->desired_pos[i] = robot->current_pos[i];
	 }

	 } else {
	 // Wygaszanie elementow przy niezsynchronizowanym robocie
	 synchro_depended_widgets_disable(true);
	 }
	 }

	 } // end try
	 CATCH_SECTION_UI_PTR
	 */
	return 1;
}

// buttons callbacks

void wgt_smb_command::on_pushButton_fl_execute_clicked()
{

}

void wgt_smb_command::on_pushButton_m_execute_clicked()
{

}

void wgt_smb_command::on_pushButton_execute_all_clicked()
{

}

void wgt_smb_command::on_pushButton_read_clicked()
{

}

void wgt_smb_command::on_pushButton_ml_copy_clicked()
{

}

void wgt_smb_command::on_pushButton_ms_copy_clicked()
{

}

void wgt_smb_command::on_pushButton_ml_left_clicked()
{

}

void wgt_smb_command::on_pushButton_ml_rigth_clicked()
{

}

void wgt_smb_command::on_pushButton_ms_left_clicked()
{

}
void wgt_smb_command::on_pushButton_ms_rigth_clicked()
{

}
