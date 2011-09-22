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
}

wgt_smb_command::~wgt_smb_command()
{

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
