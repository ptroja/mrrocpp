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

