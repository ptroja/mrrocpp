//#include "ui_ecp_r_sarkofag.h"
#include "ui_r_sarkofag.h"
#include "robot/spkm/const_spkm.h"

#include "wgt_sarkofag_inc.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"

wgt_sarkofag_inc::wgt_sarkofag_inc(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::sarkofag::UiRobot& _robot, QWidget *parent) :
	wgt_base("Spkm incremental motion", _interface, parent), robot(_robot)
{
	ui.setupUi(this);
}

wgt_sarkofag_inc::~wgt_sarkofag_inc()
{

}


