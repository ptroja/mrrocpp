#include "ui_ecp_r_spkm.h"
#include "ui_r_spkm.h"
#include "robot/spkm/const_spkm.h"

#include "wnd_spkm_inc.h"
#include "../interface.h"

wnd_spkm_inc::wnd_spkm_inc(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::spkm::UiRobot& _robot, QWidget *parent) :
	QMainWindow(parent), interface(_interface), robot(_robot)
{
	ui.setupUi(this);
}

wnd_spkm_inc::~wnd_spkm_inc()
{

}
