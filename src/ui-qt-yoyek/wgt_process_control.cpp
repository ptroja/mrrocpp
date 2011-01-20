#include "wgt_process_control.h"
#include "interface.h"

wgt_process_control::wgt_process_control(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QWidget(parent), interface(_interface)
{
	ui.setupUi(this);
}

wgt_process_control::~wgt_process_control()
{

}
