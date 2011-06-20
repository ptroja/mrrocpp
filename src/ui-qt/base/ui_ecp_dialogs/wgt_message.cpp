#include "wgt_message.h"
#include "../interface.h"
#include "../ui_ecp.h"

wgt_message::wgt_message(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Message Dialog", _interface, parent), ui(new Ui::wgt_messageClass)
{
	ui->setupUi(this);

}

wgt_message::~wgt_message()
{
	delete ui;
}

Ui::wgt_messageClass * wgt_message::get_ui()
{
	return ui;
}
