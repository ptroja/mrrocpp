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

void wgt_message::my_open(bool set_on_top)
{
	ui->label_message->setText(interface.ui_ecp_obj->ecp_to_ui_msg.string);
	wgt_base::my_open(set_on_top);
}
