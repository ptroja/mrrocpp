#include "wgt_input_integer.h"
#include "../interface.h"
#include "../ui_ecp.h"

wgt_input_integer::wgt_input_integer(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Yes No Dialog", _interface, parent), ui(new Ui::wgt_input_integerClass)
{
	ui->setupUi(this);

}

wgt_input_integer::~wgt_input_integer()
{

}

void wgt_input_integer::hideEvent(QHideEvent *event)
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;

	}
	interface.ui_ecp_obj->synchroniser.command();
	interface.ui_msg->message("wgt_input_integer::hideEvent");
	event->accept();
}

void wgt_input_integer::on_pushButton_yes_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_input_integer::on_pushButton_no_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_NO;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

Ui::wgt_input_integerClass * wgt_input_integer::get_ui()
{
	return ui;
}
