#include "wgt_yes_no.h"
#include "../interface.h"
#include "../ui_ecp.h"

wgt_yes_no::wgt_yes_no(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Yes No Dialog", _interface, parent), ui(new Ui::wgt_yes_noClass)
{
	ui->setupUi(this);

}

wgt_yes_no::~wgt_yes_no()
{
	delete ui;
}

void wgt_yes_no::hideEvent(QHideEvent *event)
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;

	}
	interface.ui_ecp_obj->synchroniser.command();
	//interface.ui_msg->message("wgt_yes_no::hideEvent");
	event->accept();
}

void wgt_yes_no::on_pushButton_yes_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_yes_no::on_pushButton_no_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_NO;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

Ui::wgt_yes_noClass * wgt_yes_no::get_ui()
{
	return ui;
}
