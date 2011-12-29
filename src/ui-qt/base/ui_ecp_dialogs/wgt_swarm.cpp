#include "wgt_swarm.h"
#include "../interface.h"
#include "../ui_ecp.h"
#include <QHideEvent>

wgt_swarm::wgt_swarm(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
		wgt_base("Yes No Dialog", _interface, parent), ui(new Ui::wgt_swarmClass)
{
	ui->setupUi(this);

}

wgt_swarm::~wgt_swarm()
{
	delete ui;
}

void wgt_swarm::hideEvent(QHideEvent *event)
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;

	}
	interface.ui_ecp_obj->synchroniser.command();
	//interface.ui_msg->message("wgt_swarm::hideEvent");
	event->accept();
}

void wgt_swarm::on_pushButton_yes_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_swarm::on_pushButton_no_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_NO;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_swarm::my_open(bool set_on_top)
{
	ui->label_message->setText(interface.ui_ecp_obj->ecp_to_ui_msg.string);
	wgt_base::my_open(set_on_top);
}

