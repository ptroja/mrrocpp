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

void wgt_swarm::my_open(bool set_on_top)
{
	ui->label_message->setText(interface.ui_ecp_obj->ecp_to_ui_msg.string);

	strcpy(stored_plan_item, interface.ui_ecp_obj->ecp_to_ui_msg.plan_item);

	ui->textEdit->setText(interface.ui_ecp_obj->ecp_to_ui_msg.plan_item);

	wgt_base::my_open(set_on_top);
}

void wgt_swarm::on_pushButton_prev_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_PREV;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_swarm::on_pushButton_next_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_NEXT;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_swarm::on_pushButton_exec_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_EXEC;

	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;
	strcpy(interface.ui_ecp_obj->ui_rep.plan_item, ui->textEdit->toPlainText().toStdString().c_str());
	my_close();
}

void wgt_swarm::on_pushButton_save_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_SAVE;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_swarm::on_pushButton_reload_clicked()
{
	ui->textEdit->setText(stored_plan_item);
}

