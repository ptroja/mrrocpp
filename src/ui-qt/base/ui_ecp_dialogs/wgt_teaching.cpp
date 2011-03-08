#include "wgt_teaching.h"
#include "../interface.h"
#include "../ui_ecp.h"

wgt_teaching::wgt_teaching(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Teaching Dialog", _interface, parent), ui(new Ui::wgt_teachingClass)
{
	ui->setupUi(this);

}

wgt_teaching::~wgt_teaching()
{

}

void wgt_teaching::hideEvent(QHideEvent *event)
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
		interface.ui_ecp_obj->ui_rep.double_number = 0.0;
	}
	interface.ui_ecp_obj->synchroniser.command();
	event->accept();
}

void wgt_teaching::on_pushButton_send_move_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	interface.ui_ecp_obj->ui_rep.double_number = ui->doubleSpinBox_input->value();
	my_close();
}

void wgt_teaching::on_pushButton_end_motion_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	interface.ui_ecp_obj->ui_rep.double_number = 0.0;
	my_close();
}

Ui::wgt_teachingClass * wgt_teaching::get_ui()
{
	return ui;
}
