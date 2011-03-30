#include "wgt_input_double.h"
#include "../interface.h"
#include "../ui_ecp.h"

wgt_input_double::wgt_input_double(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Input Double Dialog", _interface, parent), ui(new Ui::wgt_input_doubleClass)
{
	ui->setupUi(this);

}

wgt_input_double::~wgt_input_double()
{
	delete ui;
}

void wgt_input_double::hideEvent(QHideEvent *event)
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
		interface.ui_ecp_obj->ui_rep.double_number = 0.0;
	}
	interface.ui_ecp_obj->synchroniser.command();
	event->accept();
}

void wgt_input_double::on_pushButton_ok_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	interface.ui_ecp_obj->ui_rep.double_number = ui->doubleSpinBox_input->value();
	my_close();
}

void wgt_input_double::on_pushButton_cancel_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	interface.ui_ecp_obj->ui_rep.double_number = 0.0;
	my_close();
}

Ui::wgt_input_doubleClass * wgt_input_double::get_ui()
{
	return ui;
}
