#include "wgt_teaching.h"
#include "../interface.h"
#include "../ui_ecp.h"

#include "../../irp6ot_m/ui_r_irp6ot_m.h"
#include "../../irp6p_m/ui_r_irp6p_m.h"

wgt_teaching::wgt_teaching(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Teaching Dialog", _interface, parent), ui(new Ui::wgt_teachingClass)
{
	ui->setupUi(this);

}

wgt_teaching::~wgt_teaching()
{
	delete ui;
}

void wgt_teaching::hideEvent(QHideEvent *event)
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();
	event->accept();
}

void wgt_teaching::on_pushButton_send_move_clicked()
{

	if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		for (int i = 0; i < lib::irp6ot_m::NUM_OF_SERVOS; i++)
			interface.ui_ecp_obj->ui_rep.coordinates[i] = interface.irp6ot_m->current_pos[i];
	} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
		for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++)
			interface.ui_ecp_obj->ui_rep.coordinates[i] = interface.irp6p_m->current_pos[i];
	}

	interface.ui_ecp_obj->ui_rep.double_number = ui->doubleSpinBox_input->value();
	interface.ui_ecp_obj->ui_rep.reply = lib::NEXT;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;
	interface.ui_ecp_obj->synchroniser.command();
}

void wgt_teaching::on_pushButton_end_motion_clicked()
{
	interface.teachingstate = ui::common::MP_RUNNING;
	interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;

	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

Ui::wgt_teachingClass * wgt_teaching::get_ui()
{
	return ui;
}
