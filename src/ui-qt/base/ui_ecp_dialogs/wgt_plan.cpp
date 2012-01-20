#include <QHideEvent>

#include "wgt_plan.h"
#include "../interface.h"
#include "../ui_ecp.h"

#include "application/swarmitfix_plan/plan.hxx"

wgt_plan::wgt_plan(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
		wgt_base("Plan control", _interface, parent), ui(new Ui::wgt_planClass)
{
	ui->setupUi(this);

	ui->pushButton_prev->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/media-seek-backward-32.png"));
	ui->pushButton_next->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/media-seek-forward-32.png"));
	ui->pushButton_reload->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/refresh-32.png"));
	ui->pushButton_save->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/standardbutton-save-32.png"));
	ui->pushButton_exec->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/standardbutton-apply-32.png"));
}

wgt_plan::~wgt_plan()
{
	delete ui;
}

void wgt_plan::hideEvent(QHideEvent *event)
{
	//interface.ui_msg->message("wgt_plan::hideEvent");
	event->accept();
}

void wgt_plan::reload()
{
	// Use local reference with short name.
	const lib::ECP_message & request = interface.ui_ecp_obj->ecp_to_ui_msg;

	std::cout << "reload()" << std::endl
			<< request.plan_item_string << std::endl;

	// Extract data from string.
	std::istringstream istr(request.plan_item_string);
	boost::archive::text_iarchive ia(istr);
	xml_schema::istream<boost::archive::text_iarchive> is (ia);

	// Handle current plan item
	switch(request.plan_item_type) {
		case lib::PKM_AND_HEAD:
			{
				// Deserialize data.
				Pkm::ItemType item(is);

				// Check widget constraints.
				checkInputWidgetLimits(*ui->agent_input, item.agent());
				checkInputWidgetLimits(*ui->TBeg_input, item.TBeg());
				checkInputWidgetLimits(*ui->TEnd_input, item.TEnd());
				checkInputWidgetLimits(*ui->ind_input, item.ind());

				checkInputWidgetLimits(*ui->x_input, item.Xyz_Euler_Zyz()->x());
				checkInputWidgetLimits(*ui->y_input, item.Xyz_Euler_Zyz()->y());
				checkInputWidgetLimits(*ui->z_input, item.Xyz_Euler_Zyz()->z());
				checkInputWidgetLimits(*ui->alpha_input, item.Xyz_Euler_Zyz()->alpha());
				checkInputWidgetLimits(*ui->beta_input, item.Xyz_Euler_Zyz()->beta());
				checkInputWidgetLimits(*ui->gamma_input, item.Xyz_Euler_Zyz()->gamma());
				checkInputWidgetLimits(*ui->head_input, item.beta7());

				// Setup common frame.
				ui->agent_input->setValue(item.agent());
				ui->TBeg_input->setValue(item.TBeg());
				ui->TEnd_input->setValue(item.TEnd());
				ui->ind_input->setValue(item.ind());

				// Setup input widgets.
				ui->x_input->setValue(item.Xyz_Euler_Zyz()->x());
				ui->y_input->setValue(item.Xyz_Euler_Zyz()->y());
				ui->z_input->setValue(item.Xyz_Euler_Zyz()->z());
				ui->alpha_input->setValue(item.Xyz_Euler_Zyz()->alpha());
				ui->beta_input->setValue(item.Xyz_Euler_Zyz()->beta());
				ui->gamma_input->setValue(item.Xyz_Euler_Zyz()->gamma());
				ui->head_input->setValue(item.beta7());
			}

			// Disable/enable input containers
			this->setEnabled(true);
			ui->pkm_frame->setEnabled(true);
			ui->mbase_frame->setEnabled(false);

			break;
		case lib::MBASE_AND_BENCH:
			{
				// Deserialize data.
				Mbase::ItemType item(is);

				// Check widget constraints.
				checkInputWidgetLimits(*ui->agent_input, item.agent());
				checkInputWidgetLimits(*ui->TBeg_input, item.TBeg());
				checkInputWidgetLimits(*ui->TEnd_input, item.TEnd());
				checkInputWidgetLimits(*ui->ind_input, item.ind());

				checkInputWidgetLimits(*ui->row1_input, item.pinIndices().item().at(0).row());
				checkInputWidgetLimits(*ui->column1_input, item.pinIndices().item().at(0).column());
				checkInputWidgetLimits(*ui->row2_input, item.pinIndices().item().at(1).row());
				checkInputWidgetLimits(*ui->column2_input, item.pinIndices().item().at(1).column());
				checkInputWidgetLimits(*ui->row3_input, item.pinIndices().item().at(2).row());
				checkInputWidgetLimits(*ui->column3_input, item.pinIndices().item().at(2).column());

				checkInputWidgetLimits(*ui->pin_input, item.actions().item().front().pin());
				checkInputWidgetLimits(*ui->dThetaInd_input, item.actions().item().front().dThetaInd());
				checkInputWidgetLimits(*ui->dPkmTheta_input, item.actions().item().front().dPkmTheta());

				// Setup common frame
				ui->agent_input->setValue(item.agent());
				ui->TBeg_input->setValue(item.TBeg());
				ui->TEnd_input->setValue(item.TEnd());
				ui->ind_input->setValue(item.ind());

				// Setup input widgets
				ui->row1_input->setValue(item.pinIndices().item().at(0).row());
				ui->column1_input->setValue(item.pinIndices().item().at(0).column());
				ui->row2_input->setValue(item.pinIndices().item().at(1).row());
				ui->column2_input->setValue(item.pinIndices().item().at(1).column());
				ui->row3_input->setValue(item.pinIndices().item().at(2).row());
				ui->column3_input->setValue(item.pinIndices().item().at(2).column());

				// FIXME: we do not support dynamic action lists.
				assert(item.actions().item().size() == 1);

				ui->pin_input->setValue(item.actions().item().front().pin());
				ui->dThetaInd_input->setValue(item.actions().item().front().dThetaInd());

				// Disable rotation input widget if rotation pin is set to zero.
				ui->dThetaInd_input->setEnabled(item.actions().item().front().pin() ? true : false);

				ui->dPkmTheta_input->setValue(item.actions().item().front().dPkmTheta());
			}

			// Disable/enable input containers
			this->setEnabled(true);
			ui->pkm_frame->setEnabled(false);
			ui->mbase_frame->setEnabled(true);

			break;
		default:
			assert(0);
			break;
	}
}

void wgt_plan::my_open(bool set_on_top)
{
	// Handle current plan item
	reload();

	wgt_base::my_open(set_on_top);
}

void wgt_plan::on_pushButton_prev_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_PREV;

	reply();
}

void wgt_plan::on_pushButton_next_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_NEXT;

	reply();
}

void wgt_plan::on_pushButton_exec_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_EXEC;

	// Use local references with short names.
	const lib::ECP_message & request = interface.ui_ecp_obj->ecp_to_ui_msg;
	lib::UI_reply & reply = interface.ui_ecp_obj->ui_rep;

	// Extract data from string.
	std::istringstream istr(request.plan_item_string);
	boost::archive::text_iarchive ia(istr);
	xml_schema::istream<boost::archive::text_iarchive> is (ia);

	// String for serialized data.
	std::ostringstream ostr;
	boost::archive::text_oarchive oa(ostr);
	xml_schema::ostream<boost::archive::text_oarchive> os(oa);

	// Copy the read-only part.
	reply.plan_item_type = request.plan_item_type;

	// Sent back the current item.
	switch(reply.plan_item_type) {
		case lib::PKM_AND_HEAD:
			{
				// Deserialize data.
				Pkm::ItemType item(is);

				// Setup input widgets
				item.Xyz_Euler_Zyz()->x() = ui->x_input->value();
				item.Xyz_Euler_Zyz()->y() = ui->y_input->value();
				item.Xyz_Euler_Zyz()->z() = ui->z_input->value();
				item.Xyz_Euler_Zyz()->alpha() = ui->alpha_input->value();
				item.Xyz_Euler_Zyz()->beta() = ui->beta_input->value();
				item.Xyz_Euler_Zyz()->gamma() = ui->gamma_input->value();
				item.beta7() = ui->head_input->value();

				// serialize data
				os << item;
			}

			break;
		case lib::MBASE_AND_BENCH:
			{
				// Deserialize data.
				Mbase::ItemType item(is);

				// Setup input widgets data.
				item.pinIndices().item().at(0).row() = ui->row1_input->value();
				item.pinIndices().item().at(0).column() = ui->column1_input->value();
				item.pinIndices().item().at(1).row() = ui->row2_input->value();
				item.pinIndices().item().at(1).column() = ui->column2_input->value();
				item.pinIndices().item().at(2).row() = ui->row3_input->value();
				item.pinIndices().item().at(2).column() = ui->column3_input->value();

				item.actions().item().front().pin() = ui->pin_input->value();
				item.actions().item().front().dThetaInd() = ui->dThetaInd_input->value();
				item.actions().item().front().dPkmTheta() = ui->dPkmTheta_input->value();

				// serialize data
				os << item;
			}

			break;
		default:
			assert(0);
			break;
	}

	// Copy string to the reply buffer.
	reply.plan_item_string = ostr.str();

	this->reply();
}

void wgt_plan::on_pushButton_save_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_SAVE;

	reply();
}

void wgt_plan::on_pushButton_reload_clicked()
{
	reload();
}

void wgt_plan::on_pin_input_valueChanged(int i)
{
	// Disable rotation input widget when the pin is set to zero.
	ui->dThetaInd_input->setEnabled(i != 0);
}

void wgt_plan::reply()
{
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();

	this->setEnabled(false);
}
