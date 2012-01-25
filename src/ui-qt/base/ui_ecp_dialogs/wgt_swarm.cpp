#include <QHideEvent>
#include <QtXml/QXmlSimpleReader>

#include <boost/shared_ptr.hpp>

#include "wgt_swarm.h"
#include "../interface.h"
#include "../ui_ecp.h"

#include "xmlsyntaxhighlighter.h"

wgt_swarm::wgt_swarm(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
		wgt_base("Swarm control", _interface, parent), ui(new Ui::wgt_swarmClass)
{
	ui->setupUi(this);

	ui->pushButton_prev->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/media-seek-backward-32.png"));
	ui->pushButton_next->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/media-seek-forward-32.png"));
	ui->pushButton_reload->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/refresh-32.png"));
	ui->pushButton_save->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/standardbutton-save-32.png"));
	ui->pushButton_exec->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/standardbutton-apply-32.png"));

	// Setup syntax highlighter
	highlighter = (boost::shared_ptr<XmlSyntaxHighlighter>) new XmlSyntaxHighlighter(ui->textEdit->document());
}

wgt_swarm::~wgt_swarm()
{
	delete ui;
}

void wgt_swarm::hideEvent(QHideEvent *event)
{
	//interface.ui_msg->message("wgt_swarm::hideEvent");
	event->accept();
}

void wgt_swarm::my_open(bool set_on_top)
{
	ui->label_message->setText(interface.ui_ecp_obj->ecp_to_ui_msg.string);

	stored_plan_item = interface.ui_ecp_obj->ecp_to_ui_msg.plan_item_string;

	ui->textEdit->setPlainText(interface.ui_ecp_obj->ecp_to_ui_msg.plan_item_string.c_str());

	this->setEnabled(true);

	wgt_base::my_open(set_on_top);
}

void wgt_swarm::on_pushButton_prev_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_PREV;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	reply();
}

void wgt_swarm::on_pushButton_next_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_NEXT;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	reply();
}

void wgt_swarm::on_pushButton_exec_clicked()
{
	if(validate()) {
		interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_EXEC;

		interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;
		interface.ui_ecp_obj->ui_rep.plan_item_string = ui->textEdit->toPlainText().toStdString();

		reply();
	} else {
		interface.ui_msg->message(lib::NON_FATAL_ERROR, "plan item validation failed");
	}
}

void wgt_swarm::on_pushButton_save_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_SAVE;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	reply();
}

void wgt_swarm::on_pushButton_reload_clicked()
{
	ui->textEdit->setPlainText(stored_plan_item.c_str());
}

bool wgt_swarm::validate()
{
	// Setup text to XML adapter
	QXmlInputSource source;
	source.setData(ui->textEdit->toPlainText());

	// Set parser
	QXmlSimpleReader reader;

	// Return parsing status
	return reader.parse(source);
}

void wgt_swarm::reply()
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();

	this->setEnabled(false);
}
