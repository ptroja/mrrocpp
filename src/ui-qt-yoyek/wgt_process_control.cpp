#include "wgt_process_control.h"
#include "interface.h"

wgt_process_control::wgt_process_control(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QWidget(parent), interface(_interface)
{
	ui.setupUi(this);

	dwgt = new QDockWidget(interface.mw);
	//dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	dwgt->setWindowTitle("Process control");

	vl = new QVBoxLayout();
	dwgt->setLayout(vl);

	vl->addWidget(this);
	dwgt->setWidget(this);
	dwgt->hide();
	interface.mw->addDockWidget(Qt::LeftDockWidgetArea, dwgt);
}

wgt_process_control::~wgt_process_control()
{

}

void wgt_process_control::on_mp_start_pushButton_clicked()
{
	interface.pulse_start_mp();
}

void wgt_process_control::on_mp_stop_pushButton_clicked()
{
	interface.pulse_stop_mp();
}

void wgt_process_control::on_mp_pause_pushButton_clicked()
{
	interface.pulse_pause_mp();
}

void wgt_process_control::on_mp_resume_pushButton_clicked()
{
	interface.pulse_resume_mp();
}
