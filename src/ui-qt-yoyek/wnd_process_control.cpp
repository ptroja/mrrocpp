#include "wnd_process_control.h"
#include "interface.h"

wnd_process_control::wnd_process_control(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QMainWindow(parent), interface(_interface)
{
	ui.setupUi(this);
}

wnd_process_control::~wnd_process_control()
{

}

void wnd_process_control::on_mp_start_pushButton_clicked()
{

}

void wnd_process_control::on_mp_stop_pushButton_clicked()
{

}

void wnd_process_control::on_mp_pause_pushButton_clicked()
{

}

void wnd_process_control::on_mp_resume_pushButton_clicked()
{

}
