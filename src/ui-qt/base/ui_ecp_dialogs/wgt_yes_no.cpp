#include "wgt_yes_no.h"
#include "../interface.h"

wgt_yes_no::wgt_yes_no(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Process control", _interface, parent)
{
	ui.setupUi(this);

}

wgt_yes_no::~wgt_yes_no()
{

}

void wgt_yes_no::on_mp_start_pushButton_clicked()
{
	interface.pulse_start_mp();
}

void wgt_yes_no::on_mp_stop_pushButton_clicked()
{
	interface.pulse_stop_mp();
}

void wgt_yes_no::on_mp_pause_pushButton_clicked()
{
	interface.pulse_pause_mp();
}

void wgt_yes_no::on_mp_resume_pushButton_clicked()
{
	interface.pulse_resume_mp();
}

void wgt_yes_no::on_mp_trigger_pushButton_clicked()
{
	interface.pulse_trigger_mp();
}

//ECP
void wgt_yes_no::on_all_ecp_trigger_pushButton_clicked()
{
	interface.pulse_trigger_ecp();
}

// Reader
void wgt_yes_no::on_all_reader_start_pushButton_clicked()
{
	interface.pulse_start_all_reader();
}

void wgt_yes_no::on_all_reader_stop_pushButton_clicked()
{
	interface.pulse_stop_all_reader();
}

void wgt_yes_no::on_all_reader_trigger_pushButton_clicked()
{
	interface.pulse_trigger_all_reader();
}

