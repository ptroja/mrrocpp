#include "wgt_yes_no.h"
#include "../interface.h"

wgt_yes_no::wgt_yes_no(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Yes No Dialog", _interface, parent), ui(new Ui::wgt_yes_noClass)
{
	ui->setupUi(this);

}

wgt_yes_no::~wgt_yes_no()
{

}

void wgt_yes_no::on_pushButton_yes_clicked()
{

}

void wgt_yes_no::on_pushButton_no_clicked()
{

}

Ui::wgt_yes_noClass * wgt_yes_no::get_ui()
{
	return ui;
}
