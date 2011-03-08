#include "wgt_yes_no.h"
#include "../interface.h"

wgt_yes_no::wgt_yes_no(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Yes No Dialog", _interface, parent)
{
	ui.setupUi(this);

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
