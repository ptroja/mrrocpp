#ifndef WGT_INPUT_INTEGER_H
#define WGT_INPUT_INTEGER_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_input_integer.h"
#include "../wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_input_integer : public wgt_base
{
Q_OBJECT

public:
	wgt_input_integer(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_input_integer();


	void hideEvent(QHideEvent * event);
	void my_open(bool set_on_top=false);
private:
	Ui::wgt_input_integerClass* ui;

private slots:

	void on_pushButton_ok_clicked();
	void on_pushButton_cancel_clicked();

};

#endif // WGT_INPUT_INTEGER_H
