#ifndef WGT_INPUT_DOUBLE_H
#define WGT_INPUT_DOUBLE_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_input_double.h"
#include "../wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_input_double : public wgt_base
{
Q_OBJECT

public:
	wgt_input_double(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_input_double();

	void hideEvent(QHideEvent * event);
	void my_open(bool set_on_top=false);
private:
	Ui::wgt_input_doubleClass* ui;

private slots:

	void on_pushButton_ok_clicked();
	void on_pushButton_cancel_clicked();

};

#endif // WGT_INPUT_DOUBLE_H
