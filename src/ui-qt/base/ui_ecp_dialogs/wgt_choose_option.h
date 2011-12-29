#ifndef WGT_CHOOSE_OPTION_H
#define WGT_CHOOSE_OPTION_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_choose_option.h"
#include "../wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_choose_option : public wgt_base
{
Q_OBJECT

public:
	wgt_choose_option(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_choose_option();

	void hideEvent(QHideEvent * event);

	void my_open(bool set_on_top=false);

private:
	Ui::wgt_choose_optionClass* ui;

private slots:

	void on_pushButton_1_clicked();
	void on_pushButton_2_clicked();
	void on_pushButton_3_clicked();
	void on_pushButton_4_clicked();
	void on_pushButton_cancel_clicked();

};

#endif // WGT_CHOOSE_OPTION_H
