#ifndef WGT_YES_NO_H
#define WGT_YES_NO_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_yes_no.h"
#include "../wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_yes_no : public wgt_base
{
Q_OBJECT

public:
	wgt_yes_no(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_yes_no();

	Ui::wgt_yes_noClass * get_ui();

	void hideEvent(QHideEvent * event);

private:
	Ui::wgt_yes_noClass* ui;

private slots:

	void on_pushButton_yes_clicked();
	void on_pushButton_no_clicked();

};

#endif // WGT_YES_NO_H
