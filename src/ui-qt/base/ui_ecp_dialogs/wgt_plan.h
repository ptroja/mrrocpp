#ifndef WGT_PLAN_H
#define WGT_PLAN_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_plan.h"
#include "../wgt_base.h"
#include "../../../base/lib/com_buf.h"

#include "application/swarmitfix/plan.hxx"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_plan : public wgt_base
{
Q_OBJECT

public:
	wgt_plan(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_plan();

	void hideEvent(QHideEvent * event);
	void my_open(bool set_on_top = false);

private:
	Ui::wgt_planClass* ui;

	//! Deactivate widget into idle mode
	void reply();

	//! Reload inputs with original request
	void reload();

private slots:

	void on_pushButton_prev_clicked();
	void on_pushButton_next_clicked();
	void on_pushButton_exec_clicked();
	void on_pushButton_save_clicked();
	void on_pushButton_reload_clicked();

};

#endif // WGT_PLAN_H
