#ifndef WGT_PROCESS_CONTROL_H
#define WGT_PROCESS_CONTROL_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_process_control.h"
#include "wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_process_control : public wgt_base
{
Q_OBJECT

public:
	wgt_process_control(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_process_control();

private:
	Ui::wgt_process_controlClass ui;

private slots:
	void on_mp_start_pushButton_clicked();
	void on_mp_stop_pushButton_clicked();
	void on_mp_pause_pushButton_clicked();
	void on_mp_resume_pushButton_clicked();
	void on_mp_trigger_pushButton_clicked();

};

#endif // WGT_PROCESS_CONTROL_H
