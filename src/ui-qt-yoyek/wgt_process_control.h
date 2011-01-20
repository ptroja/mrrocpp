#ifndef WGT_PROCESS_CONTROL_H
#define WGT_PROCESS_CONTROL_H

#include <QtGui/QWidget>
#include "ui_wgt_process_control.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_process_control : public QWidget
{
Q_OBJECT

public:
	wgt_process_control(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_process_control();

private:
	Ui::wgt_process_controlClass ui;
	mrrocpp::ui::common::Interface& interface;

private slots:
	void on_mp_start_pushButton_clicked();
	void on_mp_stop_pushButton_clicked();
	void on_mp_pause_pushButton_clicked();
	void on_mp_resume_pushButton_clicked();

};

#endif // WGT_PROCESS_CONTROL_H
