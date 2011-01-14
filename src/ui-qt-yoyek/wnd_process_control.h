#ifndef WND_PROCESS_CONTROL_H
#define WND_PROCESS_CONTROL_H

#include <QtGui/QMainWindow>
#include "ui_wnd_process_control.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wnd_process_control : public QMainWindow
{
Q_OBJECT

public:
	wnd_process_control(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wnd_process_control();

private:
	Ui::wnd_process_controlClass ui;
	mrrocpp::ui::common::Interface& interface;

private slots:
	void on_mp_start_pushButton_clicked();
	void on_mp_stop_pushButton_clicked();
	void on_mp_pause_pushButton_clicked();
	void on_mp_resume_pushButton_clicked();

};

#endif // WND_PROCESS_CONTROL_H
