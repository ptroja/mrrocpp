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

	void process_control_window_init();

	void my_open();

	Ui::wgt_process_controlClass * get_ui();

private:
	Ui::wgt_process_controlClass* ui;

	// aktualizacja ustawien przyciskow
	int init();
	int block_all_ecp_trigger_widgets();
	int unblock_all_ecp_trigger_widgets();

signals:
	void process_control_window_init_signal();

public slots:
	void process_control_window_init_slot();

private slots:
	// MP
	void on_mp_start_pushButton_clicked();
	void on_mp_stop_pushButton_clicked();
	void on_mp_pause_pushButton_clicked();
	void on_mp_resume_pushButton_clicked();
	void on_mp_trigger_pushButton_clicked();

	// ECP
	void on_all_ecp_trigger_pushButton_clicked();

	// Reader
	void on_all_reader_start_pushButton_clicked();
	void on_all_reader_stop_pushButton_clicked();
	void on_all_reader_trigger_pushButton_clicked();

};

#endif // WGT_PROCESS_CONTROL_H
