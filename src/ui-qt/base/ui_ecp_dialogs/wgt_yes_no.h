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

private:
	Ui::wgt_yes_noClass ui;

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

#endif // WGT_YES_NO_H
