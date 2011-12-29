#ifndef WGT_TEACHING_H
#define WGT_TEACHING_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_teaching.h"
#include "../wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_teaching : public wgt_base
{
Q_OBJECT

public:
	wgt_teaching(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_teaching();


	void hideEvent(QHideEvent * event);
	void my_open(QString label, bool set_on_top = false);
private:
	Ui::wgt_teachingClass* ui;

private slots:

	void on_pushButton_send_move_clicked();
	void on_pushButton_end_motion_clicked();

};

#endif // WGT_TEACHING_H
