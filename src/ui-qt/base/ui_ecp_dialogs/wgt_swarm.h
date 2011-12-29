#ifndef WGT_SWARM_H
#define WGT_SWARM_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_swarm.h"
#include "../wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_swarm : public wgt_base
{
Q_OBJECT

public:
	wgt_swarm(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_swarm();

	Ui::wgt_swarmClass * get_ui();

	void hideEvent(QHideEvent * event);
	void my_open(bool set_on_top = false);
private:
	Ui::wgt_swarmClass* ui;

private slots:

	void on_pushButton_yes_clicked();
	void on_pushButton_no_clicked();

};

#endif // WGT_SWARM_H
