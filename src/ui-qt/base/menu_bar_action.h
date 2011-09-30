#ifndef MENUBARACTION_H_
#define MENUBARACTION_H_

#include <QAction>
#include "ui_robot.h"
#include "wgt_base.h"


namespace Ui
{
class SignalDispatcher;


class MenuBarAction : public QAction
{
Q_OBJECT

public:
	MenuBarAction(QString text, mrrocpp::ui::common::UiRobot* robo, QWidget *parent);
	MenuBarAction(QString text, wgt_base* _wgt, SignalDispatcher *signalDispatcher, QWidget *parent);

signals:
	void triggered(mrrocpp::ui::common::UiRobot* robot);
	void triggered(wgt_base* wgt);

private slots:
	void reemitTriggered();

private:
	union
	{
		mrrocpp::ui::common::UiRobot* robot;
		wgt_base* wgt;
	};

	bool robot_action;

};
}
#endif
