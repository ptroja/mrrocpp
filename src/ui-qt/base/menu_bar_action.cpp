#include "menu_bar_action.h"
#include "signal_dispatcher.h"

namespace Ui
{


MenuBarAction::MenuBarAction(QString text, mrrocpp::ui::common::UiRobot* robo, QWidget *parent) : QAction(parent), robot_action(true)
{
	robot = robo;
	setText(text);
	connect(this, SIGNAL(triggered()), this, SLOT(reemitTriggered()));
}

MenuBarAction::MenuBarAction(QString text, wgt_base* _wgt, SignalDispatcher *signalDispatcher, QWidget *parent) : QAction(parent), robot_action(false)
{
	wgt = _wgt;
	setText(text);
	connect(this, SIGNAL(triggered()), this, SLOT(reemitTriggered()));
	connect(this, SIGNAL(triggered(wgt_base*)), signalDispatcher, SLOT(open_new_window(wgt_base*)), Qt::AutoCompatConnection);
}

void MenuBarAction::reemitTriggered()
{
	robot_action ? emit triggered(robot) : emit triggered(wgt);
}

}
