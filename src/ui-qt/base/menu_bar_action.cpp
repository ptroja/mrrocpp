#include "menu_bar_action.h"
//#include "../base/ui_robot.h"
//#include "../irp6ot_m/ui_r_irp6ot_m.h"

namespace Ui
{

//MenuBarAction::MenuBarAction(mrrocpp::ui::common::UiRobot* robo, QWidget *parent) : QAction(parent)
//{
//	robot=robo;
//	connect(this, SIGNAL(triggered()), this, SLOT(reemitTriggered()));
//}

MenuBarAction::MenuBarAction(QString text, mrrocpp::ui::common::UiRobot* robo, QWidget *parent) : QAction(parent)
{
	robot=robo;
	setText(text);
	connect(this, SIGNAL(triggered()), this, SLOT(reemitTriggered()));
	//printf("connectted in MenuBarAction\n");
}


void MenuBarAction::reemitTriggered()
{
	emit triggered(robot); //!@#$%^&*(*&^%$#@#$%^&*!!!!!!
}

}
