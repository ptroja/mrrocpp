#ifndef MENUBARACTION_H_
#define MENUBARACTION_H_

#include <QAction>
#include "../base/ui_robot.h"


namespace Ui
{



class MenuBarAction : public QAction
{
Q_OBJECT

public:
//	MenuBarAction(mrrocpp::ui::common::UiRobot* robo, QWidget *parent);
	MenuBarAction(QString text, mrrocpp::ui::common::UiRobot* robo, QWidget *parent);


signals:
	void triggered(mrrocpp::ui::common::UiRobot* robot);
	//void triggered(mrrocpp::ui::irp6ot_m::UiRobot* robot);

private slots:
	void reemitTriggered();

private:
	mrrocpp::ui::common::UiRobot* robot;
//	mrrocpp::ui::irp6ot_m::UiRobot* irp;

};
}
#endif
