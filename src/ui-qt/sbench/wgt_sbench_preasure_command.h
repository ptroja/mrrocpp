#ifndef WGT_SBENCH_PREASURE_COMMAND_H
#define WGT_SBENCH_PREASURE_COMMAND_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "wgt_sbench_command.h"

namespace mrrocpp {
namespace ui {
namespace sbench {
const std::string WGT_SBENCH_PREASURE_COMMAND = "WGT_SBENCH_PREASURE_COMMAND";
}
}
}

class wgt_sbench_preasure_command : public wgt_sbench_command
{
Q_OBJECT

public:
	wgt_sbench_preasure_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_sbench_preasure_command();

	void init();
	void execute();
	void read_and_copy();

};

#endif // WGT_SBENCH_COMMAND_H
