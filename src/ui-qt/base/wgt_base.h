#ifndef WGT_BASE_H
#define WGT_BASE_H

#include <QtGui/QWidget>
//#include <QVBoxLayout>
#include <QDockWidget>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_base : public QWidget
{
Q_OBJECT

public:
	wgt_base(QString _widget_label, mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_base();
	virtual void my_open();
	void my_close();

	QString widget_label;
	QDockWidget* dwgt;
	//QVBoxLayout* vl;

protected:
	mrrocpp::ui::common::Interface& interface;

};

#endif // WGT_PROCESS_CONTROL_H
