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

	typedef void (wgt_base::*my_open_ptr)();

	QString widget_label;
	QDockWidget* dwgt;
	//QVBoxLayout* vl;
	virtual void synchro_depended_init(){}
	virtual void init_and_copy(){}

protected:
	mrrocpp::ui::common::Interface& interface;

};

#endif // WGT_PROCESS_CONTROL_H
