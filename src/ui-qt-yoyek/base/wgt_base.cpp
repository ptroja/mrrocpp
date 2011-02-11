#include "wgt_base.h"
#include "interface.h"

wgt_base::wgt_base(QString _string, mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QWidget(parent), interface(_interface)
{
	dwgt = new QDockWidget(interface.get_main_window());
	//dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	dwgt->setWindowTitle(_string);

	//vl = new QVBoxLayout();
	//dwgt->setLayout(vl);

	//	vl->addWidget(this);
	dwgt->setWidget(this);
	dwgt->hide();
	dwgt->setFloating(true);
	interface.get_main_window()->addDockWidget(Qt::LeftDockWidgetArea, dwgt);
}

void wgt_base::my_open()
{
	dwgt->show();
	dwgt->raise();
}

wgt_base::~wgt_base()
{

}
