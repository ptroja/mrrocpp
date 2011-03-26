#include "wgt_base.h"
#include "interface.h"

wgt_base::wgt_base(QString _widget_label, mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QWidget(parent), widget_label(_widget_label), interface(_interface)
{
	dwgt = new QDockWidget(interface.get_main_window());
	//dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	dwgt->setWindowTitle(widget_label);

	//vl = new QVBoxLayout();
	//dwgt->setLayout(vl);

	//	vl->addWidget(this);
	dwgt->setWidget(this);
	dwgt->hide();
	//dwgt->setFloating(true);
	interface.get_main_window()->addDockWidget(Qt::LeftDockWidgetArea, dwgt);
}

void wgt_base::my_open()
{
	MainWindow *mw = interface.get_main_window();
	if (!(dwgt->isVisible())) {
		if ((interface.wgt_pc->dwgt) != dwgt) {
			mw->tabifyDockWidget(interface.wgt_pc->dwgt, dwgt);
		}
	}
	dwgt->show();
	dwgt->raise();
}

void wgt_base::my_close()
{
	dwgt->hide();

}

wgt_base::~wgt_base()
{

}
