#include "wgt_base.h"
#include "interface.h"

wgt_base::wgt_base(QString _string, mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QWidget(parent), interface(_interface)
{
	dwgt = new QDockWidget(interface.mw);
	//dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	dwgt->setWindowTitle(_string);

	//vl = new QVBoxLayout();
	//dwgt->setLayout(vl);

	//	vl->addWidget(this);
	dwgt->setWidget(this);
	dwgt->hide();
	interface.mw->addDockWidget(Qt::LeftDockWidgetArea, dwgt);
}

wgt_base::~wgt_base()
{

}
