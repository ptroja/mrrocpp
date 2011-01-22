#include "wgt_spkm_inc.h"
#include "../interface.h"

wgt_spkm_inc::wgt_spkm_inc(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QWidget(parent), interface(_interface)
{
	ui.setupUi(this);

	dwgt = new QDockWidget(interface.mw);
	//dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	dwgt->setWindowTitle("Spkm Incremental Move");

	vl = new QVBoxLayout();
	dwgt->setLayout(vl);

	vl->addWidget(this);
	dwgt->setWidget(this);
	dwgt->hide();
	interface.mw->addDockWidget(Qt::LeftDockWidgetArea, dwgt);
}

wgt_spkm_inc::~wgt_spkm_inc()
{

}
