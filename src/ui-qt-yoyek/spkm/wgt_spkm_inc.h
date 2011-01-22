#ifndef WGT_SPKM_INC_H
#define WGT_SPKM_INC_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_spkm_inc.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_spkm_inc : public QWidget
{
Q_OBJECT

public:
	wgt_spkm_inc(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_spkm_inc();

	QDockWidget* dwgt;
	QVBoxLayout* vl;

private:
	Ui::wgt_spkm_incClass ui;
	mrrocpp::ui::common::Interface& interface;
};

#endif // WGT_SPKM_INC_H
