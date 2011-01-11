#ifndef WND_SPKM_INC_H
#define WND_SPKM_INC_H

#include <QtGui/QMainWindow>
#include "ui_wnd_spkm_inc.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wnd_spkm_inc : public QMainWindow
{
Q_OBJECT

public:
	wnd_spkm_inc(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wnd_spkm_inc();

private:
	Ui::wnd_spkm_incClass ui;
	mrrocpp::ui::common::Interface& interface;
};

#endif // WND_SPKM_INC_H
