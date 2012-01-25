#ifndef WGT_MESSAGE_H
#define WGT_MESSAGE_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_message.h"
#include "../wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class wgt_message : public wgt_base
{
Q_OBJECT

public:
	wgt_message(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_message();

	void my_open(bool set_on_top = false);
private:
	Ui::wgt_messageClass* ui;

};

#endif // WGT_MESSAGE_H
