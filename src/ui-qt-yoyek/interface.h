#ifndef __INTERFACE_H
#define __INTERFACE_H

#include <QMainWindow>
#include "mainwindow.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/lib/sr/sr_ui.h"
#include "base/lib/configurator.h"

#include "ui.h"

namespace mrrocpp {
namespace ui {
namespace common {

class sr_buffer;

class Interface
{
private:

public:
	busy_flag communication_flag;
	sr_buffer* ui_sr_obj;

	Interface();
	void init();
	~Interface();
	int set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion);

	UI_NOTIFICATION_STATE_ENUM notification_state;

	std::string sr_attach_point;
	bool is_sr_thread_loaded;


	void create_threads();


	MainWindow* mw;

};

}
}
}

#endif

