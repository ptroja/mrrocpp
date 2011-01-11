#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <strings.h>

#include <QtGui/QApplication>
#include "mainwindow.h"

#include "interface.h"
#include "ui_sr.h"

namespace mrrocpp {
namespace ui {
namespace common {

Interface::Interface()
{
	mw = new MainWindow();
	//	printf("sr_buffer\n");
	//	thread_id.interrupt();
	//	thread_id.join();
}

void Interface::init()
{

	is_sr_thread_loaded = false;

	create_threads();

	mw->show();

}

Interface::~Interface()
{
	//	printf("sr_buffer\n");
	//	thread_id.interrupt();
	//	thread_id.join();
}

int Interface::set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion)
{
	if (new_notifacion != notification_state) {

		notification_state = new_notifacion;

		switch (new_notifacion)
		{
			case UI_N_STARTING:
				/*
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "STARTING", 0);
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_MAGENTA, 0);
				 */
				break;
			case UI_N_READY:
				/*
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "READY", 0);
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_BLUE, 0);
				 */
				break;
			case UI_N_BUSY:
				/*
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "BUSY", 0);
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
				 */
				break;
			case UI_N_EXITING:
				/*
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "EXITING", 0);
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_MAGENTA, 0);
				 */
				break;
			case UI_N_COMMUNICATION:
				/*
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "COMMUNICATION", 0);
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
				 */
				break;
			case UI_N_SYNCHRONISATION:
				/*
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "SYNCHRONISATION", 0);
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
				 */
				break;
			case UI_N_PROCESS_CREATION:
				/*
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "PROCESS CREATION", 0);
				 PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
				 */
				break;
		}

		return 1;

	}

	return 0;

}

void Interface::create_threads()
{
	/* Temporary
	 meb_tid = new feb_thread(*main_eb);

	 ui_ecp_obj = new ecp_buffer(*this);
	 */
	delay(1);
	ui_sr_obj = new sr_buffer(*this);

#if defined(__QNXNTO__)

#endif

}

}
}
}
