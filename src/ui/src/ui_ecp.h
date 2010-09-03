// -------------------------------------------------------------------------
//                            ui.h
// Definicje struktur danych i metod dla procesu UI
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __UI_ECP_H
#define __UI_ECP_H

#include <boost/thread/thread.hpp>
#include <stdexcept>
#include <iostream>
#include <string>
#include <list>

#include "base/lib/com_buf.h"
#include "base/lib/srlib.h"
#include "base/lib/condition_synchroniser.h"

#include "ui/src/ui.h"

/**************************** ui_ecp_buffer *****************************/

class ui_ecp_buffer  : public boost::noncopyable{
private:

	Ui& ui;
	boost::thread *thread_id;

public:
	UI_ECP_COMMUNICATION_STATE communication_state;
	lib::ECP_message ecp_to_ui_msg;
	lib::UI_reply ui_rep;

	//! main thread loop
	void operator()();

	lib::condition_synchroniser synchroniser;
	ui_ecp_buffer(Ui& _ui);
	~ui_ecp_buffer();

};

#endif

