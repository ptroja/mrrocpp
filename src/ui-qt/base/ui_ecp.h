// -------------------------------------------------------------------------
// Definicje struktur danych i metod dla procesu UI
// -------------------------------------------------------------------------

#ifndef __UI_ECP_H
#define __UI_ECP_H

#include <boost/thread/thread.hpp>
#include <stdexcept>
#include <iostream>
#include <string>
#include <list>

#include "base/lib/com_buf.h"
#include "base/ecp_mp/ecp_ui_msg.h"
#include "base/lib/condition_synchroniser.h"

#include "ui.h"

namespace mrrocpp {
namespace ui {
namespace common {

/**************************** ecp_buffer *****************************/

class ecp_buffer : public boost::noncopyable
{
private:
	Interface& interface;
	boost::thread thread_id;

	lib::ECP_message _ecp_to_ui_msg;

public:
	UI_ECP_COMMUNICATION_STATE communication_state;
	const lib::ECP_message & ecp_to_ui_msg;
	lib::UI_reply ui_rep;

	//! main thread loop
	void operator()();

	lib::condition_synchroniser synchroniser;

	ecp_buffer(Interface& _interface);

	~ecp_buffer();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

