/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <strings.h>

#include <cstring>
#include <csignal>
#include <dirent.h>

#include <boost/bind.hpp>
#include <boost/utility.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <fcntl.h>
#include <cstring>
#include <sys/types.h>
#include <sys/utsname.h>
#include <iostream>
#include <fstream>

#include <cerrno>

#include "interface.h"
//#include "ui/src/ui.h"
//#include "ui/src/ui_class.h"
//#include "ui/src/ui_ecp.h"
#include "ui_sr.h"

#include "base/lib/mis_fun.h"
#include "base/lib/sr/srlib.h"

#include "base/lib/configurator.h"

#include "base/lib/sr/srlib.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace ui {
namespace common {

void sr_buffer::operator()()
{

	interface.mask_signals_for_thread();
	lib::set_thread_name("sr");

	lib::fd_server_t ch;

	ch = messip::port_create(interface.sr_attach_point);
	assert(ch);

	thread_started.command();

	while (true) {
		lib::sr_package_t sr_msg;

		int32_t type, subtype;
		int rcvid = messip::port_receive(ch, type, subtype, sr_msg);
		//	printf("SR received: %d\n", licznik);

		if (rcvid != MESSIP_MSG_NOREPLY) {
			//fprintf(stderr, "sr_buffer::rcvid = %d\n", rcvid);
			continue;
		}

		if (strlen(sr_msg.process_name) > 1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
				{
			//	flushall();

			put_one_msg(sr_msg);
		} else {
			printf("SR(%s:%d) unexpected message\n", __FILE__, __LINE__);
		}
	}
}

sr_buffer::sr_buffer(Interface& _interface) :
		interface(_interface), cb(UI_SR_BUFFER_LENGHT), thread_started()
{
	thread_id = boost::thread(boost::bind(&sr_buffer::operator(), this));
}

sr_buffer::~sr_buffer()
{
	//	printf("sr_buffer\n");
	//	thread_id.interrupt();
	//	thread_id.join();
}

void sr_buffer::put_one_msg(const lib::sr_package_t& new_msg)
{

	boost::mutex::scoped_lock lock(mtx);
	cb.push_back(new_msg);

	return;
}

void sr_buffer::get_one_msg(lib::sr_package_t& new_msg)
{
	boost::mutex::scoped_lock lock(mtx);
	new_msg = cb.front();
	cb.pop_front();

	return;
}

void sr_buffer::inter_get_one_msg(lib::sr_package_t& new_msg)
{
	new_msg = cb_inter.front();
	cb_inter.pop_front();

	return;
}

void sr_buffer::copy_buffers()
{
	boost::mutex::scoped_lock lock(mtx);
	cb_inter = cb;

}

void sr_buffer::clear_buffer()
{
	boost::mutex::scoped_lock lock(mtx);
	cb.clear();

}

bool sr_buffer::buffer_empty() // sprawdza czy bufor jest pusty
{
	boost::mutex::scoped_lock lock(mtx);
	return cb.empty();
}

bool sr_buffer::inter_buffer_empty() // sprawdza czy bufor inter jest pusty
{
	return cb_inter.empty();
}

}
} //namespace ui
} //namespace mrrocpp
