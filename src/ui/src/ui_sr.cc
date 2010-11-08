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

#include <pthread.h>
#include <cerrno>

#include "ui/src/ui.h"
#include "ui/src/ui_class.h"
#include "ui/src/ui_ecp.h"
#include "ui/src/ui_sr.h"

#include "base/lib/mis_fun.h"
#include "base/lib/sr/srlib.h"

#include "base/lib/configurator.h"

#include "base/lib/sr/srlib.h"

#if !defined(__QNXNTO__)
/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include <Pt.h>
#include <Ph.h>
#endif

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#endif

namespace mrrocpp {
namespace ui {
namespace common {

void sr_buffer::operator()()
{
	lib::set_thread_name("sr");

#if !defined(USE_MESSIP_SRR)
	name_attach_t *attach;

	if ((attach = name_attach(NULL, interface.sr_attach_point.c_str(),
			NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		perror(
				"BLAD SR ATTACH, przypuszczalnie nie uruchomiono gns, albo blad wczytywania konfiguracji");
		return;
	}
#else
	messip_channel_t *ch = messip::port_create(interface.sr_attach_point);
	assert(ch);
#endif /* USE_MESSIP_SRR */

	interface.is_sr_thread_loaded = true;
	while (1) {
		lib::sr_package_t sr_msg;

#if !defined(USE_MESSIP_SRR)
		int rcvid = MsgReceive_r(attach->chid, &sr_msg, sizeof(sr_msg), NULL);

		if (rcvid < 0) /* Error condition, exit */
		{
			if (rcvid == -EINTR) {
				//fprintf(stderr, "MsgReceive_r() interrupted by signal\n");
				continue;
			}

			fprintf(stderr, "SR: Receive failed (%s)\n", strerror(-rcvid));
			// TODO: throw
			break;
		}

		if (rcvid == 0) /* Pulse received */
		{
			switch (sr_msg.hdr.code) {
			case _PULSE_CODE_DISCONNECT:
				ConnectDetach(sr_msg.hdr.scoid);
				break;
			case _PULSE_CODE_UNBLOCK:
				break;
			default:
				break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (sr_msg.hdr.type >= _IO_BASE && sr_msg.hdr.type <= _IO_MAX) {
			//  	  printf("w SR _IO_BASE _IO_MAX %d\n",_IO_CONNECT );
			//  MsgError(rcvid, ENOSYS);
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}

		int16_t status;
		MsgReply(rcvid, EOK, &status, sizeof(status));
#else
		int32_t type, subtype;
		int rcvid = messip::port_receive(ch, type, subtype, sr_msg);

		if(rcvid != MESSIP_MSG_NOREPLY)
			continue;
#endif

		if (strlen(sr_msg.process_name) > 1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{
			flushall();

			put_one_msg(sr_msg);
		} else {
			printf("SR(%s:%d) unexpected message\n", __FILE__, __LINE__);
		}
	}
}

sr_buffer::sr_buffer(Interface& _interface) :
	interface(_interface), cb(UI_SR_BUFFER_LENGHT) {
	thread_id = boost::thread(boost::bind(&sr_buffer::operator(), this));
}

sr_buffer::~sr_buffer() {
	//	printf("sr_buffer\n");
	//	thread_id.interrupt();
	//	thread_id.join();
}

void sr_buffer::put_one_msg(const lib::sr_package_t& new_msg) {

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

bool sr_buffer::buffer_empty() // sprawdza czy bufor jest pusty
{
	boost::mutex::scoped_lock lock(mtx);
	return cb.empty();
}

}
} //namespace ui
} //namespace mrrocpp
