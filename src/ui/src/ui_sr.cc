/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>

#include <string.h>
#include <signal.h>
#include <dirent.h>

#include <boost/bind.hpp>
#include <boost/utility.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <iostream>
#include <fstream>

#include <pthread.h>
#include <errno.h>

#include "ui/ui.h"
#include "ui/ui_class.h"
#include "ui/src/ui_ecp.h"
#include "ui/src/ui_sr.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "lib/configurator.h"
#include "lib/mis_fun.h"

#include "lib/srlib.h"

#if !defined(USE_MESSIP_SRR)
/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include <Pt.h>
#include <Ph.h>

void ui_sr_buffer::operator()() {
	lib::set_thread_name("sr");

	name_attach_t *attach;

	if ((attach = name_attach(NULL, ui.sr_attach_point.c_str(),
			NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		perror(
				"BLAD SR ATTACH, przypuszczalnie nie uruchomiono gns, albo blad wczytywania konfiguracji");
		return;
	}
	ui.is_sr_thread_loaded = true;
	while (1) {
		lib::sr_package_t sr_msg;
		//	printf("przed MsgReceive: \n");
		int rcvid = MsgReceive_r(attach->chid, &sr_msg, sizeof(sr_msg), NULL);
		//	printf("za MsgReceive: \n");
		if (rcvid < 0) /* Error condition, exit */
		{
			if (rcvid == -EINTR) {
				//fprintf(stderr, "MsgReceive_r() interrupted by signal\n");
				continue;
			}

			fprintf(stderr, "SR: Receive failed (%s)\n", strerror(-rcvid));
			// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			break;
		}

		if (rcvid == 0) /* Pulse received */
		{
			// printf("sr puls\n");
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

		if (strlen(sr_msg.process_name) > 1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{
			// prrintf("srt: \n");
			flushall();

			ui.ui_sr_obj->put_one_msg(sr_msg);

		} else {
			printf("SR(%s:%d) unexpected message\n", __FILE__, __LINE__);
		}

	}
}

#endif /* USE_MESSIP_SRR */

ui_sr_buffer::ui_sr_buffer(Ui& _ui) :
	ui(_ui), cb(UI_SR_BUFFER_LENGHT) {

	thread_id = new boost::thread(boost::bind(&ui_sr_buffer::operator(), this));
}

ui_sr_buffer::~ui_sr_buffer() {
	//	printf("ui_sr_buffer\n");
	//	thread_id->interrupt();
	//	thread_id->join(); // join it
	//	delete thread_id;
}

void ui_sr_buffer::put_one_msg(const lib::sr_package_t& new_msg) {

	boost::mutex::scoped_lock lock(mtx);
	cb.push_back(new_msg);

	return;
}

void ui_sr_buffer::get_one_msg(lib::sr_package_t& new_msg) {
	boost::mutex::scoped_lock lock(mtx);
	new_msg = cb.front();
	cb.pop_front();

	return;
}

bool ui_sr_buffer::buffer_empty() // sprawdza czy bufor jest pusty
{
	boost::mutex::scoped_lock lock(mtx);
	return cb.empty();
}
