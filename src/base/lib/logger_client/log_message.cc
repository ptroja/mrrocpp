/*
 * log_message.cc
 *
 *  Created on: 02-11-2011
 *      Author: mboryn
 */

#include <cstring>

#include "log_message.h"

namespace logger {

log_message::log_message():number(0), seconds(0), nanoseconds(0), time_elems(0){
	memset(text, 0, log_message_text_buf_size);
	for(int i=0; i<log_message_time_buf_size; ++i){
		time_buf[i].tv_nsec = 0;
		time_buf[i].tv_sec = 0;
	}
}

void log_message::prepare_text(){
	// does nothing
}

}
