/*
 * log_message.cc
 *
 *  Created on: 02-11-2011
 *      Author: mboryn
 */

#include <cstring>
#include <cstdio>

#include "log_message.h"

namespace logger {

config_message::config_message(){
	memset(header, 0, log_message_text_buf_size);
	memset(filename_prefix, 0, log_message_text_buf_size);
}

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

void log_message::append_Homog_matrix(const mrrocpp::lib::Homog_matrix& hm)
{
	char hm_text[300];
	strcpy(hm_text, "");

	for(int i=0; i<3; ++i){
		for(int j=0; j<4; ++j){
			char v[16];
			sprintf(v, "%0.6lf;", hm(i, j));
			strcat(hm_text, v);
		}
	}
	strcat(text, hm_text);
}

} // namespace logger
