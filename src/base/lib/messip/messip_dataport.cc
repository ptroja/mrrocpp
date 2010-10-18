/*
 * messip_dataport.cc
 *
 *  Created on: Dec 9, 2009
 *      Author: ptroja
 */

#include "base/lib/messip/messip_dataport.h"

namespace messip {

int port_send_pulse( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   int32_t msec_timeout)
{
	return messip_send(ch, type, subtype, NULL, 0, NULL, NULL, -1, msec_timeout);
}

int port_receive_pulse( messip_channel_t * ch,
   int32_t & type,
   int32_t & subtype,
   int32_t msec_timeout)
{
	return messip_receive(ch, &type, &subtype, NULL, 0, msec_timeout);
}

int port_reply_ack( messip_channel_t * ch,
   int index,
   int32_t msec_timeout)
{
	return messip_reply(ch, index, 0, NULL, 0, msec_timeout);
}

int port_reply_nack( messip_channel_t * ch,
   int index,
   int32_t msec_timeout)
{
	return messip_reply(ch, index, -1, NULL, 0, msec_timeout);
}

messip_channel_t *
port_create(const std::string & name,
   int32_t msec_timeout,
   int32_t maxnb_msg_buffered)
{
	return messip_channel_create(NULL, name.c_str(), msec_timeout, maxnb_msg_buffered);
}

int
port_delete(messip_channel_t * ch,
   int32_t msec_timeout)
{
	return messip_channel_delete(ch, msec_timeout);
}

messip_channel_t *
port_connect(const std::string & name,
   int32_t msec_timeout)
{
	return messip_channel_connect(NULL, name.c_str(), msec_timeout);
}

int
port_disconnect( messip_channel_t * ch,
   int32_t msec_timeout)
{
	return messip_channel_disconnect(ch, msec_timeout);
}

} /* namespace messip */
