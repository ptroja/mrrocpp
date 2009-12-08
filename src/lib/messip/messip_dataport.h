/*
 * messip_port.h
 *
 *  Created on: Dec 8, 2009
 *      Author: ptroja
 */

#ifndef MESSIP_PORT_H_
#define MESSIP_PORT_H_

#include <string>

#include "messip.h"

namespace messip {

template <class SendData, class ReceiveData>
int port_send( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   const SendData & send,
   int32_t * answer,
   ReceiveData & reply,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_send(ch, type, subtype, &send, sizeof(send), answer, &reply, sizeof(reply), msec_timeout);
}

template <class ReceiveData>
int port_receive( messip_channel_t * ch,
   int32_t & type,
   int32_t & subtype,
   ReceiveData & data,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_receive(ch, type, subtype, &data, sizeof(data), msec_timeout);
}

template <class ReplyData>
int port_reply( messip_channel_t * ch,
   int index,
   int32_t answer,
   const ReplyData & data,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_reply(ch, index, answer, &data, sizeof(data), msec_timeout);
}

messip_channel_t *
port_create(messip_cnx_t * cnx,
   const std::string & name,
   int32_t msec_timeout = MESSIP_NOTIMEOUT,
   int32_t maxnb_msg_buffered = 0)
{
	return messip_channel_create(cnx, name.c_str(), msec_timeout, maxnb_msg_buffered);
}

int
port_delete(messip_channel_t * ch,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_channel_delete(ch, msec_timeout);
}

messip_channel_t *
port_connect( messip_cnx_t * cnx,
   const std::string & name,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_channel_connect(cnx, name.c_str(), msec_timeout);
}

int
port_disconnect( messip_channel_t * ch,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_channel_disconnect(ch, msec_timeout);
}

} /* namespace messip */

#endif /* MESSIP_PORT_H_ */
