/*
 * messip_port.cc
 *
 *  Created on: Dec 8, 2009
 *      Author: ptroja
 */

#include "messip.h"

template <class SendData, class ReceiveData>
int messip_send( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   const SendData & send,
   int32_t * answer,
   ReceiveData & reply,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_send(ch, type, subtype, &send, sizeof(send), &reply, sizeof(reply), msec_timeout);
}
