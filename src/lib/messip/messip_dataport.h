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
   ReceiveData & reply,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	int32_t answer;
	return messip_send(ch, type, subtype, &send, sizeof(send), &answer, &reply, sizeof(reply), msec_timeout);
}

template <class SendData>
int port_send_sync( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   const SendData & send,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	int32_t answer;
	return messip_send(ch, type, subtype, &send, sizeof(send), &answer, NULL, 0, msec_timeout);
}

template <class SendData>
int port_send_async( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   const SendData & send,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_send(ch, type, subtype, &send, sizeof(send), NULL, NULL, -1, msec_timeout);
}

int port_send_pulse( messip_channel_t * ch,
   int32_t type,
   int32_t subtype,
   int32_t msec_timeout = MESSIP_NOTIMEOUT);

template <class ReceiveData>
int port_receive( messip_channel_t * ch,
   int32_t & type,
   int32_t & subtype,
   ReceiveData & data,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_receive(ch, &type, &subtype, &data, sizeof(data), msec_timeout);
}

int port_receive_pulse( messip_channel_t * ch,
   int32_t & type,
   int32_t & subtype,
   int32_t msec_timeout = MESSIP_NOTIMEOUT);

template <class ReplyData>
int port_reply( messip_channel_t * ch,
   int index,
   int32_t status,
   const ReplyData & data,
   int32_t msec_timeout = MESSIP_NOTIMEOUT)
{
	return messip_reply(ch, index, status, &data, sizeof(data), msec_timeout);
}

int port_reply_ack( messip_channel_t * ch,
   int index,
   int32_t msec_timeout = MESSIP_NOTIMEOUT);

int port_reply_nack( messip_channel_t * ch,
   int index,
   int32_t msec_timeout = MESSIP_NOTIMEOUT);

messip_channel_t *
port_create(const std::string & name,
   int32_t msec_timeout = MESSIP_NOTIMEOUT,
   int32_t maxnb_msg_buffered = 0);

int
port_delete(messip_channel_t * ch,
   int32_t msec_timeout = MESSIP_NOTIMEOUT);

messip_channel_t *
port_connect(const std::string & name,
   int32_t msec_timeout = MESSIP_NOTIMEOUT);

int
port_disconnect( messip_channel_t * ch,
   int32_t msec_timeout = MESSIP_NOTIMEOUT);

//! Object oriented interface to server-side message passing
class ServerPort {
	private:
		//! underlaying communication channel
		messip_channel_t * ch;

	public:
		//! Construcor
		//! @param name name of the communication channel
		ServerPort(const std::string & name) {
			if ((ch = port_create(name)) == NULL) {
				// TODO: throw
			}
		}

		//! Destructor
		~ServerPort() {
			if (port_delete(ch) == -1) {
				// TODO: throw
			}
		}

		//! Receive data from the port
		//! @param type received message type
		//! @param subtype received message subtype
		//! @param data data to receive
		//! @param msec_timeout operation timeout
		template <class ReceiveData>
		int Receive(int32_t & type,
		   int32_t & subtype,
		   ReceiveData & data,
		   int32_t msec_timeout = MESSIP_NOTIMEOUT);

		//! Receive pulse message
		//! @param type received pulse type
		//! @param subtype received pulse subtype
		//! @param msec_timeout operation timeout
		int ReceivePulse(int32_t & type,
		   int32_t & subtype,
		   int32_t msec_timeout = MESSIP_NOTIMEOUT);

		//! Reply to the message with acknowledge
		//! @param index message reference index
		//! @param msec_timeout operation timeout
		int ReplyAck(int index,
			int32_t msec_timeout = MESSIP_NOTIMEOUT);

		//! Reply to the message with error indicator
		//! @param index message reference index
		//! @param msec_timeout operation timeout
		int ReplyNack(int index,
			int32_t msec_timeout = MESSIP_NOTIMEOUT);
};

//! Object oriented interface to client-side message passing
class ClientPort {
	private:
		//! underlaying communication channel
		messip_channel_t * ch;

	public:
		//! Construcor
		//! @param name name of the communication channel
		ClientPort(const std::string & name) {
			if ((ch = port_connect(name)) == NULL) {
				// TODO: throw
			}
		}

		//! Destructor
		~ClientPort() {
			if (port_disconnect(ch) == -1) {
				// TODO: throw
			}
		}

		//! Send and receive data with the port
		//! @param type message type
		//! @param subtype message subtype
		//! @param send data to trasmit
		//! @param reply data to receive
		//! @param msec_timeout operation timeout
		template <class SendData, class ReceiveData>
		int Send(int32_t type,
		   int32_t subtype,
		   const SendData & send,
		   ReceiveData & reply,
		   int32_t msec_timeout = MESSIP_NOTIMEOUT);

		//! Send data synchronously with the port
		//! @param type message type
		//! @param subtype message subtype
		//! @param send data to trasmit
		//! @param msec_timeout operation timeout
		template <class SendData>
		int SendSync(int32_t type,
		   int32_t subtype,
		   const SendData & send,
		   int32_t msec_timeout = MESSIP_NOTIMEOUT)
		{
			return port_send_sync(ch, type, subtype, send, msec_timeout);
		}

		//! Send data asynchronously with the port
		//! @param type message type
		//! @param subtype message subtype
		//! @param send data to trasmit
		//! @param msec_timeout operation timeout
		template <class SendData>
		int SendAsync(int32_t type,
		   int32_t subtype,
		   const SendData & send,
		   int32_t msec_timeout = MESSIP_NOTIMEOUT)
		{
			return port_send_async(ch, type, subtype, send, msec_timeout);
		}

		//! Send pulse with the port
		//! @param type message type
		//! @param subtype message subtype
		//! @param msec_timeout operation timeout
		int SendPulse(int32_t type,
		   int32_t subtype,
		   int32_t msec_timeout = MESSIP_NOTIMEOUT);
};

} /* namespace messip */

#endif /* MESSIP_PORT_H_ */
