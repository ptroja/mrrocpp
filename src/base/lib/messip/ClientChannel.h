/*
 * ClientChannel.h
 *
 *  Created on: May 19, 2010
 *      Author: ptroja
 */

#ifndef CLIENTCHANNEL_H_
#define CLIENTCHANNEL_H_

namespace messip {

//! Object oriented interface to client-side message passing
class ClientChannel {
	private:
		//! underlaying communication channel
		messip_channel_t * ch;

	public:
		//! Construcor
		//! @param name name of the communication channel
		ClientChannel(const std::string & name) {
			if ((ch = port_connect(name)) == NULL) {
				// TODO: throw
			}
		}

		//! Destructor
		~ClientChannel() {
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

} // namespace

#endif /* CLIENTCHANNEL_H_ */
