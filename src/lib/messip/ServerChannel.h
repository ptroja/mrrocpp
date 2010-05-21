/*
 * ServerChannel.h
 *
 *  Created on: May 19, 2010
 *      Author: ptroja
 */

#ifndef SERVERCHANNEL_H_
#define SERVERCHANNEL_H_

namespace messip {

//! Object oriented interface to server-side message passing
class ServerChannel {
	private:
		//! underlaying communication channel
		messip_channel_t * ch;

	public:
		//! Construcor
		//! @param name name of the communication channel
		ServerChannel(const std::string & name) {
			if ((ch = port_create(name)) == NULL) {
				// TODO: throw
			}
		}

		//! Destructor
		~ServerChannel() {
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

} // namespace

#endif /* SERVERCHANNEL_H_ */
