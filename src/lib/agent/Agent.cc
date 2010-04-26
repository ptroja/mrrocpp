#include <iostream>
#include <exception>

#include <boost/foreach.hpp>
#include <boost/serialization/string.hpp>

#include "lib/messip/messip.h"
#include "../xdr_iarchive.hpp"

#include "DataBuffer.hh"

void Agent::registerBuffer(DataBufferBase & buf)
{
	std::pair<buffers_t::iterator, bool> result = buffers.insert(buffer_item_t(buf.getName(), &buf));

	if(!result.second) {
		std::cerr << "Duplicated name of the buffer ('" << buf.getName() << "')" << std::endl;
		throw;
	}
}

void Agent::listBuffers() const
{
	std::cout << "Buffer list of \"" << getName() << "\"[" << buffers.size() << "]:" << std::endl;
	BOOST_FOREACH(buffer_item_t item, buffers) {
		std::cout << "\t" << item.first << std::endl;
	}
}

Agent::Agent(const std::string & _name) : AgentBase(_name)
{
#if defined(USE_MESSIP_SRR)
	channel = messip_channel_create(NULL, getName().c_str(), MESSIP_NOTIMEOUT, 0);
#else /* USE_MESSIP_SRR  */
	channel = name_attach(NULL, getName().c_str(), 0 /*NAME_FLAG_ATTACH_GLOBAL*/);
#endif /* USE_MESSIP_SRR */

	if(channel == NULL)
	{
		// TODO:
		std::cerr << "server channel create for '" << getName() << "' failed" << std::endl;
		throw;
	}

	tid = new boost::thread(&Agent::ReceiveDataLoop, this);
}

Agent::~Agent()
{
	// TODO: handle cancelation of the thread?
	tid->join();
	delete tid;

#if defined(USE_MESSIP_SRR)
	if(messip_channel_delete(channel, MESSIP_NOTIMEOUT) == -1)
#else /* USE_MESSIP_SRR  */
	if(name_detach(channel, 0) == -1)
#endif /* USE_MESSIP_SRR */
	{
		// TODO:
		std::cerr << "server channel create for for '" << getName() << "' failed" << std::endl;
		throw;
	}
}

void Agent::operator ()()
{
	do {
	} while (step());
}

bool Agent::checkCondition(const OrBufferContainer &condition)
{
//	BOOST_FOREACH(const AndBufferContainer & andCondition, condition) {
//		std::cout << "(";
//		BOOST_FOREACH(const DataBufferBase * ptr, andCondition) {
//			std::cout << ptr->getName() << " & ";
//		}
//		std::cout << ") | ";
//	}

	BOOST_FOREACH(const AndBufferContainer & andCondition, condition) {
		if (andCondition.isFresh())
			return true;
	}
	return false;
}

void Agent::ReceiveDataLoop(void)
{
	// receive thread loop
	while(true) {
#if defined(USE_MESSIP_SRR)
		// buffer for the message to receive
		char msg[4096];

		// additional message type/subtype information
		int32_t type, subtype;

		int ret = messip_receive(channel, &type, &subtype, msg, sizeof(msg), MESSIP_NOTIMEOUT);

		if (ret == -1) {
			std::cerr << "messip_receive() failed" << std::endl;
			throw;
		}
		else if (ret != MESSIP_MSG_NOREPLY) {
			std::cerr << "Unknown message received (" << ret << ")" << std::endl;
			continue;
		}

		// initialize the archive with the data received
		xdr_iarchive<sizeof(msg)> ia(msg,channel->datalenr);
#else /* USE_MESSIP_SRR */
		// data structure with additional message info
		_msg_info info;

		/* We specify the header as being at least a pulse */
		typedef struct _pulse msg_header_t;

		// a message data including the required header
		struct _msg {
			msg_header_t hdr;
			char data[4096-sizeof(msg_header_t)];
		} msg;

		while(true) {
			int rcvid = MsgReceive(channel->chid, &msg, sizeof(msg), &info);

			/* MsgReceive failed */
			if (rcvid == -1) {
				perror("MsgReceive()");
				// TODO:
				throw;
			}

			/* Pulse received */
			if (rcvid == 0) {
				std::cerr << "Pulse received" << std::endl;
				switch (msg.hdr.code) {
					case _PULSE_CODE_DISCONNECT:
						ConnectDetach(msg.hdr.scoid);
						break;
					case _PULSE_CODE_UNBLOCK:
						break;
					default:
						break;
					}
				continue;
			}

			/* A QNX IO message received, reject */
			if (msg.hdr.type >= _IO_BASE && msg.hdr.type <= _IO_MAX) {
				std::cerr << "A QNX IO message received, reject" << std::endl;

				MsgReply(rcvid, EOK, 0, 0);
				continue;
			}

			// Unblock the sender
			MsgReply(rcvid, EOK, 0, 0);

			// handle a new message
			break;
		}

		// initialize the archive with the data received
		xdr_iarchive<sizeof(msg)> ia((const char *) &msg,(std::size_t) info.msglen);
#endif
		// TODO: make this real-time safe
		std::string msg_buffer_name;

		ia >> msg_buffer_name;

//		std::cout << "Message received for data buffer: "
//			<< msg_buffer_name << ", size "
//#if (USE_MESSIP_SRR)
//			<< channel->datalenr << std::endl;
//#else
//			<< info.msglen << std::endl;
//#endif

		// lock access to the data buffers
		boost::unique_lock<boost::mutex> lock(mtx);

		buffers_t::iterator result = buffers.find(msg_buffer_name);
		if (result != buffers.end()) {
			result->second->Set(ia);
		} else {
			// TODO: exception?
			std::cerr << "Message received for unknown buffer '"
				<< msg_buffer_name
				<< "'" << std::endl;
		}

		cond.notify_all();
	}
}

void Agent::Wait(OrBufferContainer & condition)
{
	boost::unique_lock<boost::mutex> lock(mtx);
	while(!checkCondition(condition))
		cond.wait(lock);
}

void Agent::Wait(AndBufferContainer & andCondition)
{
	OrBufferContainer orCondition(andCondition);
	Wait(orCondition);
}

void Agent::Wait(DataBufferBase & dataBufferCondition)
{
	OrBufferContainer orCondition(dataBufferCondition);
	Wait(orCondition);
}
