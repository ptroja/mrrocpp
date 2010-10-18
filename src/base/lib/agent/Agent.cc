#include <iostream>
#include <exception>

#include <boost/foreach.hpp>
#include <boost/serialization/string.hpp>
#include <boost/thread/locks.hpp>

#include "base/lib/messip/messip.h"
#include "base/lib/xdr/xdr_iarchive.hpp"

#include "DataBuffer.h"

void Agent::Start(void)
{
	loop_tid = boost::thread(&Agent::operator(), this);
}

void Agent::Join(void)
{
	loop_tid.join();
}

void Agent::registerBuffer(DataBufferBase & buf)
{
	std::pair<buffers_t::iterator, bool> result;

	{
		// lock access to the data buffers
		boost::unique_lock<boost::mutex> lock(mtx);

		result = buffers.insert(buffer_item_t(buf.getName(), &buf));
	}

	if(!result.second) {
		std::cerr << "Duplicated buffer ('" << buf.getName() << "')" << std::endl;
		throw;
	}
}

void Agent::listBuffers() const
{
	// lock access to the data buffers
	boost::unique_lock<boost::mutex> lock(mtx);

	std::cout << "Buffer list of \"" << getName() << "\"[" << buffers.size() << "]:" << std::endl;

	BOOST_FOREACH(const buffer_item_t item, buffers) {
		std::cout << "\t" << item.first << std::endl;
	}
}

Agent::Agent(const std::string & _name)
	: AgentBase(_name)
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

	tid = boost::thread(&Agent::ReceiveDataLoop, this);
}

Agent::~Agent()
{
#if defined(BOOST_THREAD_PLATFORM_PTHREAD)
	// handle cancelation of the receiver thread

	// cancel the thread
	if(pthread_cancel(tid.native_handle()) != 0) {
		fprintf(stderr, "pthread_cancel() failed\n");
	}

	// wait after finishing
	void *retval;
	if(pthread_join(tid.native_handle(), &retval) != 0) {
		fprintf(stderr, "pthread_join() failed\n");
	}

	// check if thread was properly cancelled
	if(retval != PTHREAD_CANCELED) {
		fprintf(stderr, "retval != PTHREAD_CANCELED\n");
	}
#endif

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

int Agent::ReceiveMessage(void * msg, std::size_t msglen, bool block)
{
#if defined(USE_MESSIP_SRR)
	// receive loop
	while(true) {
		// additional message type/subtype information
		int32_t type, subtype;

		int ret = messip_receive(channel, &type, &subtype, msg, msglen, block ? MESSIP_NOTIMEOUT : 0);

		if (ret == -1) {
			std::cerr << "messip_receive() failed" << std::endl;
			throw;
		}
		else if (!block && ret == MESSIP_MSG_TIMEOUT) {
			return -1;
		}
		else if (ret != MESSIP_MSG_NOREPLY) {
//			std::cerr << "Unknown message received (" << ret << ")" << std::endl;
			continue;
		}

		return channel->datalenr;
	}
#else /* USE_MESSIP_SRR */
	while(true) {
		if(!block) {
			if (TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, NULL, NULL, NULL) == -1) {
				perror("TimerTimeout()");
				// TODO: exception
				throw;
			}
		}

		// data structure with additional message info
		_msg_info info;

		int rcvid = MsgReceive(channel->chid, msg, msglen, &info);

		/* MsgReceive failed */
		if (rcvid == -1) {
			if(!block && errno == ETIMEDOUT) {
				return -1;
			} else {
				perror("MsgReceive()");
				// TODO:
				throw;
			}
		}

		/* We specify the header as being at least a pulse */
		typedef struct _pulse msg_header_t;

		msg_header_t * hdr = (msg_header_t *) msg;

		/* Pulse received */
		if (rcvid == 0) {
			std::cerr << "Pulse received" << std::endl;
			switch (hdr->code) {
				case _PULSE_CODE_DISCONNECT:
					ConnectDetach(hdr->scoid);
					break;
				case _PULSE_CODE_UNBLOCK:
					break;
				default:
					break;
				}
			continue;
		}

		/* A QNX IO message received, reject */
		if (hdr->type >= _IO_BASE && hdr->type <= _IO_MAX) {
			std::cerr << "A QNX IO message received, reject" << std::endl;

			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}

		// Unblock the sender
		MsgReply(rcvid, EOK, 0, 0);

		return info.msglen;
	}
#endif /* USE_MESSIP_SRR */
}

void Agent::ReceiveDataLoop(void)
{
	// create object for real-time usage in the loop
	std::string msg_buffer_name;

	// allocate memory for the string object
	// TODO: this size should be #defined for the whole library
	msg_buffer_name.reserve(100);

	while(true) {
		// buffer for the message to receive
		char msg[16384];

		int msglen = ReceiveMessage(msg, sizeof(msg), true);

		// initialize the archive with the data received
		xdr_iarchive<sizeof(msg)> ia(msg,msglen);

		ia >> msg_buffer_name;

		// lock access to the data buffers
		boost::mutex::scoped_lock l(mtx);

		Store(msg_buffer_name, ia);

		do {
			// try to receive the message
			if((msglen = ReceiveMessage(msg, sizeof(msg), false)) == -1)
				break;

			// initialize the archive with the data received
			xdr_iarchive<sizeof(msg)> ia(msg,msglen);

			ia >> msg_buffer_name;

			Store(msg_buffer_name, ia);
		} while(true);

		cond.notify_all();
	}
}

void Agent::Wait(DataCondition & condition)
{
	boost::unique_lock<boost::mutex> lock(mtx);

	while(!condition.isNewData())
		cond.wait(lock);

	BOOST_FOREACH(const buffer_item_t item, buffers) {
		item.second->Update();
	}
}

void Agent::Wait(void)
{
	boost::unique_lock<boost::mutex> lock(mtx);

	BOOST_FOREACH(const buffer_item_t item, buffers) {
		item.second->Update();
	}
}
