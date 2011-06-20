/*
 * rendezvous.h
 *
 *  Created on: Jun 8, 2011
 *      Author: ptroja
 */

#ifndef RENDEZVOUS_H_
#define RENDEZVOUS_H_

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

template<typename SENT_T, typename REPLY_T>
class Rendezvous {
private:
	bool sent;
	bool replied;
	boost::condition_variable cond;
	boost::mutex mtx;
	SENT_T * sdata;
	REPLY_T * rdata;
public:
	Rendezvous() :
		sent(false), replied(false)
	{}

	void Send(const SENT_T & smsg, REPLY_T & rmsg)
	{
		boost::lock<boost::mutex> lock(mtx);

		sdata = &smsg;

		cond.notify_one();

		while(!replied)
			cond.wait(lock);

		rmsg = *rdata;
		replied = false;
	};

	void Receive(SENT_T & msg)
	{
		boost::lock<boost::mutex> lock(mtx);

		while(!sent)
			cond.wait(lock);

		msg = *sdata;
		sent = false;
	};

	void Reply(const REPLY_T & msg)
	{
		boost::lock<boost::mutex> lock(mtx);

		rdata = &msg;
		replied = true;

		cond.notify_one();
	}
};

#endif /* RENDEZVOUS_H_ */
