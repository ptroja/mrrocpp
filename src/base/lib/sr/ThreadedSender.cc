/*!
 * @file ThreadedSender.cc
 * @brief Threaded system reporting sender - definitions.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <cstdio>

#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "base/lib/sr/ThreadedSender.h"
#include "base/lib/sr/SenderBase.h"
#include "base/lib/sr/srlib.h"

namespace mrrocpp {
namespace lib {

ThreadedSender::ThreadedSender(const std::string & sr_name) :
	SenderBase(sr_name), cb(SR_BUFFER_LENGHT), has_command(false)
{
	thread_id = boost::thread(boost::bind(&ThreadedSender::operator(), this));
}

ThreadedSender::~ThreadedSender()
{
}

void ThreadedSender::send_package(const lib::sr_package_t& new_msg)
{
	boost::unique_lock <boost::mutex> lock(mtx);
	cb.push_back(new_msg);

	has_command = true;

	cond.notify_one();
}

void ThreadedSender::get_one_msg(lib::sr_package_t& new_msg)
{
	boost::unique_lock <boost::mutex> lock(mtx);
	new_msg = cb.front();
	cb.pop_front();
	has_command = false;
}

bool ThreadedSender::buffer_empty() const
{
	boost::unique_lock <boost::mutex> lock(mtx);
	return cb.empty();
}

void ThreadedSender::wait_for_new_msg()
{
	boost::unique_lock <boost::mutex> lock(mtx);

	while (!has_command) {
		cond.wait(lock);
	}
}

void ThreadedSender::operator()()
{
	while (true) {
		wait_for_new_msg();

		while (!buffer_empty()) {
			sr_package_t local_message;
			get_one_msg(local_message);
			try {
				Send(local_message);
			} catch (...) {
				printf("send_package_to_sr error multi_thread variant\n");
			}
		}
	}
}

} // namespace lib
} // namespace mrrocpp
