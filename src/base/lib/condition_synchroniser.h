/*!
 * @file condition_synchroniser.h
 * @brief Multi-thread synchronization on a condition.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

#ifndef __CONDITION_SYNCHRONIZER_H
#define __CONDITION_SYNCHRONIZER_H

namespace mrrocpp {
namespace lib {

/**
 * Synchronize multiple threads on a boolean condition
 */
class condition_synchroniser
{
public:
	//! Wait for condition
	void wait();

	//! Signal the condition
	void command();

	//! Null the condition
	void null_command();

	//! Constructor
	condition_synchroniser();

private:
	//! Active command condition
	boost::condition_variable cond;

	//! Mutex related to condition variable
	boost::mutex mtx;

	//! Flag indicating active command to execute
	bool has_command;
};

} // namespace lib
} // namespace mrrocpp

#endif /* __CONDITION_SYNCHRONIZER_H */
