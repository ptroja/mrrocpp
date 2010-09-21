/*!
 * @file condition_synchroniser.cc
 * @brief Multi-thread synchronization on a condition.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include "base/lib/condition_synchroniser.h"

namespace mrrocpp {
namespace lib {

condition_synchroniser::condition_synchroniser() :
	has_command(false)
{
}

void condition_synchroniser::command()
{
	boost::unique_lock <boost::mutex> lock(mtx);

	// assign command for execution

	has_command = true;

	cond.notify_one();
}

void condition_synchroniser::null_command()
{
	boost::unique_lock <boost::mutex> lock(mtx);

	has_command = false;
}

void condition_synchroniser::wait()
{
	boost::unique_lock <boost::mutex> lock(mtx);

	while (!has_command) {
		cond.wait(lock);
	}

	has_command = false;
}

} // namespace lib
} // namespace mrrocpp
