#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

#ifndef __CONDITION_SYNCHRONIZER_H
#define __CONDITION_SYNCHRONIZER_H

namespace mrrocpp {
namespace lib {

class condition_synchroniser {
public:
	// wait for condition and new_command;
	void wait();
	// set new_command;
	void command();
	// set has_command=0;
	void null_command();
	condition_synchroniser();

private:
	boost::condition_variable cond; //! active command condition
	boost::mutex mtx; //! mutex related to condition variable
	bool has_command; //! flag indicating active command to execute
};

} // namespace lib
} // namespace mrrocpp

#endif /* __CONDITION_SYNCHRONIZER_H */
