// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __VIS_SERVER_H
#define __VIS_SERVER_H

#include <boost/thread/thread.hpp>
#include <boost/utility.hpp>

namespace mrrocpp {
namespace edp {
namespace common {

// TODO: remove forward declarations
class motor_driven_effector;

class vis_server : boost::noncopyable
{
private:
	motor_driven_effector &master;

	boost::thread *thread_id;

public:
	//! main loop
	void operator()();

    vis_server(motor_driven_effector &_master);
    ~vis_server();
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
