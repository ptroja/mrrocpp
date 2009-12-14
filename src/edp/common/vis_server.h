// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __VIS_SERVER_H
#define __VIS_SERVER_H

#include <boost/thread/thread.hpp>
#include <boost/utility.hpp>

#include "edp/common/edp_extension_thread.h"

namespace mrrocpp {
namespace edp {
namespace common {

// TODO: remove forward declarations
class manip_and_conv_effector;

class vis_server : boost::noncopyable
{
private:
	manip_and_conv_effector &master;

	boost::thread *thread_id;

public:
	//! main loop
	void operator()();

    vis_server(manip_and_conv_effector &_master);
    ~vis_server();
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
