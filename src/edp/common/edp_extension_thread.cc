#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>

#include "lib/impconst.h"
#include "edp/common/edp_effector.h"
#include "edp/common/edp_extension_thread.h"



namespace mrrocpp {
namespace edp {
namespace common {


edp_extension_thread::edp_extension_thread(effector &_master) :
	master (_master)
{}

edp_extension_thread::~edp_extension_thread()
{}

/*
void * edp_extension_thread::thread_start(void* arg)
{
	return static_cast<edp_extension_thread*> (arg)->thread_main_loop(arg);
}
*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

