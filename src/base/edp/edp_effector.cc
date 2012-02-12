// ------------------------------------------------------------------------
//                                  edp.cc
//
// EDP_MASTER Effector Driver Master Process
// -------------------------------------------------------------------------

#include <cstdio>
#include <cstring>

#include "base/lib/messip/messip_dataport.h"

#include "base/lib/mis_fun.h"
#include "base/edp/edp_effector.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*--------------------------------------------------------------------------*/
effector::effector(shell &_shell, const lib::robot_name_t & l_robot_name) :
	server_attach(lib::invalid_fd),
	edp_shell(_shell),
	robot_name(l_robot_name),
	config(_shell.config),
	msg(_shell.msg),
	robot_test_mode(true)
{

	if (config.exists(lib::ROBOT_TEST_MODE)) {
		robot_test_mode = config.exists_and_true(lib::ROBOT_TEST_MODE);
	}

	if (robot_test_mode) {
		msg->message("Robot test mode activated");
	}

}

effector::~effector()
{
	if(server_attach != lib::invalid_fd) {
		messip::port_delete(server_attach);
	}
}

/*--------------------------------------------------------------------------*/
void effector::initialize_communication()
{
	const std::string
			server_attach_point(config.get_edp_resourceman_attach_point());

	server_attach = messip::port_create(server_attach_point);

	if (server_attach == lib::invalid_fd) {
		msg->message(lib::SYSTEM_ERROR, errno, "failed to name attach");
		fprintf(stderr, "name_attach() failed: %s\n", strerror(errno));
		throw std::runtime_error("communication error");
	}

	/* Ustawienie priorytetu procesu */
	if(!robot_test_mode) {
		lib::set_thread_priority(lib::PTHREAD_MAX_PRIORITY - 2);
	}

}

void effector::establish_error(lib::r_buffer_base & reply, uint64_t err0, uint64_t err1)
{
	reply.reply_type = lib::ERROR;
	reply.error_no.error0 = err0;
	reply.error_no.error1 = err1;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
