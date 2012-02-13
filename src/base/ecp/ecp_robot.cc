/*!
 * @file
 * @brief File contains ecp base robot definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <cstring>
#include <cstdio>
#include <cerrno>
#include <csignal>
#include <stdint.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <boost/thread.hpp>

#include "base/lib/sr/sr_ecp.h"
#include "base/lib/configurator.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace robot {

common_buffers_ecp_robot::common_buffers_ecp_robot(const lib::robot_name_t & _robot_name, int _number_of_servos, lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::c_buffer & c_buffer_ref, lib::r_buffer & r_buffer_ref) :
		ecp_robot_base(_robot_name, _number_of_servos, _config, _sr_ecp_msg),
		ecp_command(c_buffer_ref),
		reply_package(r_buffer_ref)
{
}

// konstruktor wywolywany z ECP
common_buffers_ecp_robot::common_buffers_ecp_robot(const lib::robot_name_t & _robot_name, int _number_of_servos, common::task::task_base& _ecp_object, lib::c_buffer & c_buffer_ref, lib::r_buffer & r_buffer_ref) :
		ecp_robot_base(_robot_name, _number_of_servos, _ecp_object),
		ecp_command(c_buffer_ref),
		reply_package(r_buffer_ref)
{
}

common_buffers_ecp_robot::~common_buffers_ecp_robot()
{
}

// konstruktor wywolywany z UI
ecp_robot_base::ecp_robot_base(const lib::robot_name_t & _robot_name, int _number_of_servos, lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg) :
		robot(_robot_name),
		communicate_with_edp(true),
		sr_ecp_msg(_sr_ecp_msg),
		number_of_servos(_number_of_servos),
		edp_section(_config.get_edp_section(robot_name)),
		is_new_data(false),
		is_new_request(false),
		data_ports_used(false)
{
	EDP_pid = _config.process_spawn(edp_section);

	connect_to_edp(_config);
}

// konstruktor wywolywany z ECP
ecp_robot_base::ecp_robot_base(const lib::robot_name_t & _robot_name, int _number_of_servos, common::task::task_base& _ecp_object) :
		//ecp_robot_base(_robot_name, _number_of_servos, _ecp_object.config, *_ecp_object.sr_ecp_msg)
		robot(_robot_name),
		communicate_with_edp(true),
		sr_ecp_msg(*_ecp_object.sr_ecp_msg),
		number_of_servos(_number_of_servos),
		edp_section(_ecp_object.config.get_edp_section(robot_name)),
		is_new_data(false),
		is_new_request(false),
		data_ports_used(false)
{
	EDP_pid = -1;

	connect_to_edp(_ecp_object.config);
}

ecp_robot_base::~ecp_robot_base()
{
	// Close and invalidate the connection with EDP
	if (EDP_fd != lib::invalid_fd) {
		messip::port_disconnect(EDP_fd);
	}

	if (EDP_pid != -1) {
		if (kill(EDP_pid, SIGTERM) == -1) {
			perror("kill()");
		} else {
			//    		int status;
			//    		if (waitpid(EDP_pid, &status, 0) == -1) {
			//    			perror("waitpid()");
			//    		}
		}
	}
}

void ecp_robot_base::connect_to_edp(lib::configurator &config)
{
	const std::string edp_net_attach_point = config.get_edp_resourceman_attach_point(robot_name);

	printf("connect_to_edp");
	fflush(stdout);

	unsigned int tmp = 0;

	while ((EDP_fd = messip::port_connect(edp_net_attach_point)) == lib::invalid_fd)
	{
		if ((tmp++) < lib::CONNECT_RETRY) {
			boost::this_thread::sleep(lib::CONNECT_DELAY);
			printf(".");
			fflush(stdout);
		} else {
			int e = errno; // kod bledu systemowego
			fprintf(stderr, "Unable to locate EDP_MASTER process at channel \"%s\": %s\n", edp_net_attach_point.c_str(), strerror(e));
			sr_ecp_msg.message(lib::SYSTEM_ERROR, e, ": Unable to locate EDP_MASTER process");
			BOOST_THROW_EXCEPTION(exception::se_r());
		}
	}

	printf(".done\n");
}

pid_t ecp_robot_base::get_EDP_pid(void) const
{
	return EDP_pid;
}

bool ecp_robot_base::is_synchronised(void) const
{
	// Czy robot zsynchronizowany?
	return synchronised;
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
