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

#ifdef __gnu_linux__
#include <execinfo.h>
#include <exception>
#include <iostream>
#endif /* __gnu_linux__ */

#include "base/lib/sr/sr_ecp.h"
#include "base/lib/configurator.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace robot {

// konstruktor wywolywany z UI
ecp_robot_base::ecp_robot_base(const lib::robot_name_t & _robot_name, int _number_of_servos, const std::string &_edp_section, lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg) :
	robot(_robot_name), is_created_by_ui(true), communicate_with_edp(true), sr_ecp_msg(_sr_ecp_msg),
			number_of_servos(_number_of_servos), edp_section(_edp_section)
{
	connect_to_edp(_config);
}

// konstruktor wywolywany z ECP
ecp_robot_base::ecp_robot_base(const lib::robot_name_t & _robot_name, int _number_of_servos, const std::string &_edp_section, common::task::task_base& _ecp_object) :
	robot(_robot_name), is_created_by_ui(false), communicate_with_edp(true), sr_ecp_msg(*_ecp_object.sr_ecp_msg),
			number_of_servos(_number_of_servos), edp_section(_edp_section)
{
	connect_to_edp(_ecp_object.config);
}

// -------------------------------------------------------------------
ecp_robot_base::~ecp_robot_base(void)
{

	if (EDP_fd) {
		messip::port_disconnect(EDP_fd);
	}

	if (is_created_by_ui) {
		if (kill(EDP_MASTER_Pid, SIGTERM) == -1) {
			perror("kill()");
		} else {
			//    		int status;
			//    		if (waitpid(EDP_MASTER_Pid, &status, 0) == -1) {
			//    			perror("waitpid()");
			//    		}
		}
	}
}

void ecp_robot_base::connect_to_edp(lib::configurator &config)
{
	EDP_MASTER_Pid = (is_created_by_ui) ? config.process_spawn(edp_section) : -1;

	const std::string edp_net_attach_point =
			config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point", edp_section);

	printf("connect_to_edp");
	fflush(stdout);

	unsigned int tmp = 0;

	while ((EDP_fd = messip::port_connect(edp_net_attach_point)) == NULL)

	{
		if ((tmp++) < lib::CONNECT_RETRY) {
			usleep(lib::CONNECT_DELAY);
			printf(".");
			fflush(stdout);
		} else {
			int e = errno; // kod bledu systemowego
			fprintf(stderr, "Unable to locate EDP_MASTER process at channel \"%s\": %s\n", edp_net_attach_point.c_str(), strerror(errno));
			sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "Unable to locate EDP_MASTER process");
			throw ECP_main_error(lib::SYSTEM_ERROR, 0);
		}
	}
	printf(".done\n");
}

pid_t ecp_robot_base::get_EDP_pid(void) const
{
	return EDP_MASTER_Pid;
}

bool ecp_robot_base::is_synchronised(void) const
{
	// Czy robot zsynchronizowany?
	return synchronised;
}

ECP_error::ECP_error(lib::error_class_t err_cl, uint64_t err_no, uint64_t err0, uint64_t err1) :
	error_class(err_cl), error_no(err_no)
{
	error.error0 = err0;
	error.error1 = err1;
#ifdef __gnu_linux__
	void * array[25];
	int nSize = backtrace(array, 25);
	char ** symbols = backtrace_symbols(array, nSize);

	for (int i = 0; i < nSize; i++) {
		std::cerr << symbols[i] << std::endl;
	}

	free(symbols);
#endif /* __gnu_linux__ */
}

ECP_main_error::ECP_main_error(lib::error_class_t err_cl, uint64_t err_no) :
	error_class(err_cl), error_no(err_no)
{
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

