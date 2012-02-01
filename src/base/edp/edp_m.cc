// -------------------------------------------------------------------------
//                                       edp_m.cc
//
// EDP_MASTER Effector Driver Master Process
// Powloka
//
// Ostatnia modyfikacja:
// -------------------------------------------------------------------------

#include <exception>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <cerrno>
#include <sys/wait.h>

#include <boost/exception/all.hpp>
#include <boost/shared_ptr.hpp>

#include "config.h"

#if defined(HAVE_MLOCKALL)
#	include <sys/mman.h>
#endif /* HAVE_MLOCKALL */

#include "base/lib/configurator.h"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/sr_edp.h"
#include "base/edp/edp_effector.h"
#include "edp_shell.h"
#include "base/lib/mis_fun.h"

namespace mrrocpp {
namespace edp {
namespace common {

// Bufor polecen i odpowiedzi EDP_MASTER
boost::shared_ptr <effector> master;

// obiekt do wykrywania obecnosci drugiego edp jeszcze przed powolaniem klasy efectora
boost::shared_ptr <shell> edp_shell;

/* Przechwycenie sygnalu */
void catch_signal(int sig)
{
	switch (sig)
	{
		case SIGTERM:
		case SIGHUP:
			if (edp_shell) {
				edp_shell->msg->message("edp terminated");
			}
			exit(EXIT_SUCCESS);
			break;
		case SIGSEGV:
			fprintf(stderr, "Segmentation fault in EDP process\n");
			signal(SIGSEGV, SIG_DFL);
			break;
	} // end: switch
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

int main(int argc, char *argv[])
{
	// delay(10000);

	try {
		// allow for empty session name for easier valgrind/tcheck_cl launching
		if (argc < 4) {
			fprintf(stderr, "Usage: edp_m binaries_node_name mrrocpp_path edp_config_section [rsp_attach_name]\n");
			throw std::runtime_error("Usage: edp_m binaries_node_name mrrocpp_path edp_config_section [rsp_attach_name]");
		}

		// przechwycenie SIGTERM
		signal(SIGTERM, &edp::common::catch_signal);
		signal(SIGHUP, &edp::common::catch_signal);
		signal(SIGSEGV, &edp::common::catch_signal);

		// avoid transporting Ctrl-C signal from UI console
		signal(SIGINT, SIG_IGN);

		// create configuration object
		lib::configurator _config(argv[1], argv[2], argv[3]);

		edp::common::edp_shell = (boost::shared_ptr <edp::common::shell>) new edp::common::shell(_config);

		if (!edp::common::edp_shell->detect_hardware_busy()) {
			throw std::runtime_error("hardware busy while loading, closing automatically ...");
		}

#if defined(HAVE_MLOCKALL)
		// Try to lock memory to avoid swapping whlie executing in real-time
		if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
			perror("No real-time warrany: mlockall() failed");
		}
#endif /* HAVE_MLOCKALL */

		lib::set_process_sched();

		edp::common::master =
				(boost::shared_ptr <edp::common::effector>) edp::common::return_created_efector(*(edp::common::edp_shell));

		edp::common::master->initialize_communication();

		edp::common::master->create_threads();

		edp::common::master->msg->message("edp loaded");

		//	printf("1\n");
		//	delay (20000);
		edp::common::master->main_loop();
		//	printf("end\n");
	}

	catch (std::runtime_error & e) {
		printf("edp master runtime error: %s \n", e.what());

		if (edp::common::edp_shell) {
			edp::common::edp_shell->msg->message(lib::FATAL_ERROR, e.what());
			edp::common::edp_shell->msg->message("edp terminated");
		}
	}

	catch (boost::exception & e) {
		std::cerr << diagnostic_information(e);
	}

	catch (std::exception & e) {
		std::cerr << "EDP: " << e.what() << std::endl;
	}

	catch (...) { // Dla zewnetrznej petli try
		perror("Unidentified error in EDP");
		// Komunikat o bledzie wysylamy do SR
		edp::common::master->msg->message(lib::FATAL_ERROR, EDP_UNIDENTIFIED_ERROR);
	}

	return -1;
}
