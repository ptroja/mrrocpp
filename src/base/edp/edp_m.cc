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
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#endif /* __QNXNTO__ */

#include <boost/exception/all.hpp>

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

namespace mrrocpp {
namespace edp {
namespace common {

effector* master; // Bufor polecen i odpowiedzi EDP_MASTER

#ifdef __QNXNTO__
static _clockperiod old_cp;
static const int TIME_SLICE = 500000; // by Y
#endif /* __QNXNTO__ */

/* Przechwycenie sygnalu */
void catch_signal(int sig)
{
	switch (sig)
	{
		case SIGTERM:
		case SIGHUP:
#ifdef __QNXNTO__
			ClockPeriod(CLOCK_REALTIME, &old_cp, NULL, 0);
#endif /* __QNXNTO__ */
			master->close_hardware_busy_file();
			master->msg->message("edp terminated");

			_exit(EXIT_SUCCESS);
			break;
		case SIGSEGV:
			fprintf(stderr, "Segmentation fault in EDP process\n");
			master->close_hardware_busy_file();
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
			exit(EXIT_FAILURE);
		}

#ifdef __QNXNTO__

		// zmniejszenie stalej czasowej ticksize dla szeregowania
		_clockperiod new_cp;
		new_cp.nsec = edp::common::TIME_SLICE;
		new_cp.fract = 0;
		ClockPeriod(CLOCK_REALTIME, &new_cp, &edp::common::old_cp, 0);
#endif /* __QNXNTO__ */

		// przechwycenie SIGTERM
		signal(SIGTERM, &edp::common::catch_signal);
		signal(SIGHUP, &edp::common::catch_signal);
		signal(SIGSEGV, &edp::common::catch_signal);

		// avoid transporting Ctrl-C signal from UI console

		signal(SIGINT, SIG_IGN);

		// create configuration object
		lib::configurator _config(argv[1], argv[2], argv[3]);

#if defined(HAVE_MLOCKALL)
		// Try to lock memory to avoid swapping whlie executing in real-time
		if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
			perror("mlockall()");
		}
#endif /* HAVE_MLOCKALL */

		edp::common::master = edp::common::return_created_efector(_config);
		if (!edp::common::master->detect_hardware_busy()) {
			return EXIT_FAILURE;
		}

		edp::common::master->create_threads();

		if (!edp::common::master->initialize_communication()) {
			return EXIT_FAILURE;
		}

		//	printf("1\n");
		//	delay (20000);
		edp::common::master->main_loop();
		//	printf("end\n");
	}

	catch (boost::exception & e) {
		std::cerr << diagnostic_information(e);
	}

	catch (System_error & fe) {
		std::cerr << "EDP: System_error" << std::endl;
	}

	catch (std::exception & e) {
		std::cerr << "EDP: " << e.what() << std::endl;
	}

	catch (...) { // Dla zewnetrznej petli try
		perror("Unidentified error in EDP");
		// Komunikat o bledzie wysylamy do SR
		edp::common::master->msg->message(lib::FATAL_ERROR, EDP_UNIDENTIFIED_ERROR);
	}
}
