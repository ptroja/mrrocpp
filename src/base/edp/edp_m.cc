// -------------------------------------------------------------------------
//                                       edp_m.cc
//
// EDP_MASTER Effector Driver Master Process
// Powloka
//
// Ostatnia modyfikacja:
// -------------------------------------------------------------------------

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

// niezbedny naglowek z definiacja PROCESS_SPAWN_RSH
#include "base/lib/configurator.h"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"
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
#ifdef __QNXNTO__
			ClockPeriod(CLOCK_REALTIME, &old_cp, NULL, 0);
#endif /* __QNXNTO__ */
			master->sh_msg->message("edp terminated");
			_exit(EXIT_SUCCESS);
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
		if (argc < 5) {
			fprintf(stderr, "Usage: edp_m binaries_node_name mrrocpp_path config_file edp_config_section <session_name> [rsp_attach_name]\n");
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
		signal(SIGSEGV, &edp::common::catch_signal);

		// avoid transporting Ctrl-C signal from UI console
#if defined(PROCESS_SPAWN_RSH)
		signal(SIGINT, SIG_IGN);
#endif

		// create configuration object
		lib::configurator _config(argv[1], argv[2], argv[3], argv[4], (argc < 6) ? "" : argv[5]);

		// block test-mode timer signal for all the threads
		if (_config.value <int> (lib::ROBOT_TEST_MODE)) {
			/* Block timer signal from test mode timer for all threads */
			//		    fprintf(stderr, "Blocking signal %d\n", SIGRTMIN);
			sigset_t mask;
			if (sigemptyset(&mask) == -1) {
				perror("sigemptyset()");
			}
			if (sigaddset(&mask, SIGRTMIN) == -1) {
				perror("sigaddset()");
			}
			if (sigprocmask(SIG_BLOCK, &mask, NULL)) {
				perror("sigprocmask()");
			}
		}

		edp::common::master = edp::common::return_created_efector(_config);

		edp::common::master->create_threads();

		if (!edp::common::master->initialize_communication()) {
			return EXIT_FAILURE;
		}

		//	printf("1\n");
		//	delay (20000);
		edp::common::master->main_loop();
		//	printf("end\n");
	}

	catch (System_error & fe) {
		// Obsluga bledow systemowych
		/*
		 // Wystapil blad w komunikacji miedzyprocesowej, oczekiwanie na jawne
		 // zabicie procesu przez operatora
		 for (;;) {
		 delay(100);
		 //   printf("\a"); // Sygnal dzwiekowy
		 }
		 */
	} // end: catch(System_error fe)

	catch (...) { // Dla zewnetrznej petli try
		perror("Unidentified error in EDP");
		// Komunikat o bledzie wysylamy do SR
		edp::common::master->msg->message(lib::FATAL_ERROR, EDP_UNIDENTIFIED_ERROR);
		/*
		 // Wystapil niezidentyfikowany blad, oczekiwanie na jawne zabicie procesu
		 // przez operatora

		 for (;;) {
		 delay(100);
		 // printf("\a"); // Sygnal dzwiekowy
		 }
		 */
	}
}
