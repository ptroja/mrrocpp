// -------------------------------------------------------------------------
//                                       edp_m.cc
//
// EDP_MASTER Effector Driver Master Process
// Powloka
//
// Ostatnia modyfikacja:
// -------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <sys/wait.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#endif /* __QNXNTO__ */
#include <pthread.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "edp/common/edp_effector.h"

namespace mrrocpp {
namespace edp {
namespace common {

effector* master; // Bufor polecen i odpowiedzi EDP_MASTER

#ifdef __QNXNTO__
static _clockperiod old_cp;
#endif /* __QNXNTO__ */

/* Przechwycenie sygnalu */
void catch_signal(int sig) {
	switch (sig) {
	case SIGTERM:
#ifdef __QNXNTO__
		ClockPeriod(CLOCK_REALTIME, &old_cp, NULL, 0);
#endif /* __QNXNTO__ */
		master->msg->message("EDP terminated");
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

int main(int argc, char *argv[], char **arge) {

	// delay(10000);

	try {
		if (argc < 6) {
			fprintf(
					stderr, "Usage: edp_m binaries_node_name mrrocpp_path config_file edp_config_section <session_name> [rsp_attach_name]\n");
			exit(EXIT_FAILURE);
		}

#ifdef __QNXNTO__
		// zmniejszenie stalej czasowej ticksize dla szeregowania
		_clockperiod new_cp;
		new_cp.nsec = TIME_SLICE; // impconst.h
		new_cp.fract = 0;
		ClockPeriod(CLOCK_REALTIME, &new_cp, &edp::common::old_cp, 0);
#endif /* __QNXNTO__ */

		// przechwycenie SIGTERM
		signal(SIGTERM, &edp::common::catch_signal);
		signal(SIGSEGV, &edp::common::catch_signal);
#if defined(PROCESS_SPAWN_RSH)
		signal(SIGINT, SIG_IGN);
#endif
#ifndef	__QNXNTO__
	    /* Block timer signal for test mode timer in Linux*/
	    fprintf(stderr, "Blocking signal %d\n", SIGRTMIN);
	    sigset_t mask;
	    if (sigemptyset (&mask) == -1) {
	    	perror("sigemptyset()");
	    }
	    if (sigaddset (&mask, SIGRTMIN) == -1) {
	    	perror("sigaddset()");
	    }
	    if (sigprocmask (SIG_BLOCK, &mask, NULL)) {
	    	perror("sigprocmask()");
	    }
#endif /* __QNXNTO__ */

		// odczytanie konfiguracji
		lib::configurator * _config = new lib::configurator(argv[1], argv[2], argv[3],
				argv[4], argv[5]);

		/* Lokalizacja procesu wywietlania komunikatow SR */
		/*
		 if ((msg = new lib::sr_edp(lib::EDP, config->return_string_value("resourceman_attach_point"),
		 config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", "[ui]"))) == NULL) {
		 perror ( "Unable to locate SR ");
		 throw System_error();
		 }
		 */
		//	printf("przed\n");
		//		delay(10000);
		//			printf("za\n");
		edp::common::master = edp::common::return_created_efector(*_config);

		edp::common::master->initialize();

		edp::common::master->create_threads();

		if (!edp::common::master->initialize_communication()) {
			return EXIT_FAILURE;
		}

		//	printf("1\n");
		//	delay (20000);
		edp::common::master->main_loop();
		//	printf("end\n");
	}

	catch (edp::common::System_error fe) {
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



