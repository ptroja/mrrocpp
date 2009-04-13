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
#include <process.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/types.h>
#include <sys/sched.h>
#include <sys/netmgr.h>
#include <pthread.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "edp/common/edp.h"

namespace mrrocpp {
namespace edp {
namespace common {

// sr_edp *msg;                // Wskaznik na obiekt do komunikacji z SR

effector* master; // Bufor polecen i odpowiedzi EDP_MASTER

static _clockperiod old_cp;

/* Przechwycenie sygnalu */
void catch_signal(int sig) {
	switch (sig) {
	case SIGTERM:
		ClockPeriod(CLOCK_REALTIME, &old_cp, NULL, 0);
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
	// STATE next_state;    // stan nastepny, do ktorego przejdzie EDP_MASTER

	try {
		if (argc < 6) {
			fprintf(
					stderr, "Usage: edp_m binaries_node_name mrrocpp_path config_file edp_config_section <session_name> [rsp_attach_name]\n");
			exit(EXIT_FAILURE);
		}

		// zmniejszenie stalej czasowej ticksize dla szeregowania
		_clockperiod new_cp;
		new_cp.nsec = TIME_SLICE; // impconst.h
		new_cp.fract = 0;
		ClockPeriod(CLOCK_REALTIME, &new_cp, &edp::common::old_cp, 0);

		// przechwycenie SIGTERM
		signal(SIGTERM, &edp::common::catch_signal);
		signal(SIGSEGV, &edp::common::catch_signal);
#if defined(PROCESS_SPAWN_RSH)
		signal(SIGINT, SIG_IGN);
#endif

		// odczytanie konfiguracji
		lib::configurator * _config = new lib::configurator(argv[1], argv[2], argv[3],
				argv[4], argv[5]);

#if defined(PROCESS_SPAWN_YRSH)
		if (argc > 6) {
			_config->answer_to_y_rsh_spawn(argv[6]);
			signal(SIGINT, SIG_IGN);
		}
#endif
		/* Lokalizacja procesu wywietlania komunikatow SR */
		/*
		 if ((msg = new sr_edp(EDP, config->return_string_value("resourceman_attach_point"),
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
		edp::common::master->msg->message(FATAL_ERROR, EDP_UNIDENTIFIED_ERROR);
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



