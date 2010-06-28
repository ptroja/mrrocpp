// ------------------------------------------------------------------------
//
//                      MASTER PROCESS (MP) - main()
//
// ------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "base/mp/mp.h"

#include <exception>
#include <boost/exception/get_error_info.hpp>
#include <boost/exception/diagnostic_information.hpp>

#include "lib/exception.h"

namespace mrrocpp {
namespace mp {
namespace common {

// obiekt z metodami i polami dla procesu MP (polimorficzny)
task::task* mp_t;

void catch_signal_in_mp(int sig)
{
	// print info message
	fprintf(stderr, "MP: %s\n", strsignal(sig));
	pid_t child_pid;
	int status;
	switch (sig) {
	case SIGTERM:
		mp_t->sh_msg->message("MP terminated");
		// restore default (none) handler for SIGCHLD
		signal(SIGCHLD, SIG_DFL);
		delete mp_t;
		exit(EXIT_SUCCESS);
		break;
	case SIGSEGV:
		signal(SIGSEGV, SIG_DFL);
		break;
	case SIGCHLD:
		child_pid = waitpid(-1, &status, 0);
		if (child_pid == -1) {
			perror("MP: waitpid()");
		} else if (child_pid == 0) {
			fprintf(stderr, "MP: no child exited\n");
		} else {
			//fprintf(stderr, "UI: child %d...\n", child_pid);
			if (WIFEXITED(status)) {
				fprintf(stderr, "MP: child %d exited normally with status %d\n",
						child_pid, WEXITSTATUS(status));
			}
			if (WIFSIGNALED(status)) {
#ifdef WCOREDUMP
				if (WCOREDUMP(status)) {
					fprintf(stderr, "MP: child %d terminated by signal %d (core dumped)\n",
							child_pid, WTERMSIG(status));
				}
				else
#endif /* WCOREDUMP */
				{
					fprintf(stderr, "MP: child %d terminated by signal %d\n",
							child_pid, WTERMSIG(status));
				}
			}
			if (WIFSTOPPED(status)) {
				fprintf(stderr, "MP: child %d stopped\n", child_pid);
			}
			if (WIFCONTINUED(status)) {
				fprintf(stderr, "MP: child %d resumed\n", child_pid);
			}
		}
		break;
	}
	flushall();
}

} // namespace common
} // namespace mp
} // namespace mrrocpp



int main (int argc, char *argv[], char **arge)
{
	// zewnetrzne try
	try {

		if (argc < 6) {
			printf("Usage: mp_m_c <ui_node_name> <mrrocpp_local_path> <config_file> <session_name>\n");
			exit(EXIT_FAILURE);
		}

		// TODO: new/delete fixup
		lib::configurator * _config = new lib::configurator(argv[1], argv[2], argv[3], MP_SECTION, argv[5]);

		mp::common::mp_t = mp::task::return_created_mp_task(*_config);

		mp::common::mp_t->sr_ecp_msg->message("MP loaded");

		lib::set_thread_priority(pthread_self(), MAX_PRIORITY-4);

		signal(SIGTERM, &(mp::common::catch_signal_in_mp));
		//signal(SIGINT,  &(catch_signal_in_mp));
		signal(SIGSEGV, &(mp::common::catch_signal_in_mp));
		signal(SIGCHLD, &(mp::common::catch_signal_in_mp));
#if defined(PROCESS_SPAWN_RSH)
		// ignore Ctrl-C signal, which comes from UI console
		signal(SIGINT, SIG_IGN);
#endif

		// Wewnetrzna petla nieskonczona
		for (;;) {

			try {
				mp::common::mp_t->sr_ecp_msg->message("MP - wcisnij start");

				// Oczekiwanie na zlecenie START od UI
				mp::common::mp_t->wait_for_start();

				// Wyslanie START do wszystkich ECP
				mp::common::mp_t->start_all(mp::common::mp_t->robot_m);

				mp::common::mp_t->main_task_algorithm();

				// Oczekiwanie na STOP od UI
				mp::common::mp_t->wait_for_stop();

				// Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
				mp::common::mp_t->terminate_all (mp::common::mp_t->robot_m);
			}

			catch (lib::exception::NonFatal_error & e) {
				if(uint64_t const * err_code=boost::get_error_info<lib::exception::error_code>(e) ) {
					if(*err_code == ECP_STOP_ACCEPTED) {
						mp::common::mp_t->sr_ecp_msg->message("ECP STOP ACCEPTED");
					} else {
						mp::common::mp_t->sr_ecp_msg->message(lib::NON_FATAL_ERROR, *err_code);
						mp::common::mp_t->stop_and_terminate();
					}
				}
			}
		}
	}
	catch (lib::exception::Error_base & e) {  /* Dla zewnetrznej petli try*/
		// not sure if SR object still exists
		std::cerr << "Exception in MP:" << std::endl;
		std::cerr << diagnostic_information(e);
		mp::common::mp_t->sr_ecp_msg->message (lib::FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
	}

	catch (std::exception & e) {  /* Dla zewnetrznej petli try*/
		// not sure if SR object still exists
		std::cerr << e.what() << std::endl;
		mp::common::mp_t->sr_ecp_msg->message (lib::FATAL_ERROR, MP_UNIDENTIFIED_ERROR);
	}

	return (EXIT_FAILURE);
}
