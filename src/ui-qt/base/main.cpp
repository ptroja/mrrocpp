#include <cstdio>
#include <signal.h>
#include <sys/wait.h>
#include <QtGui/QApplication>

#include "interface.h"

mrrocpp::ui::common::Interface* interface;

/* Przechwycenie sygnalu */
void catch_signal(int sig)
{

	// print a message
	fprintf(stderr, "UI: %s\n", strsignal(sig));

	//std::cout << std::endl << std::endl << "catch_signal: " << interface->sigchld_handling << std::endl << std::endl;

	switch (sig)
	{
		case SIGINT:
			if (interface) {
				interface->UI_close();
			}
			break;
		case SIGALRM:
			break;
		case SIGSEGV:
			signal(SIGSEGV, SIG_DFL);
			break;
		case SIGCHLD:
			if ((interface) && (interface->check_sigchld_handling())) {
				//	std::cout << std::endl << std::endl << "catch_signal inside: " << interface->sigchld_handling
				// << std::endl << std::endl;
				interface->wait_for_child_termination(-1, false);
			}
			break;
		default:
			fprintf(stderr, "UI: unknown signal (%d)\n", sig);
	} // end: switch
}

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QLocale::setDefault(QLocale::English);
	interface = new mrrocpp::ui::common::Interface();

	interface->init();
	int r = a.exec();

//	std::cerr << "main: delete interface" << std::endl;
	delete interface;

// interface->UI_close();

	return r;
}
